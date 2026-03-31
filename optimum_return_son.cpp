#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rc_channels.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <math.h>

static constexpr int   MAX_WP              = 500;
static constexpr int   LOG_HZ_MS           = 200;
static constexpr int   STEP                = 5;
static constexpr float RET_ALT             = -2.0f;
static constexpr float WP_RAD              = 0.5f;
static constexpr float WP_HOLD             = 0.1f;
static constexpr int   OFFBOARD_WARMUP     = 30;
static constexpr uint64_t OFFBOARD_WAIT_US = 3000000ULL;

// -----------------------------------------------------------------------
// RC Kanal 7 esik degerleri
// -----------------------------------------------------------------------
static constexpr int   RC_CH7_INDEX        = 6;
static constexpr float RC_THRESHOLD_LOW    = -0.5f;
static constexpr float RC_THRESHOLD_HIGH   =  0.5f;

enum class SwPos { UNKNOWN, DOWN, MID, UP };

static SwPos rc_ch7_position(float val)
{
    if (val < RC_THRESHOLD_LOW)  return SwPos::DOWN;
    if (val > RC_THRESHOLD_HIGH) return SwPos::UP;
    return SwPos::MID;
}

struct WP { float x, y, z, d; };

static WP   g_log[MAX_WP];
static int  g_cnt     = 0;
static bool g_logging = false;
static bool g_hset    = false;
static WP   g_home    = {};
static bool g_running = false;
static int  g_handle  = -1;

enum class Cmd { NONE, HOME, RETURN, STATUS, STOP };
static volatile Cmd g_cmd = Cmd::NONE;

// Persistent vehicle_command publisher
static orb_advert_t g_vcmd_pub = nullptr;

static void vcmd_publish(uint32_t c,
                         float p1=0, float p2=0, float p3=0,
                         float p4=0, float p5=0, float p6=0, float p7=0)
{
    struct vehicle_command_s m;
    memset(&m, 0, sizeof(m));
    m.timestamp        = hrt_absolute_time();
    m.command          = c;
    m.param1=p1; m.param2=p2; m.param3=p3;
    m.param4=p4; m.param5=p5; m.param6=p6; m.param7=p7;
    m.target_system    = 1;
    m.target_component = 1;
    m.source_system    = 1;
    m.source_component = 1;
    m.confirmation     = 0;
    m.from_external    = false;

    if (g_vcmd_pub == nullptr) {
        g_vcmd_pub = orb_advertise(ORB_ID(vehicle_command), &m);
    } else {
        orb_publish(ORB_ID(vehicle_command), g_vcmd_pub, &m);
    }
}

static bool switch_to_offboard(orb_advert_t ocm_pub,
                                orb_advert_t sp_pub,
                                float tx, float ty,
                                struct offboard_control_mode_s &ocm,
                                struct trajectory_setpoint_s   &sp)
{
    PX4_INFO("Offboard warm-up: %d setpoint yayinlaniyor...", OFFBOARD_WARMUP);

    for (int i = 0; i < OFFBOARD_WARMUP; i++) {
        ocm.timestamp  = hrt_absolute_time();
        ocm.position   = true;
        orb_publish(ORB_ID(offboard_control_mode), ocm_pub, &ocm);

        sp.timestamp   = hrt_absolute_time();
        sp.position[0] = tx;
        sp.position[1] = ty;
        sp.position[2] = RET_ALT;
        sp.yaw         = NAN;
        orb_publish(ORB_ID(trajectory_setpoint), sp_pub, &sp);

        px4_usleep(50000);
    }

    PX4_INFO("MAV_CMD_DO_SET_MODE (offboard) gonderiliyor...");
    vcmd_publish(176, 1.0f, 6.0f, 0.0f);

    int status_sub = orb_subscribe(ORB_ID(vehicle_status));
    struct vehicle_status_s vstatus;
    memset(&vstatus, 0, sizeof(vstatus));

    uint64_t t_start  = hrt_absolute_time();
    uint64_t last_cmd = t_start;
    bool offboard_ok  = false;

    while (hrt_absolute_time() - t_start < OFFBOARD_WAIT_US) {
        ocm.timestamp  = hrt_absolute_time();
        orb_publish(ORB_ID(offboard_control_mode), ocm_pub, &ocm);
        sp.timestamp   = hrt_absolute_time();
        orb_publish(ORB_ID(trajectory_setpoint), sp_pub, &sp);

        bool upd = false;
        orb_check(status_sub, &upd);
        if (upd) {
            orb_copy(ORB_ID(vehicle_status), status_sub, &vstatus);
            if (vstatus.nav_state == 14) {
                offboard_ok = true;
                break;
            }
        }

        uint64_t now_us = hrt_absolute_time();
        if (now_us - last_cmd > 500000ULL) {
            vcmd_publish(176, 1.0f, 6.0f, 0.0f);
            last_cmd = now_us;
        }

        px4_usleep(50000);
    }

    orb_unsubscribe(status_sub);

    if (offboard_ok) {
        PX4_INFO("Offboard mod AKTIF (nav_state=14)");
    } else {
        PX4_WARN("Offboard gecisi tamamlanamadi (nav_state=%d), devam ediliyor...",
                 (int)vstatus.nav_state);
    }

    return offboard_ok;
}

static constexpr uint64_t LAND_DISARM_TIMEOUT_US = 30000000ULL;  // 30sn disarm bekleme

static void perform_land(orb_advert_t &ocm_pub, orb_advert_t &sp_pub,
                         struct offboard_control_mode_s &ocm,
                         struct trajectory_setpoint_s   &sp,
                         float hold_x, float hold_y)
{
    PX4_INFO("[LAND] LAND modu ile inis basliyor...");
    PX4_INFO("[LAND] Son konum: x=%.2f y=%.2f", (double)hold_x, (double)hold_y);

    // ==================================================================
    // ADIM 0: LAND moduna gecmeden once 3 saniye mevcut konumda bekle.
    //         Offboard->LAND gecisindeki ani ivme kaynaklı sapmayi onler.
    // ==================================================================
    PX4_INFO("[LAND] Stabilizasyon bekleniyor (3sn)...");
    {
        uint64_t t_stab = hrt_absolute_time();
        while (hrt_absolute_time() - t_stab < 3000000ULL) {
            ocm.timestamp  = hrt_absolute_time();
            ocm.position   = true;
            orb_publish(ORB_ID(offboard_control_mode), ocm_pub, &ocm);

            sp.timestamp   = hrt_absolute_time();
            sp.position[0] = hold_x;
            sp.position[1] = hold_y;
            sp.position[2] = RET_ALT;
            sp.yaw         = NAN;
            orb_publish(ORB_ID(trajectory_setpoint), sp_pub, &sp);

            px4_usleep(50000);  // 50ms
        }
    }
    PX4_INFO("[LAND] Stabilizasyon tamamlandi, LAND moduna geciliyor...");

    // ==================================================================
    // ADIM 1: Offboard setpointleri yayinlamaya devam ederken
    //         MAV_CMD_DO_SET_MODE ile LAND moduna gec
    //         (176: MAV_CMD_DO_SET_MODE, param1=1 custom, param2=9 AUTO.LAND)
    // ==================================================================
    PX4_INFO("[LAND] MAV_CMD_DO_SET_MODE -> AUTO.LAND (custom_mode=9) gonderiliyor...");

    // LAND moduna gecis: base_mode=1 (custom), custom_main_mode=4 (AUTO),
    // custom_sub_mode=6 (AUTO.LAND) — PX4 LAND modu
    // param1=1, param2=4, param3=6
    vcmd_publish(176, 1.0f, 4.0f, 6.0f);

    // LAND modunun aktif olmasini dogrula
    int status_sub = orb_subscribe(ORB_ID(vehicle_status));
    struct vehicle_status_s vstatus;
    memset(&vstatus, 0, sizeof(vstatus));

    uint64_t t_start  = hrt_absolute_time();
    uint64_t last_cmd = t_start;
    bool land_ok      = false;

    // LAND moduna girilene kadar hem setpoint hem mod komutu gonder
    while (hrt_absolute_time() - t_start < 5000000ULL) {  // 5sn bekleme
        // Offboard setpointleri surdurelim (mod gecisi gerceklesmeden kesersek failsafe olabilir)
        ocm.timestamp  = hrt_absolute_time();
        ocm.position   = true;
        orb_publish(ORB_ID(offboard_control_mode), ocm_pub, &ocm);

        sp.timestamp   = hrt_absolute_time();
        sp.position[0] = hold_x;
        sp.position[1] = hold_y;
        sp.position[2] = RET_ALT;
        sp.yaw         = NAN;
        orb_publish(ORB_ID(trajectory_setpoint), sp_pub, &sp);

        bool upd = false;
        orb_check(status_sub, &upd);
        if (upd) {
            orb_copy(ORB_ID(vehicle_status), status_sub, &vstatus);
            // nav_state == 17: AUTO.LAND modu (PX4 vehicle_status)
            if (vstatus.nav_state == 17) {
                land_ok = true;
                PX4_INFO("[LAND] LAND modu AKTIF (nav_state=17)");
                break;
            }
        }

        // Her 500ms'de bir mod komutunu tekrarla
        uint64_t now_us = hrt_absolute_time();
        if (now_us - last_cmd > 500000ULL) {
            vcmd_publish(176, 1.0f, 4.0f, 6.0f);
            last_cmd = now_us;
        }

        px4_usleep(50000);
    }

    orb_unsubscribe(status_sub);

    if (!land_ok) {
        PX4_WARN("[LAND] LAND moduna gecilemedi (nav_state=%d), yine de devam ediliyor...",
                 (int)vstatus.nav_state);
    }

    // ==================================================================
    // ADIM 2: Offboard yayinlarini durdur (LAND modu artik kontrolu aldi)
    // ==================================================================
    PX4_INFO("[LAND] Offboard yayinlari durduruluyor...");
    if (sp_pub  != nullptr) { orb_unadvertise(sp_pub);  sp_pub  = nullptr; }
    if (ocm_pub != nullptr) { orb_unadvertise(ocm_pub); ocm_pub = nullptr; }

    // ==================================================================
    // ADIM 3: Aracin yere inmesini ve otomatik disarm'i bekle
    //         PX4, LAND modunda yere indikten sonra otomatik disarm yapar.
    // ==================================================================
    PX4_INFO("[LAND] Inis ve otomatik disarm bekleniyor (max 30sn)...");

    int pos_sub2 = orb_subscribe(ORB_ID(vehicle_local_position));
    int status_sub2 = orb_subscribe(ORB_ID(vehicle_status));
    struct vehicle_local_position_s pos2;
    struct vehicle_status_s vstatus2;
    memset(&pos2,    0, sizeof(pos2));
    memset(&vstatus2, 0, sizeof(vstatus2));

    uint64_t t_land   = hrt_absolute_time();
    uint64_t last_log = t_land;

    while (hrt_absolute_time() - t_land < LAND_DISARM_TIMEOUT_US) {
        bool upd = false;

        orb_check(pos_sub2, &upd);
        if (upd) orb_copy(ORB_ID(vehicle_local_position), pos_sub2, &pos2);

        upd = false;
        orb_check(status_sub2, &upd);
        if (upd) orb_copy(ORB_ID(vehicle_status), status_sub2, &vstatus2);

        // Arma durumu: arming_state == 1 → STANDBY (disarm oldu)
        if (vstatus2.arming_state == 1) {
            PX4_INFO("[LAND] Otomatik DISARM tamamlandi (arming_state=STANDBY)");
            break;
        }

        uint64_t now_us = hrt_absolute_time();
        if (now_us - last_log > 2000000ULL) {
            last_log = now_us;
            PX4_INFO("[LAND]   iniyor... z=%.2f arming_state=%d",
                     (double)pos2.z, (int)vstatus2.arming_state);
        }

        px4_usleep(100000);  // 100ms
    }

    orb_unsubscribe(pos_sub2);
    orb_unsubscribe(status_sub2);

    PX4_INFO("[LAND] Tamamlandi!");
}

static void do_return()
{
    if (g_cnt < 5) { PX4_WARN("Log yok (%d nokta)", g_cnt); return; }
    g_logging = false;
    PX4_INFO("Geri donus: %d nokta", g_cnt);

    static WP wps[MAX_WP / STEP + 2];
    int wc = 0;
    for (int i = g_cnt - 1; i >= 0; i -= STEP) {
        wps[wc++] = g_log[i];
        if (wc >= (int)(MAX_WP / STEP)) break;
    }
    wps[wc++] = g_home;
    PX4_INFO("%d waypoint", wc);

    struct offboard_control_mode_s ocm;
    memset(&ocm, 0, sizeof(ocm));
    ocm.position  = true;
    ocm.timestamp = hrt_absolute_time();
    orb_advert_t ocm_pub = orb_advertise(ORB_ID(offboard_control_mode), &ocm);

    struct trajectory_setpoint_s sp;
    memset(&sp, 0, sizeof(sp));
    sp.timestamp   = hrt_absolute_time();
    sp.position[0] = wps[0].x;
    sp.position[1] = wps[0].y;
    sp.position[2] = RET_ALT;
    sp.yaw         = NAN;
    orb_advert_t sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &sp);

    int pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    struct vehicle_local_position_s pos;
    memset(&pos, 0, sizeof(pos));

    bool ob = switch_to_offboard(ocm_pub, sp_pub, wps[0].x, wps[0].y, ocm, sp);
    if (!ob) {
        PX4_WARN("Offboard gecisi tamamlanamadi, yine de devam ediliyor.");
    }

    // Waypoint dongusu
    for (int i = 0; i < wc; i++) {
        float tx = wps[i].x, ty = wps[i].y;
        PX4_INFO("[%d/%d] hedef x=%.2f y=%.2f z=%.2f",
                 i + 1, wc, (double)tx, (double)ty, (double)RET_ALT);

        uint64_t t0 = hrt_absolute_time();
        while (hrt_absolute_time() - t0 < 15000000ULL) {
            ocm.timestamp  = hrt_absolute_time();
            ocm.position   = true;
            orb_publish(ORB_ID(offboard_control_mode), ocm_pub, &ocm);

            sp.timestamp   = hrt_absolute_time();
            sp.position[0] = tx;
            sp.position[1] = ty;
            sp.position[2] = RET_ALT;
            sp.yaw         = NAN;
            orb_publish(ORB_ID(trajectory_setpoint), sp_pub, &sp);

            bool upd = false;
            orb_check(pos_sub, &upd);
            if (upd) orb_copy(ORB_ID(vehicle_local_position), pos_sub, &pos);

            float dx = pos.x - tx, dy = pos.y - ty, dz = pos.z - RET_ALT;
            if (sqrtf(dx * dx + dy * dy + dz * dz) < WP_RAD) {
                PX4_INFO("  [%d] WP ulasildi", i + 1);
                break;
            }
            px4_usleep(50000);
        }
        px4_usleep((uint32_t)(WP_HOLD * 1e6f));
    }

    // ---------------------------------------------------------------
    // DUZELTME v3:
    // Onceki: Offboard modda kademeli z setpointi dusurulerek inis yapiliyordu.
    // Yeni:   Setpointler AKARKEN MAV_CMD_DO_SET_MODE ile LAND moduna gecilir,
    //         PX4 kendi inis algoritmasini calistirir — failsafe tetiklenmez.
    // ---------------------------------------------------------------
    PX4_INFO("Tum waypoint'lere ulasildi, inis baslatiliyor...");
    perform_land(ocm_pub, sp_pub, ocm, sp,
                 wps[wc - 1].x, wps[wc - 1].y);

    // perform_land icinde unadvertise yapildi, tekrar yapma
    orb_unsubscribe(pos_sub);
    PX4_INFO("Tamamlandi!");
}

// -----------------------------------------------------------------------
// HOME komutunu calistir
// -----------------------------------------------------------------------
static void do_home(int pos_sub)
{
    struct vehicle_local_position_s pos;
    memset(&pos, 0, sizeof(pos));
    bool u = false;
    orb_check(pos_sub, &u);
    if (u) orb_copy(ORB_ID(vehicle_local_position), pos_sub, &pos);
    g_home    = {pos.x, pos.y, pos.z, 0};
    g_hset    = true;
    g_cnt     = 0;
    g_logging = true;
    PX4_INFO("HOME: x=%.2f y=%.2f z=%.2f",
             (double)pos.x, (double)pos.y, (double)pos.z);
    PX4_INFO("Loglama basladi");
}

static int task_main(int argc, char *argv[])
{
    PX4_INFO("flow_return basladi.");
    g_running = true;

    int pos_sub  = orb_subscribe(ORB_ID(vehicle_local_position));
    int dist_sub = orb_subscribe(ORB_ID(distance_sensor));
    int rc_sub   = orb_subscribe(ORB_ID(rc_channels));

    struct vehicle_local_position_s pos;
    struct distance_sensor_s        dist;
    struct rc_channels_s            rc;

    memset(&pos,  0, sizeof(pos));
    memset(&dist, 0, sizeof(dist));
    memset(&rc,   0, sizeof(rc));

    uint64_t last     = 0;

    SwPos prev_sw    = SwPos::UNKNOWN;
    bool return_armed = false;

    while (g_running) {

        // ------------------------------------------------------------------
        // 1) Manuel komut isleme
        // ------------------------------------------------------------------
        Cmd cmd = g_cmd; g_cmd = Cmd::NONE;

        if (cmd == Cmd::HOME) {
            do_home(pos_sub);

        } else if (cmd == Cmd::RETURN) {
            do_return();
            return_armed = false;

        } else if (cmd == Cmd::STATUS) {
            bool u = false;
            orb_check(pos_sub, &u);
            if (u) orb_copy(ORB_ID(vehicle_local_position), pos_sub, &pos);
            PX4_INFO("HOME:%s LOG:%s NOKTA:%d x=%.2f y=%.2f",
                     g_hset    ? "EVET" : "HAYIR",
                     g_logging ? "AKTIF" : "DURDU",
                     g_cnt, (double)pos.x, (double)pos.y);

        } else if (cmd == Cmd::STOP) {
            g_running = false;
            break;
        }

        // ------------------------------------------------------------------
        // 2) RC Kanal 7 switch okuma
        // ------------------------------------------------------------------
        {
            bool rc_upd = false;
            orb_check(rc_sub, &rc_upd);
            if (rc_upd) {
                orb_copy(ORB_ID(rc_channels), rc_sub, &rc);
            }

            float ch7_val = rc.channels[RC_CH7_INDEX];
            SwPos cur_sw  = rc_ch7_position(ch7_val);

            if (cur_sw != prev_sw) {

                if (cur_sw == SwPos::DOWN) {
                    PX4_INFO("[CH7] ASAGI -> HOME komutu tetiklendi");
                    do_home(pos_sub);
                    return_armed = true;

                } else if (cur_sw == SwPos::MID) {
                    PX4_INFO("[CH7] ORTA -> beklemede");

                } else if (cur_sw == SwPos::UP) {
                    if (return_armed) {
                        PX4_INFO("[CH7] USTE -> RETURN komutu tetiklendi");
                        do_return();
                        return_armed = false;
                    } else {
                        PX4_WARN("[CH7] USTE -> Once ASAGI konumuna alin (HOME)");
                    }
                }

                prev_sw = cur_sw;
            }
        }

        // ------------------------------------------------------------------
        // 3) Pozisyon loglama
        // ------------------------------------------------------------------
        if (g_logging) {
            uint64_t now = hrt_absolute_time();
            if (now - last >= (uint64_t)(LOG_HZ_MS * 1000)) {
                last = now;
                bool u = false;
                orb_check(pos_sub,  &u);
                if (u) orb_copy(ORB_ID(vehicle_local_position), pos_sub,  &pos);
                orb_check(dist_sub, &u);
                if (u) orb_copy(ORB_ID(distance_sensor),        dist_sub, &dist);
                if (g_cnt < MAX_WP) {
                    g_log[g_cnt++] = {pos.x, pos.y, pos.z, dist.current_distance};
                } else {
                    PX4_WARN("Buffer doldu");
                    g_logging = false;
                }
            }
        }

        px4_usleep(50000);
    }

    if (g_vcmd_pub != nullptr) {
        orb_unadvertise(g_vcmd_pub);
        g_vcmd_pub = nullptr;
    }
    orb_unsubscribe(pos_sub);
    orb_unsubscribe(dist_sub);
    orb_unsubscribe(rc_sub);
    PX4_INFO("flow_return durdu.");
    return 0;
}

extern "C" __EXPORT int flow_return_main(int argc, char *argv[])
{
    if (argc < 2) {
        PX4_INFO("Kullanim: flow_return start|home|return|status|stop");
        PX4_INFO("  start  : gorevi baslat (RC CH7 izleme aktif)");
        PX4_INFO("  home   : mevcut konumu HOME olarak kaydet");
        PX4_INFO("  return : kayitli yolu tersine izleyerek eve don");
        PX4_INFO("  status : mevcut durumu goster");
        PX4_INFO("  stop   : gorevi durdur");
        return 0;
    }

    if (!strcmp(argv[1], "start")) {
        if (g_running) { PX4_WARN("Zaten calisiyor"); return 0; }
        g_handle = px4_task_spawn_cmd("flow_return",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_DEFAULT,
                                      2048,
                                      task_main,
                                      nullptr);
        return g_handle > 0 ? 0 : -1;

    } else if (!strcmp(argv[1], "home")) {
        if (!g_running) { PX4_WARN("Once 'start' calistir"); return -1; }
        g_cmd = Cmd::HOME;

    } else if (!strcmp(argv[1], "return")) {
        if (!g_running) { PX4_WARN("Once 'start' calistir"); return -1; }
        g_cmd = Cmd::RETURN;

    } else if (!strcmp(argv[1], "status")) {
        if (!g_running) { PX4_WARN("Calısmiyor"); return -1; }
        g_cmd = Cmd::STATUS;

    } else if (!strcmp(argv[1], "stop")) {
        g_cmd    = Cmd::STOP;
        g_handle = -1;
    }

    return 0;
}
