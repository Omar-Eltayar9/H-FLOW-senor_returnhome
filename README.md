# H-FLOW-senor_returnhome for ULUDOĞAN SAVUNMA SANAYI Nano-Class Unmanned Helicopter
https://uludogantech.com/en/nano-class-unmanned-helicopter/ 
# PX4 H-FLOW Return-to-Home System

# GPS-Denied Breadcrumb Navigation System (PX4 Autopilot)

An autonomous "breadcrumb" trace-back navigation system designed for mini-helicopters and multicopters operating in GPS-denied environments. This project features a custom C++ PX4 firmware module that logs flight paths using optical flow telemetry and safely reverses the trajectory to return home upon GPS signal loss.

## 🚀 Overview

* **Autonomous Path Reversal:** Registers flight coordinates in real-time and traces the exact path backward when triggered by GPS failure or an RC switch.
* **GPS-Denied Estimation:** Utilizes an **H-FLOW** module via CAN bus for optical flow velocity fusion and downward distance sensing.
* **Collision Prevention:** Integrates 3x **TOF Laser Range Sensors Mini** (Front, Left, Right) to actively monitor obstacles and mitigate risks during the autonomous return phase.

---

## 🛠️ System Architecture

* **Flight Controller:** CUAV nano7 / PX4 Autopilot (v1.17.0)
* **Simulation Environment:** Gazebo Sim 8 (using `gz_x500_flow_baylands` for surface texture)
* **Ground Control:** QGroundControl & EdgeTX Companion GUI
* **State Estimator:** EKF2 (configured for Optical Flow + Range Finder fusion)

---

## 💻 How It Works (Firmware & Implementation)

1. **Custom PX4 Module (`flow_return`):** A custom background application written in ESP-IDF/PX4-compliant C++ that streams local NED position coordinates ($X, Y, Z$) into a ring buffer at **5 Hz (every 200 ms)**.
2. **RC Configuration (Channel 7):** A 3-position hardware switch maps the **HOME** coordinates on takeoff and activates the autonomous **RETURN** sequence hands-free.
3. **Offboard Trajectory Navigation:** When triggered, the module stops logging, filters the coordinates using a step-reduction factor, enters PX4 **Offboard Mode**, and safely commands the vehicle backward using precise waypoint trajectory setpoints.
4. **Obstacle Avoidance Integration:** Front and lateral TOF sensor distance values are parsed via MAVLink (`OBSTACLE_DISTANCE`) to halt or alter the trajectory if an obstacle blocks the reverse return path.

---

## 📊 Key Results & Validation

* **Stable GPS-Denied Fusion:** Successfully validated EKF2 state estimation tracking completely independent of GPS data.
* **High-Accuracy Path Tracking:** Achieved reliable reverse navigation tracking up to 500 coordinates with a tight **0.5-meter waypoint arrival acceptance radius**.
* **Automated Land Sequence:** Programmed smooth, automated transitions from Offboard return flight directly into `AUTO.LAND` with automatic motor disarm upon reaching the HOME station.
* **SITL Simulation Proven:** Fully tested within **Gazebo Sim 8** and monitored via **QGroundControl** with zero background thread crashes.

---

## 🔧 Critical PX4 Parameters

Configure the following parameters in QGroundControl to replicate the environment:

| Parameter | Value | Description |
| :--- | :--- | :--- |
| `EKF2_GPS_CTRL` | 0 | Disable GPS fusion |
| `EKF2_OF_CTRL` | 1 | Enable Optical Flow fusion |
| `EKF2_HGT_REF` | 1 | Set height reference to Range Finder (LiDAR) |
| `EKF2_RNG_CTRL` | 1 | Enable Range finder data |
| `SYS_HAS_GPS` | 0 | Configure vehicle as operating without GPS |

---


