# 🛩️ Drone_FinalYear_Project   (DroneGCS)

An **open-source Ground Control System (GCS)** designed for an **autonomous UAV drone** that performs navigation, object detection, and obstacle avoidance using onboard and remote processing.  
This project integrates **Flask**, **SocketIO**, **OpenCV**, **Tkinter**, and **MAVLink** to enable real-time communication between the **Raspberry Pi-based drone** and the **ground control laptop**.

---

## 🚀 Overview

The **DroneGCS** project demonstrates an end-to-end control and monitoring system for an autonomous UAV.  
It features a **Raspberry Pi 3 B+** as the onboard control unit, equipped with a **Pi Camera** for real-time video streaming and **Flask-SocketIO** for telemetry exchange.  
The ground station (laptop) runs a **Tkinter-based GUI** that allows users to monitor flight status, view live video, and send control commands to the drone.

---

## 🧠 Key Features

- 🎥 **Live Video Streaming:** Real-time PiCam feed using Flask streaming.  
- ⚙️ **Flask + SocketIO Server:** Manages telemetry, MAVLink communication, and obstacle detection data.  
- 🧭 **Autonomous Navigation:** Path execution between points using camera and sensor inputs.  
- 🚧 **Obstacle Detection & Avoidance:** Uses OpenCV; objects within 2m are marked **red** (avoid), others **green**.  
- 🖥️ **Ground Control GUI:** Tkinter-based interface to monitor and control flight operations.  
- 📡 **Network Communication:** Raspberry Pi (drone) and Laptop (ground station) communicate over a common Wi-Fi network.  
- ✈️ **MAVLink Integration:** Sends/receives drone commands for manual or semi-autonomous operation.  

---

## 🧰 Tech Stack

**Hardware:**
- Raspberry Pi 3 B+
- Raspberry Pi Camera Module v1.3
- Drone flight controller (with MAVLink support)
- Sensors for obstacle detection (e.g., ultrasonic or LiDAR)
- Wi-Fi network for communication

**Software:**
- Python 3.x  
- Flask + Flask-SocketIO  
- OpenCV  
- Tkinter  
- MAVLink (DroneKit or pymavlink)  
- Threading, JSON, and Sockets for communication  

---

## 🕹️ Drone Control System (Flight Dynamics)

These are the **four main controls** that determine how a drone moves in 3D space:

| Control  | Description | RC Channel | PWM Range | Effect |
|-----------|-------------|-------------|-------------|---------|
| **Roll** | Controls the tilt of the drone left or right. | 1 | 1000–2000 | Left (-) / Right (+) |
| **Pitch** | Controls the tilt of the drone forward or backward. | 2 | 1000–2000 | Forward (-) / Backward (+) |
| **Throttle** | Controls the altitude of the drone. | 3 | 1000–2000 | Down (-) / Up (+) |
| **Yaw** | Controls the rotation of the drone left or right around its vertical axis. | 4 | 1000–2000 | Rotate Left (-) / Rotate Right (+) |

---

## 🔗 MAVLink Communication Setup

The Raspberry Pi communicates with the flight controller and ground station using **MAVLink**.  
The data link between the Raspberry Pi and Laptop is established over **UDP**, enabling real-time telemetry and heartbeat detection.

### 🧩 Step 1: Start MAVProxy on Raspberry Pi

    ```bash
    mavproxy.py --master=/dev/serial0 --baudrate 57600 --out udp:192.168.11.6:14550
- master=/dev/serial0: Connects to the flight controller through serial port.

- baudrate 57600: Sets communication speed.

- out udp:192.168.11.6:14550: Sends MAVLink messages to the laptop’s IP (192.168.11.6) at port 14550.

### 🧩 Step 2: Listen for MAVLink Heartbeat on Laptop
    from pymavlink import mavutil
    
    self.master = mavutil.mavlink_connection("udp:0.0.0.0:14550")
    print("Waiting for heartbeat...")
    self.master.wait_heartbeat()
    print("Connected!")
- The address udp:0.0.0.0:14550 allows the system to receive the drone heartbeat from any IP address.

- The ground station connects to port 14550 and waits for incoming drone data.

- Once connected, the heartbeat confirms that the drone and ground station are successfully communicating.

---

## 🧩 System Architecture

      ┌─────────────────────────────┐
      │      Ground Control GUI     │
      │        (Laptop - Tkinter)   │
      └──────────────┬──────────────┘
                     │ Flask-SocketIO
                     ▼
          ┌──────────────────────┐
          │ Raspberry Pi (Drone) │
          │ - Flask Server       │
          │ - Camera Stream      │
          │ - Obstacle Detection │
          │ - MAVLink Interface  │
          └──────────────────────┘
                     │
                     ▼
               Drone Hardware

---

## 🧠 How It Works
#### **Drone (Raspberry Pi)**

  - Captures live video feed from the Pi Camera.
  
  - Detects obstacles using OpenCV and distance thresholds.
  
  - Sends telemetry and obstacle data to the Flask-SocketIO server.
  
 #### **Ground Control (Laptop)**
  
 - Receives data and displays live video stream.
  
 - Provides manual control options and status visualization.
  
 - Communicates with the flight controller using MAVLink commands.

---

## 📈 Future Improvements

- Add GPS-based route planning and pathfinding algorithms.

- Integrate SLAM (Simultaneous Localization and Mapping) for indoor navigation.

- Build a web-based ground station interface for remote access.

- Include voice or gesture-based drone control.

---

