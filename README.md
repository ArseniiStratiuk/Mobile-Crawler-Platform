# Vepryk 6+ Mobile Crawler Platform

## Overview

The **Vepryk 6+** is a tracked mobile platform designed for high-mobility operations in challenging environments. This project implements a hierarchical control architecture that bridges the gap between operator intent and hardware execution.

The system employs a **Raspberry Pi 4** for high-level control (strategic tasks, data processing, operator communication) and an **ESP32** for low-level motor management, communicating via the **MAVLink** protocol over SSH-encrypted channels. The platform features a discrete ternary control scheme for robust command interpretation and an AprilTag-based vision subsystem for autonomous tracking and safety.

## System Architecture

The platform control system is built on a hierarchical architecture with two levels:

*   **High-Level Controller (Raspberry Pi 4 Model B):** Handles strategic tasks, data processing, operator communication, and command generation. It runs the custom control logic and wrapper library.
*   **Low-Level Controller (ESP32):** Responsible for real-time command execution, motor driver management, and stabilization. It runs firmware provided by the platform manufacturer and receives commands via serial interface (UART) using the MAVLink protocol.

### Software Stack
The software is designed around a producer-consumer pattern:
*   **Host Computer:** Runs a C++ control application (SDL2 for input, native sockets) that polls joystick state, applies quantization, encodes MAVLink messages, and transmits them via an SSH tunnel.
*   **Rover (Raspberry Pi):** Executes a C++17 bridge service that maintains a TCP server for network packets and a UART interface (115200 baud, 8N1) to the ESP32.

## Setup & Usage Instructions

### 1. Network Connection
Ensure both your host computer (laptop) and the Raspberry Pi are connected to the same network (e.g., sharing an internet connection or local hotspot).

### 2. Connect to the Platform (Raspberry Pi)
Open a terminal on your host computer and connect via SSH:

```bash
ssh platform@tank.local
```
**Password:** `best2025`

### 3. Initialize Onboard Systems
Once logged into the Raspberry Pi, execute the following commands to set up the environment and start the MAVLink bridge:

```bash
# Navigate to the project directory
cd Crawler

# Activate the Python virtual environment
source my_env/bin/activate

# Start MAVProxy to bridge UART to the network
mavproxy.py --master=/dev/ttyS0 --baudrate=57600
```

### 4. Establish Secure Tunnel (Host Computer)
On your local machine (host), open a **new terminal window**. Create an SSH tunnel to forward the video stream port (and potentially control ports) from the rover to your localhost.

```bash
ssh -L 12341:127.0.0.1:12341 platform@tank.local
```
*Enter the password `best2025` if prompted.*

### 5. Video Streaming & Control
To view the low-latency camera feed from the platform, run the following command on your host computer:

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop tcp://127.0.0.1:12341
```

> **Note:** Ensure that the camera software and MAVLink proxy are running on the Raspberry Pi before starting the GUI or video stream on your host machine.
