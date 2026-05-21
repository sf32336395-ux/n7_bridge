# N7 Autonomous Cone Following Bridge

This project provides a bridge between **ZED 2i visual perception** (centerline detection) and the **Foxtron N7 vehicle control interface** (UDS/DoIP).

## 🚀 Overview
The bridge listens to 3D centerline coordinates from the camera, calculates the required steering angle using the **Pure Pursuit** algorithm, and sends encrypted UDS commands to the N7's ECU to actuate the steering and maintain a constant speed.

## 🛠️ Requirements (Linux)
Ensure your Linux machine (Ubuntu 22.04 recommended) has the following:
*   **Python 3.10+**
*   **Dependencies**:
    ```bash
    pip install doipclient udsoncan numpy
    ```
*   **Hardware**: Ethernet connection to the N7 vehicle's OBD-II/DoIP gateway.
*   **IP Configuration**: Set your machine's static IP to the same subnet as the vehicle (e.g., `192.168.1.XX`).

## 📁 File Structure
*   `n7_control_bridge.py`: The core logic that converts vision data to vehicle commands.
*   `integration_example.py`: A template showing how to plug this bridge into your vision loop.
*   `FoxPi_write.py`, `FoxPi_TP.py`, `FoxPi_read.py`: Low-level FoxtronPi driver libraries (must be in the same folder).

## 🎮 How to Run
1.  **Test the Logic (Mock Mode)**:
    ```bash
    python3 integration_example.py
    ```
    *This will print calculated steering angles to the console without needing a vehicle connection.*

2.  **Real Vehicle Test**:
    *   Connect the Ethernet cable.
    *   Ensure the vehicle is in a safe, open area.
    *   Modify `DOIP_SERVER_IP` in `n7_control_bridge.py` if necessary.
    *   Run the script and monitor the vehicle's response.

## ⚠️ Safety Warning
*   **Low Speed Only**: Initial target speed is restricted to 5 km/h.
*   **Manual Override**: The driver must remain in the seat at all times. Tapping the brake or grabbing the steering wheel should trigger the vehicle's safety override.
*   **Emergency Stop**: Press `Ctrl+C` in the terminal to immediately stop the control signal and release the diagnostic session.

---
Developed for the 2026 Design and Practice of Intelligent Vehicles course.
