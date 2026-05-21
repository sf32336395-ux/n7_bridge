# N7 Autonomous Bridge - Technical Context

## 🧠 Architectural Logic
This module acts as the **Controller** and **Vehicle Actuator Interface** in the autonomous driving stack.

### 1. Perception to Control Mapping
*   **Coordinate System**: 
    *   **ZED SDK**: Uses a Left-Handed coordinate system by default (Z-forward, X-right).
    *   **Bridge Logic**: Maps ZED coordinates to the vehicle's ISO-8855 frame (X-forward, Y-left). 
    *   *Transformation*: `Vehicle_X = ZED_Z`, `Vehicle_Y = -ZED_X`.
*   **Target Selection**: The bridge expects a "Look-ahead Point" from the `zed_yolo_tf` module, typically chosen on the centerline at a distance $L_{fw}$ (adaptive or fixed).

### 2. Control Law: Pure Pursuit
The steering angle $\delta$ is calculated based on the kinematic bicycle model:
$$\delta = \tan^{-1}\left(\frac{2L\sin(\alpha)}{L_{fw}}\right)$$
Where:
*   $L$: Vehicle wheelbase ($2.92m$ for N7).
*   $\alpha$: Angle between the vehicle's heading and the look-ahead point.
*   $L_{fw}$: Look-ahead distance.

### 3. Vehicle Actuation (UDS/DoIP)
The bridge encapsulates the **14-parameter control list** required by the `FoxPi_write.FoxPi_Driving_Ctrl` method:
*   **SWA (Steering Wheel Angle)**: Calculated as $\delta \times 15.0$ (Steering Ratio).
*   **Speed**: Enforced via `TargetSpd` and `APSSpeedCMD` parameters.
*   **Session Management**: `FoxPi_TP` runs in a background thread to maintain the **Extended Diagnostic Session (0x03)** and handle **Security Access (0x27)**.

## 🐧 Linux Deployment Guide
### Environment Setup
1.  Ensure you have `python3-pip` and `python3-venv`.
2.  It is recommended to run inside a virtual environment:
    ```bash
    python3 -m venv n7_env
    source n7_env/bin/activate
    pip install doipclient udsoncan numpy
    ```

### Network Troubleshooting
*   The N7 DoIP gateway typically expects the client to be on `192.168.1.XX`.
*   Use `ping 192.168.1.10` to verify the connection to the vehicle.
*   If the vehicle is not responding to UDS commands, check if another diagnostic tool is already holding the session.

## 🛠️ Development Conventions
*   **Safety Limits**: SWA is hard-capped at $\pm 450^\circ$.
*   **Mock Mode**: If `doipclient` or `udsoncan` are not installed, the bridge automatically falls back to a mock mode that prints signals to `stdout`.
