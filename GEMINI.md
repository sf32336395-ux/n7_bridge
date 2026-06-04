# N7 Autonomous Bridge - Technical Context

## 📋 Project Status (As of 2026-06-04)
Successfully implemented and stabilized the autonomous steering control stack. The system now features high-precision path tracking and smooth actuation commands suitable for N7 vehicle dynamics.

## 🧠 Architectural Logic
This module acts as the **Controller** and **Vehicle Actuator Interface**, bridging ROS 2 perception data with UDS/DoIP vehicle protocols.

### 1. Control Law: Pure Pursuit
The steering angle $\delta$ is calculated based on the kinematic bicycle model:
$$\delta = \tan^{-1}\left(\frac{2L\sin(\alpha)}{L_{fw}}\right)$$
*   **Vehicle Wheelbase ($L$)**: $2.92m$ (Fixed for N7).
*   **Look-ahead Distance ($L_{fw}$)**: Configurable via ROS parameters (Default: $5.0m - 7.0m$).

### 2. Stability Enhancements (New)
To address steering instability and perception noise, two major upgrades were implemented:
*   **Path Interpolation**: Instead of selecting the "closest" point, the system now performs linear interpolation between ROS path points to find a precise look-ahead target at exactly $L_{fw}$.
*   **SWA Low-Pass Filter (LPF)**: Implemented an exponential smoothing filter to prevent sharp steering jumps.
    *   Formula: $SWA_{out} = \alpha \cdot SWA_{new} + (1 - \alpha) \cdot SWA_{old}$
    *   Default $\alpha$: $0.1$ (High smoothing).

### 3. Coordinate Systems
*   **ROS (ISO-8855)**: $X$-forward, $Y$-left.
*   **Bridge Input**: Expects relative coordinates where $X$ is lateral and $Z$ is longitudinal.
*   **Mapping**: `Vehicle_X = Target_Z`, `Vehicle_Y = -Target_X`.

## 🚀 Execution Guide

### 1. Environment Setup
```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/chan_baby/foxtronpi-pyclient
```

### 2. Running the System
```bash
# Start the follower node with custom stability parameters
python3 ros2_lane_follower.py --ros-args -p look_ahead_distance:=6.5 -p smoothing_alpha:=0.1
```

## 🛠️ Debugging & Validation
*   **Precision Logging**: Target coordinates are logged with 4-decimal precision to monitor micro-movements in the perception stack.
*   **Safety Limits**: Steering Wheel Angle (SWA) is hard-capped at $\pm 450^\circ$.
*   **Mock Mode**: Automatically activated if DoIP/UDS libraries or hardware are missing.

---
*This document is maintained via Gemini CLI to reflect the latest workspace state.*
