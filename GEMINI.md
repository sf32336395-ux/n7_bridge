# N7 Autonomous Bridge - Technical Context

## 📋 Project Status (As of 2026-05-28)
Successfully integrated perception data with vehicle control. The workflow now supports end-to-end execution from `rosbag` data to N7 virtual control commands.

## 🧠 Architectural Logic
This module acts as the **Controller** and **Vehicle Actuator Interface** in the autonomous driving stack.

### 1. Perception to Control Mapping
*   **Data Source**: Transitioned from raw image processing to direct ROS 2 `/centerline` (Path) subscription.
*   **Coordinate System**: 
    *   **ROS/ISO-8855**: Uses X-forward, Y-left.
    *   **ZED SDK**: Uses Z-forward, X-right.
    *   **Correction**: Recent updates in `ros2_lane_follower.py` have fixed the mapping errors between ROS and ZED frames.
*   **Target Selection**: The bridge uses a Look-ahead Point derived from the `/centerline` Path.

### 2. Control Law: Pure Pursuit
Implemented in `ros2_lane_follower.py`:
$$\delta = \tan^{-1}\left(\frac{2L\sin(\alpha)}{L_{fw}}\right)$$
*   **Vehicle Wheelbase ($L$)**: $2.92m$.
*   **Look-ahead Distance ($L_{fw}$)**: Configurable via ROS parameters (Default: 4.0m - 6.0m).

### 3. Vehicle Actuation (UDS/DoIP)
Managed via `n7_control_bridge.py` and `foxtronpi-pyclient`:
*   **SWA (Steering Wheel Angle)**: $\delta \times 15.0$ (Steering Ratio). Left is positive (+), Right is negative (-).
*   **Communication**: Uses UDS/DoIP. `FoxPi_TP` maintains the Extended Diagnostic Session (0x03).
*   **Mock Mode**: Automatically activated if physical hardware is not detected.

## 🚀 Execution Guide

### 1. Environment Setup
```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/chan_baby/foxtronpi-pyclient
```

### 2. Running the System
1.  **Play Data**: 
    `ros2 bag play /home/chan_baby/Downloads/rosbag_test/rosbag2_2026_05_21-16_36_36/ --loop`
2.  **Start Control Node**:
    `python3 /home/chan_baby/Downloads/n7_bridge-main/ros2_lane_follower.py`
    *Optional: Adjust look-ahead distance:*
    `python3 ros2_lane_follower.py --ros-args -p look_ahead_distance:=6.0`

### 3. Visualization (RViz2)
*   **Fixed Frame**: `zed_left_camera_frame` (or `base_link`).
*   **Key Topics**:
    *   `/centerline` (Path) - Use bright color (Green).
    *   `/zed/zed_node/left/color/rect/image` (Image).
    *   `/cone_markers` (MarkerArray).

## 🛠️ Development Conventions
*   **Safety**: SWA capped at $\pm 450^\circ$.
*   **Diagnostic**: A status monitor is integrated into `ros2_lane_follower.py` to show real-time rosbag streaming status.
*   **Library Dependency**: Ensure `foxtronpi-pyclient` is in the `PYTHONPATH`.
