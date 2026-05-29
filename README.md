# Autonomous Warehouse Inspection Robot
## Complete Build, Run, and Debug Guide
### ROS2 Jazzy + Gazebo Harmonic on Ubuntu 24.04

---

## TABLE OF CONTENTS

1. [Prerequisites](#1-prerequisites)
2. [Workspace Setup](#2-workspace-setup)
3. [Build Instructions](#3-build-instructions)
4. [Run Instructions](#4-run-instructions)
5. [Topic Reference](#5-topic-reference)
6. [TF Tree Reference](#6-tf-tree-reference)
7. [PID Tuning Guide](#7-pid-tuning-guide)
8. [Debugging Instructions](#8-debugging-instructions)
9. [Architecture Summary](#9-architecture-summary)

---

## 1. PREREQUISITES

Verify your environment before building:

```bash
# Confirm ROS2 Jazzy
echo $ROS_DISTRO          # should print: jazzy

# Confirm Gazebo Harmonic
gz sim --version           # should print: Gazebo Sim, version 8.x.x

# Confirm Python 3.12
python3 --version          # should print: Python 3.12.x

# Install missing dependencies if needed
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  python3-colcon-common-extensions
```

---

## 2. WORKSPACE SETUP

The project files are already in:
```
~/ros2_ws/src/warehouse_wall_follower/
```

If you need to copy them from another location:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# (files are already here from generation)
```

---

## 3. BUILD INSTRUCTIONS

```bash
# Step 1: Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Step 2: Navigate to workspace root
cd ~/ros2_ws

# Step 3: Install Python dependencies via rosdep
rosdep install --from-paths src --ignore-src -r -y

# Step 4: Build the package
colcon build --packages-select warehouse_wall_follower --symlink-install

# Expected output:
# Starting >>> warehouse_wall_follower
# Finished <<< warehouse_wall_follower [Xs]
# Summary: 1 packages finished [Xs]

# Step 5: Source the workspace overlay
source install/setup.bash

# IMPORTANT: You must source BOTH base ROS2 AND the workspace:
# source /opt/ros/jazzy/setup.bash   ← base ROS2
# source ~/ros2_ws/install/setup.bash ← your workspace
#
# Add both to ~/.bashrc for convenience:
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Rebuild after changes

```bash
cd ~/ros2_ws
colcon build --packages-select warehouse_wall_follower --symlink-install
source install/setup.bash
```

With `--symlink-install`, Python file changes take effect immediately
without rebuilding (symlinks point to source). For URDF, YAML, and SDF
changes, a rebuild is required to copy files to the install directory.

---

## 4. RUN INSTRUCTIONS

### 4.1 Full System Launch (One Command)

```bash
# Source workspace first (if not in .bashrc)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch everything
ros2 launch warehouse_wall_follower warehouse_launch.py
```

**What happens:**
- t=0s   : Gazebo Harmonic starts, loads warehouse.sdf
- t=0s   : robot_state_publisher starts
- t=5s   : Robot spawns in warehouse at position (-14, 0, 0.15)
- t=6s   : ros_gz_bridge starts (connects Gazebo ↔ ROS2 topics)
- t=7s   : RViz2 opens with robot model and LiDAR scan
- t=8s   : wall_detector and pid_controller nodes start
- t=9s   : state_machine starts → robot begins SEARCH_WALL → FOLLOW_WALL

### 4.2 Launch with Custom Spawn Position

```bash
ros2 launch warehouse_wall_follower warehouse_launch.py \
  robot_x:=0.0 \
  robot_y:=0.0 \
  robot_yaw:=1.5708
```

### 4.3 Launch without RViz (headless mode)

```bash
ros2 launch warehouse_wall_follower warehouse_launch.py use_rviz:=false
```

### 4.4 Debug Launch (ROS2 nodes only, no Gazebo)

```bash
ros2 launch warehouse_wall_follower debug_launch.py
```

---

## 5. TOPIC REFERENCE

### Topics published by ROS2 nodes

| Topic              | Type                      | Node           | Description                        |
|--------------------|---------------------------|----------------|------------------------------------|
| `/wall_distance`   | `std_msgs/Float64`        | wall_detector  | Filtered right-wall distance (m)   |
| `/wall_info`       | `std_msgs/String`         | wall_detector  | Wall detection diagnostics JSON    |
| `/cmd_vel`         | `geometry_msgs/Twist`     | pid_controller | Velocity command to robot          |
| `/cmd_vel`         | `geometry_msgs/Twist`     | state_machine  | Direct command in non-PID states   |
| `/pid_enabled`     | `std_msgs/Bool`           | state_machine  | Enable/disable PID controller      |
| `/robot_state`     | `std_msgs/String`         | state_machine  | Current FSM state name             |
| `/pid_diagnostics` | `std_msgs/String`         | pid_controller | PID internal values JSON           |
| `/fsm_diagnostics` | `std_msgs/String`         | state_machine  | FSM status JSON                    |
| `/metrics`         | `std_msgs/String`         | metrics_node   | Performance statistics JSON        |

### Topics bridged from Gazebo

| ROS2 Topic    | Type                      | Source                  | Description             |
|---------------|---------------------------|-------------------------|-------------------------|
| `/scan`       | `sensor_msgs/LaserScan`   | Gazebo GPU LiDAR        | 360° LiDAR scan         |
| `/imu/data`   | `sensor_msgs/Imu`         | Gazebo IMU sensor       | IMU readings            |
| `/odom`       | `nav_msgs/Odometry`       | Gazebo DiffDrive plugin | Wheel odometry          |
| `/joint_states`| `sensor_msgs/JointState` | Gazebo JointStatePub    | Wheel joint positions   |
| `/tf`         | `tf2_msgs/TFMessage`      | Gazebo DiffDrive plugin | odom→base_footprint TF  |
| `/clock`      | `rosgraph_msgs/Clock`     | Gazebo                  | Simulation time         |

### Monitor topics in real time

```bash
# Watch wall distance
ros2 topic echo /wall_distance

# Watch FSM state
ros2 topic echo /robot_state

# Watch PID diagnostics (formatted)
ros2 topic echo /pid_diagnostics | python3 -c "
import sys, json
for line in sys.stdin:
    if line.strip().startswith('data:'):
        data = line.split('data: ',1)[1].strip().strip(\"'\")
        try:
            d = json.loads(data)
            print(f\"e={d['error']:+.4f}  P={d['p_term']:+.4f}  I={d['i_term']:+.4f}  D={d['d_term']:+.4f}  cmd_ang={d['angular_z']:+.4f}\")
        except: pass
"

# Watch metrics
ros2 topic echo /metrics

# View all active topics
ros2 topic list

# Check topic frequencies
ros2 topic hz /scan            # should be ~10 Hz
ros2 topic hz /wall_distance   # should be ~10 Hz
ros2 topic hz /cmd_vel         # should be ~20 Hz
```

---

## 6. TF TREE REFERENCE

```
odom  (world frame, set at robot spawn)
  └── base_footprint  (ground projection, published by DiffDrive via bridge)
        └── base_link  (wheel axle height, published by robot_state_publisher)
              ├── chassis_link       (main body, fixed)
              ├── left_wheel_link    (driven, continuous joint)
              ├── right_wheel_link   (driven, continuous joint)
              ├── caster_front_link  (passive, fixed)
              ├── caster_rear_link   (passive, fixed)
              ├── lidar_link         (LiDAR frame, fixed)
              └── imu_link           (IMU frame, fixed)
```

Verify TF tree:
```bash
ros2 run tf2_tools view_frames
# Creates frames.pdf in current directory

ros2 topic echo /tf --once
```

---

## 7. PID TUNING GUIDE

The PID parameters are in `config/robot_params.yaml` under `pid_controller`.

### Tuning procedure

**Step 1: Start with proportional only**
```yaml
pid_controller:
  ros__parameters:
    kp: 1.0
    ki: 0.0
    kd: 0.0
```
Increase `kp` until the robot oscillates, then reduce by 30%.

**Step 2: Add derivative to damp oscillations**
```yaml
    kd: 0.5   # increase until oscillation damps
```

**Step 3: Add integral to remove steady-state offset**
```yaml
    ki: 0.05  # small value; increase slowly
```

**Quick re-tuning without rebuild** (since --symlink-install is used):
```bash
# Edit the YAML
nano ~/ros2_ws/src/warehouse_wall_follower/config/robot_params.yaml
# Restart only the controller nodes
ros2 run warehouse_wall_follower pid_controller \
  --ros-args --params-file ~/ros2_ws/src/warehouse_wall_follower/config/robot_params.yaml
```

**Monitor PID in real time:**
```bash
ros2 topic echo /pid_diagnostics
```

Look for:
- `error` close to 0.0 = good tracking
- `i_term` growing large = reduce `ki` or increase `integral_max`
- `d_term` noisy/spiky = reduce `kd`

---

## 8. DEBUGGING INSTRUCTIONS

### 8.1 Check all nodes are running

```bash
ros2 node list
# Expected:
# /wall_detector
# /pid_controller
# /state_machine
# /metrics_node
# /robot_state_publisher
# /ros_gz_bridge
```

### 8.2 Check node parameters

```bash
ros2 param list /pid_controller
ros2 param get /pid_controller kp
ros2 param set /pid_controller kp 1.5  # live tuning!
```

### 8.3 Manually publish velocity commands (override PID)

```bash
# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Spin left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --once
```

### 8.4 Manually trigger state machine transitions

```bash
# Disable PID (puts robot in manual control)
ros2 topic pub /pid_enabled std_msgs/msg/Bool "data: false" --once

# Re-enable PID
ros2 topic pub /pid_enabled std_msgs/msg/Bool "data: true" --once
```

### 8.5 Check TF transforms

```bash
# Check odom → base_footprint transform
ros2 run tf2_ros tf2_echo odom base_footprint

# Check complete TF
ros2 run tf2_tools view_frames
evince frames.pdf
```

### 8.6 Common problems and solutions

**Problem: /scan topic not receiving data**
```bash
# Check if bridge is running
ros2 node list | grep bridge

# Check GZ topic is publishing
gz topic -l | grep scan
gz topic -e -t /scan  # echo gz topic

# Restart bridge
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$(ros2 pkg prefix warehouse_wall_follower)/share/warehouse_wall_follower/config/ros_gz_bridge.yaml
```

**Problem: Robot not moving**
```bash
# Check FSM state
ros2 topic echo /robot_state --once

# Check if PID is enabled
ros2 topic echo /pid_enabled --once

# Check wall_distance is valid
ros2 topic echo /wall_distance --once
# If -1.0: wall not detected → check LiDAR and bridge

# Check cmd_vel is being published
ros2 topic hz /cmd_vel
```

**Problem: Robot oscillating badly**
```bash
# Reduce kp
ros2 param set /pid_controller kp 0.8
# Increase kd
ros2 param set /pid_controller kd 1.0
```

**Problem: TF error in RViz ("No transform from 'base_link' to 'odom'")**
```bash
# Check robot_state_publisher
ros2 topic echo /robot_description --once | head -5
# If empty: robot_description not published → check URDF/xacro build

# Check /tf topic
ros2 topic hz /tf    # should be ~20 Hz

# Check joint_states
ros2 topic hz /joint_states  # should be ~50 Hz
```

**Problem: Gazebo crashes on start**
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"

# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch warehouse_wall_follower warehouse_launch.py
```

### 8.7 Record and replay with rosbag

```bash
# Record all relevant topics
ros2 bag record /scan /wall_distance /wall_info /cmd_vel \
  /robot_state /pid_diagnostics /fsm_diagnostics /metrics /odom /tf

# Replay with simulated clock
ros2 bag play <bag_dir> --clock

# Run nodes with replayed data
ros2 launch warehouse_wall_follower debug_launch.py use_sim_time:=true
```

### 8.8 Visualise PID performance

```bash
# Plot wall_distance over time using rqt_plot
ros2 run rqt_plot rqt_plot \
  /wall_distance/data \
  /pid_diagnostics/data

# Or use plotjuggler (if installed)
ros2 run plotjuggler plotjuggler
```

---

## 9. ARCHITECTURE SUMMARY

```
┌─────────────────────────────────────────────────────────────────┐
│                    GAZEBO HARMONIC                              │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Warehouse World (warehouse.sdf)                        │   │
│  │  - 32m×22m warehouse with corridors, racks, boxes       │   │
│  │  - Physics: 1kHz step, real-time factor 1.0             │   │
│  │                                                          │   │
│  │  ┌─────────────────────────────────────────────────┐    │   │
│  │  │  warehouse_robot (URDF from xacro)              │    │   │
│  │  │  - DiffDrive plugin → /cmd_vel sub, /odom pub   │    │   │
│  │  │  - JointStatePub   → /joint_states pub          │    │   │
│  │  │  - GPU LiDAR       → /scan pub (10 Hz, 360°)    │    │   │
│  │  │  - IMU             → /imu pub (100 Hz)          │    │   │
│  │  └─────────────────────────────────────────────────┘    │   │
│  └──────────────────────────────────────────────────────────┘   │
└──────────────────┬──────────────────────────────────────────────┘
                   │ gz-transport
                   ▼
         ┌─────────────────┐
         │  ros_gz_bridge  │  ← parameter_bridge with ros_gz_bridge.yaml
         │  (8 topic maps) │    bridges: /scan /odom /imu/data /joint_states
         │                 │             /cmd_vel /tf /clock /joint_states
         └────────┬────────┘
                  │ ROS2 DDS/FastDDS
    ┌─────────────┼──────────────────────────────────────────┐
    │             │                                          │
    ▼             ▼                                          ▼
┌──────────┐ ┌──────────────┐                    ┌─────────────────────┐
│ /scan    │ │ /odom        │                    │ robot_state_pub     │
│ (10 Hz)  │ │ (50 Hz)      │                    │ ← /joint_states     │
└────┬─────┘ └──────┬───────┘                    │ → /tf (base→lidar   │
     │              │                            │      base→wheels    │
     │              ▼                            │      base→imu)      │
     │     ┌─────────────────┐                   └─────────────────────┘
     │     │  metrics_node   │
     │     │  - total dist   │ → /metrics (1 Hz)
     │     │  - avg error    │
     │     │  - accuracy %   │
     │     └─────────────────┘
     │
     ├──────────────────────────────────┐
     ▼                                  ▼
┌──────────────────┐          ┌──────────────────────┐
│  wall_detector   │          │  state_machine       │
│  - sector slice  │          │  ← /scan (front obs) │
│  - median filter │          │  ← /wall_distance    │
│  - gap detection │          │  ← /wall_info        │
│                  │          │                      │
│ → /wall_distance │─────────►│  FSM states:         │
│ → /wall_info     │─────────►│  SEARCH_WALL         │
└──────────────────┘          │  FOLLOW_WALL         │
                              │  TURN_CORNER         │
                              │  LOST_WALL           │
                              │                      │
                              │ → /pid_enabled ──────┤
                              │ → /cmd_vel (non-PID) │
                              │ → /robot_state       │
                              │ → /fsm_diagnostics   │
                              └──────────┬───────────┘
                                         │ /pid_enabled
                                         ▼
                              ┌──────────────────────┐
                              │  pid_controller      │
                              │  ← /wall_distance    │
                              │  ← /pid_enabled      │
                              │                      │
                              │  Kp=1.2 Ki=0.05      │
                              │  Kd=0.8 target=0.70m │
                              │  anti-windup ±1.5    │
                              │  20 Hz control loop  │
                              │                      │
                              │ → /cmd_vel ──────────┼───► ros_gz_bridge
                              │ → /pid_diagnostics   │         │
                              └──────────────────────┘         │
                                                               ▼
                                                    Gazebo DiffDrive → robot moves
```

---

*Generated for ROS2 Jazzy + Gazebo Harmonic 8.x on Ubuntu 24.04*
