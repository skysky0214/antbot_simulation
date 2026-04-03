---
title: 5.3 Nav2 Integration
description: AntBot Nav2 Navigation Stack Integration Guide
sidebar:
  order: 3
---

AntBot uses Nav2, the official ROS 2 navigation framework, for autonomous driving.
Custom tuning is applied for the 4-wheel independent swerve-drive characteristics.

---

## Overview

### Nav2 Pipeline

![Nav2 Pipeline](../../../../assets/images/nav2_pipeline.png)

### AntBot vs Standard diff-drive

| Aspect | Standard diff-drive | AntBot swerve |
|--------|---------------------|---------------|
| Controller | DWB | **MPPI** (rollout-based optimization) |
| Motion Model (AMCL) | DifferentialMotionModel | **OmniMotionModel** |
| Velocity DOF | vx, wz (2DOF) | **vx, vy, wz (3DOF)** |
| Costmap Inflation | ~0.3m | **0.75m** (steering overshoot margin) |
| LiDAR | Single | **Dual 2D** (front + back) |
| Odometry TF | Direct publish | **EKF sensor fusion** (collision protection) |

---

## Quick Start

### Prerequisites

```bash
# Install Nav2 navigation stack, SLAM, EKF sensor fusion, and MPPI controller
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller
```

:::note
For Gazebo simulation dependencies, see [5.4 Simulation Environment Setup — Prerequisites](/antbot/en/development-guide/simulation/#prerequisites).
:::

### Run (3 Terminals)

**Terminal 1 — Gazebo Simulation**

```bash
# Start the Gazebo simulator with the robot model
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
```

**Terminal 2 — Nav2 Navigation**

```bash
# Start EKF + AMCL + full Nav2 navigation stack
ros2 launch antbot_navigation navigation.launch.py mode:=sim world:=depot
```

**Terminal 3 — RViz Visualization**

```bash
# Open RViz with Nav2-specific configuration
rviz2 -d $(ros2 pkg prefix antbot_navigation --share)/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

:::note
Wait ~8-15 seconds for the Gazebo window and controller spawners to complete.
The `world` argument accepts a world name registered in `worlds.yaml` or a full path to an SDF file.
:::

---

## Navigation Modes

### Launch File Comparison

| Launch file | Purpose | Features |
|-------------|---------|----------|
| `navigation.launch.py` | Autonomous driving with saved map | AMCL + full Nav2 stack |
| `slam.launch.py` | Build map while navigating | SLAM Toolbox (no pre-built map needed) |
| `localization.launch.py` | Localization only | No path planning, EKF + AMCL only |

```bash
# Save the SLAM-generated map to a file
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Sim / Real Mode

All launch files accept a `mode` argument that auto-selects the config directory and `use_sim_time`:

```bash
# Run in simulation environment (uses Gazebo clock)
ros2 launch antbot_navigation slam.launch.py mode:=sim
# Run on the real robot (uses system clock)
ros2 launch antbot_navigation slam.launch.py mode:=real
```

| Setting | `mode:=sim` | `mode:=real` |
|---------|-------------|--------------|
| Config directory | `config/sim/` | `config/real/` |
| `use_sim_time` | `true` | `false` |
| MPPI `vx_max` | 1.0 m/s | 2.0 m/s |
| Velocity smoother | [1.5, 0.15, 1.5] | [1.0, 0.10, 1.0] |
| EKF IMU topic | `/imu/data` | `/imu_node/imu/accel_gyro` |
| EKF process noise | Low (ideal sensors) | Higher (real noise) |
| MPPI `batch_size` | 2000 | 1500 (Jetson Orin) |

### Setting Navigation Goals

**Single Goal**

1. In RViz, click **2D Pose Estimate** to set the robot's initial position
2. Click **Nav2 Goal** to specify the target position

**Multi-Waypoint Following**

RViz → Panels → Add New Panel → `nav2_rviz_plugins/Navigation2` → check Waypoint Mode → click multiple goals → Start Waypoint Following

**Goal via CLI**

```bash
# Send a navigation goal to coordinates (x: 5.0, y: 3.0) in the map frame
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"
```

:::tip
Using `world:=depot` automatically resolves the corresponding map file (`depot_sim.yaml`) from `worlds.yaml`. To specify a map path directly, use the `map:=` argument instead:
```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim \
  map:=/path/to/my_map.yaml
```
:::

---

## System Architecture

### TF Tree

![TF Tree](../../../../assets/images/tf_tree.png)

| Transform | Publisher | Input Data | Output |
|-----------|----------|------------|--------|
| `map → odom` | AMCL or SLAM Toolbox | LiDAR scans (`/scan_0`) + map data | Global position correction |
| `odom → base_link` | EKF (Nav2 mode) or swerve controller (standalone) | Wheel odometry (`/odom`) + IMU | Robot displacement estimation |
| `base_link → *` | robot_state_publisher | URDF model | Sensor/wheel frame positions |

### odom TF Handover

:::caution
If both swerve controller and EKF publish `odom→base_link` TF simultaneously, jitter occurs.
Navigation launch auto-disables the controller's TF publish,
and Nav2 nodes start with an 8-second delay to allow EKF to establish the TF first.
If you see jitter, disable manually:

```bash
# Disable swerve controller's odom TF publishing
ros2 param set /antbot_swerve_controller enable_odom_tf false
```
:::

---

## Parameter Tuning

Config files: `antbot_navigation/config/{sim,real}/`

| Component | Role | Config File |
|-----------|------|-------------|
| **MPPI Controller** | Generates candidate paths and selects the optimal one to move the robot | `nav2_params.yaml` |
| **AMCL** | Compares LiDAR scans against the map to estimate the robot's current position | `nav2_params.yaml` |
| **EKF** | Combines wheel odometry and IMU data for improved position accuracy | `ekf.yaml` |
| **SLAM Toolbox** | Builds a map in real-time while simultaneously estimating position | `slam_toolbox_params.yaml` |
| **Costmap** | Builds an obstacle grid from sensor data for collision avoidance | `nav2_params.yaml` |

### MPPI Controller (Path Following)

MPPI (Model Predictive Path Integral) simulates thousands of candidate paths and selects the one with the highest score.

AntBot's steering can only turn **+/-60 degrees**, making sideways movement physically impossible.
To change direction, the robot uses a **rotate-in-place then drive-forward** pattern, and the parameters below are tuned accordingly.

**Key Velocity Parameters**

| Parameter | Sim | Real | Description |
|-----------|-----|------|-------------|
| `vx_max` | 1.0 | 2.0 | Max forward speed (m/s). Real robot is set faster |
| `vy_max` | 0.1 | 0.5 | Max lateral speed (m/s). **Lower values enforce forward-driving behavior** |
| `wz_max` | 1.5 | 2.0 | Max rotation speed (rad/s) |
| `batch_size` | 2000 | 1500 | Number of candidate paths per cycle. More = precise but heavier compute |
| `time_steps` | 56 | 56 | Number of lookahead steps. Multiplied by `model_dt` (0.05s) to determine prediction horizon |

:::note[time_steps and vy_max rationale]
- **Prediction horizon**: `time_steps × model_dt = 56 × 0.05s = 2.8s`. Increasing extends the horizon but raises compute cost.
- **Why vy_max is low**: A low value prevents MPPI from generating lateral paths. Attempting lateral motion causes steering to hit the +/-60 degree limit, producing unnatural movement.
:::

**MPPI Critics (Path Scoring Criteria)** — each candidate path is scored by multiple critics. Higher weight means more influence on path selection.

Swerve direction control — key settings that make AntBot drive forward rather than sideways:

| Critic | Sim | Real | Description |
|--------|-----|------|-------------|
| `PreferForwardCritic` | 15.0 | 5.0 | Encourages forward-facing movement. Higher = suppresses reverse/lateral |
| `PathAngleCritic` | 15.0 | 5.0 | Aligns robot heading with path direction. Higher = tighter alignment |
| `TwirlingCritic` | 10.0 | 5.0 | Suppresses unnecessary spinning. Higher = more stable straight-line driving |

Path following and safety:

| Critic | Weight | Description |
|--------|--------|-------------|
| `GoalCritic` | 5.0 | Rewards paths that get closer to the goal |
| `PathAlignCritic` | 10.0 | Evaluates alignment between candidate and global path |
| `PathFollowCritic` | 5.0 | Prevents deviation from the global path |
| `ObstaclesCritic` | — | Penalizes paths near obstacles. `collision_cost: 10000` blocks collision paths |
| `ConstraintCritic` | 4.0 | Penalizes paths that violate max velocity/acceleration limits |

:::tip[Sim vs Real Weight Differences]
Simulation has no friction, so the robot slides easily — higher direction control weights are needed.
The real robot's tire-ground friction naturally encourages straight driving, so lower weights suffice.
:::

### AMCL (Position Estimation)

AMCL estimates the robot's current position by comparing LiDAR scan data against the saved map.

```yaml
amcl:
  robot_model_type: "nav2_amcl::OmniMotionModel"
  scan_topic: /scan_0
```

:::caution[OmniMotionModel Required]
AntBot is a swerve robot that can move forward, backward, and sideways, so it requires `OmniMotionModel` which recognizes multi-directional movement.
Standard 2-wheel robots (diff-drive) only move forward and backward, so `DifferentialMotionModel` is sufficient for them.
However, using that model on a robot like AntBot that can also move laterally (vy) will fail to recognize sideways movement, significantly degrading localization accuracy.
:::

### EKF Sensor Fusion (Odometry Correction)

EKF (Extended Kalman Filter) combines wheel odometry and IMU sensor data to calculate a more accurate position than either sensor alone.

| Setting | Sim | Real | Description |
|---------|-----|------|-------------|
| Odometry topic | `/odom` | `/odom` | Velocity from wheels (vx, vy, vyaw) |
| IMU topic | `/imu/data` | `/imu_node/imu/accel_gyro` | Gyro/accelerometer (yaw, vyaw) |
| Process noise | Low | High | Real sensors have more noise, so uncertainty is set higher |
| `odom0_rejection_threshold` | 2.0 | 1.5 | Odometry data exceeding this value is ignored |

When the robot hits a wall, wheel slip can produce abnormally large velocity values in the odometry. `odom0_rejection_threshold` automatically ignores these spikes, preventing the position estimate from drifting significantly after a collision.

:::caution[Sim vs Real IMU Topic]
In simulation, Gazebo ros_gz_bridge publishes IMU on `/imu/data`, while the real robot uses `/imu_node/imu/accel_gyro`. The correct topic is configured in `config/sim/ekf.yaml` and `config/real/ekf.yaml` respectively.
:::

### SLAM Toolbox (Real-time Map Building)

SLAM Toolbox uses LiDAR data to build a map in real-time while simultaneously estimating position.
Used by `slam.launch.py` and does not require a pre-built map.

| Parameter | Sim | Real | Description |
|-----------|-----|------|-------------|
| `max_laser_range` | 20.0 | 12.0 | Max usable LiDAR range (m). Reduced for real LiDAR performance |
| `minimum_travel_distance` | 0.5 | 0.3 | Min distance traveled before adding a new scan to the map (m) |
| `minimum_travel_heading` | 0.5 | 0.4 | Min rotation before adding a new scan (rad) |
| `resolution` | 0.05 | 0.05 | Map grid resolution (m/pixel). 5cm per cell |
| `map_update_interval` | 5.0 | 5.0 | Map update period (seconds) |

:::tip[Sim vs Real Differences]
The real robot uses a reduced `max_laser_range` due to noisier LiDAR data,
and tighter scan insertion (`minimum_travel_distance: 0.3`) to compensate with better map coverage.
Loop closure thresholds are also stricter on the real robot to prevent false loop closures.
:::

### Costmap (Obstacle Map)

The costmap builds a grid-based obstacle map from sensor data. The robot uses this map to plan paths that avoid obstacles.

AntBot uses dual 2D LiDARs — front (`/scan_0`) and rear (`/scan_1`) — for 360-degree obstacle detection.

- **Robot footprint**: 0.70m x 0.60m
- **Inflation radius** (`inflation_radius`): 0.75m — adds a safety margin around obstacles to account for steering overshoot

| Aspect | Local Costmap | Global Costmap |
|--------|---------------|----------------|
| Reference frame | `odom` | `map` |
| Size | 5m x 5m (around robot only) | Full map |
| Update rate | 5Hz (fast, for nearby obstacles) | 1Hz (slow, for overall path planning) |
| Layers | obstacle x2 + inflation | static + obstacle x2 + inflation |

---

## World and Map Management

### Directory Structure

World (SDF) and map (PGM/YAML) files are co-located in `antbot_navigation/maps/`:

```
antbot_navigation/maps/
├── worlds.yaml          # World name → SDF/map path mapping
├── depot.sdf            # Gazebo world file
├── depot_sim.pgm        # 2D occupancy grid map
└── depot_sim.yaml       # Map metadata (resolution, origin, etc.)
```

### worlds.yaml Configuration

`worlds.yaml` maps world names to their SDF and Nav2 map files:

```yaml
worlds:
  depot:
    sdf: depot.sdf           # Gazebo world SDF
    map: depot_sim.yaml       # Nav2 map YAML
```

When launching with `world:=depot`:
- **Gazebo**: loads `maps/depot.sdf`
- **Navigation**: passes `maps/depot_sim.yaml` to the map server

### Registering a Custom World

1. **Create a map with SLAM** (or import externally):

   ```bash
   # Start SLAM mode to build a map in real-time
   ros2 launch antbot_navigation slam.launch.py mode:=sim
   # Save the generated map to a file
   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_world
   ```

2. **Place files**: copy `my_world.pgm`, `my_world.yaml`, and `my_world.sdf` into `antbot_navigation/maps/`

3. **Register in worlds.yaml**:

   ```yaml
   worlds:
     depot:
       sdf: depot.sdf
       map: depot_sim.yaml
     my_world:                    # New world
       sdf: my_world.sdf
       map: my_world.yaml
   ```

4. **Launch**:

   ```bash
   # Run simulation with the registered custom world
   ros2 launch antbot_gazebo gazebo.launch.py world:=my_world
   ros2 launch antbot_navigation navigation.launch.py mode:=sim world:=my_world
   ```

:::note
New worlds can be added by editing `worlds.yaml` alone — no launch file changes needed.
The `map` argument can still be used to override with any arbitrary map file path.
:::

---

## Troubleshooting

### "Failed to create plan"

The robot is positioned inside an obstacle on the costmap.

Use **2D Pose Estimate** in RViz to correct the initial position, or reset the costmap:

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

### Wall collision / avoidance failure

Insufficient safety margin around obstacles.

- Increase `inflation_radius` (0.75 → 1.0)
- Decrease `cost_scaling_factor` (1.5 → 1.0)
- Increase `ObstaclesCritic.collision_margin_distance` (0.1 → 0.2)

### Excessive spinning in place

The robot keeps rotating instead of moving toward the goal.

- Increase `TwirlingCritic` weight (10.0 → 15.0)
- Decrease `wz_max` (1.5 → 1.0)
- Decrease `PreferForwardCritic` weight (15.0 → 10.0)

### Missing map frame / TF timeout

Gazebo restart resets sim time to 0, invalidating all TF buffers.

**Always restart Gazebo and Navigation together.**

### Diagnostic Commands

```bash
# Check LiDAR data reception rate (expected: ~10Hz)
ros2 topic hz /scan_0
# Check EKF sensor fusion output (expected: ~50Hz)
ros2 topic hz /odometry/filtered
# Verify map → odom TF transform status
ros2 run tf2_ros tf2_echo map odom
# Check swerve controller's odom TF publish status
ros2 param get /antbot_swerve_controller enable_odom_tf
# Check Nav2 controller server lifecycle state
ros2 lifecycle get /controller_server
# List ros2_control controllers and their status
ros2 control list_controllers
```

:::tip
For `/cmd_vel` and `/odom` topic specifications, see [5.2 Key ROS Topics/Services](/antbot/en/development-guide/ros-topics/).
For simulation environment details, see [5.4 Simulation Environment Setup](/antbot/en/development-guide/simulation/).
:::
