# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Docker-based ROS2 (Humble) development environment for autonomous robot navigation. Integrates the Nav2 stack with real Livox LiDAR sensors. Supports both TurtleBot3 Gazebo simulation and real hardware.

## Branching Strategy

- **`develop`** — active development
- **`master`** — competition-ready, stable only

> Develop on host machine, run/test inside the Docker container.

## Environment Setup

Allow Docker GUI access (required before any container run):
```bash
xhost +local:docker
```

Build the Docker image:
```bash
docker build -t corra09/nav2_docker:dev .
```

Start the container (mounts source code for live editing from host):
```bash
docker run -it --privileged --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="./app/nav2_ws/src:/root/nav2_ws/src" \
  --volume="./app/initialize:/root/initialize" \
  --device /dev/dri:/dev/dri \
  corra09/nav2_docker:dev
```

## Launch Commands (inside container)

**TurtleBot3 Simulation:**
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

**Real Sensor Integration (two terminals):**
```bash
# Terminal 1 — starts Livox driver, converter, PointCloud→LaserScan, FAST-LIO odometry
ros2 launch sensor_launcher launch.py

# Terminal 2 — Nav2 stack consuming real sensor data
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

**Debug TF tree:**
```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

## Architecture

### Sensor Pipeline (real hardware mode)

```
Livox LiDAR
  └─ livox_ros_driver2 → /livox/lidar (CustomMsg)
       └─ livox_converter → /livox/lidar_pc2 (PointCloud2)
            └─ pointcloud_to_laserscan → /scan (LaserScan)

Livox PointCloud + IMU
  └─ FAST_LIO_ROS2 → /odom (nav_msgs/Odometry)
```

### Nav2 Stack

`/scan` + `/odom` → **AMCL** → `/amcl_pose` + TF transforms → **Planner** → **Controller** → velocity commands

All Nav2 nodes are lifecycle-managed. The lifecycle manager (in `nav2_bringup`) starts and transitions all nodes.

### Custom Packages

| Package | Location | Purpose |
|---|---|---|
| `livox_converter` | `app/nav2_ws/src/livox_converter/` | Python ROS2 node: Livox CustomMsg → PointCloud2 |
| `sensor_launcher` | `app/nav2_ws/src/sensor_launcher/` | Orchestrates all sensor drivers in one launch |

### Key Configuration Files

- **Nav2 parameters:** `app/nav2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml` — AMCL tuning, DWB planner, costmaps, recovery behaviors
- **Main launch:** `app/nav2_ws/src/navigation2/nav2_bringup/launch/tb3_simulation_launch.py`
- **Navigation launch:** `app/nav2_ws/src/navigation2/nav2_bringup/launch/navigation_launch.py`
- **Sensor launch:** `app/nav2_ws/src/sensor_launcher/launch/launch.py`
- **Container init:** `app/initialize/init.bash` — sources ROS2 Humble, sets `TURTLEBOT3_MODEL=waffle`, configures Gazebo model paths

## Switching from Simulation to Real Sensors

Three files in `nav2_bringup` must be modified:

1. **`launch/tb3_simulation_launch.py`** — set `use_sim_time` to `False`; comment out Gazebo launch
2. **`worlds/waffle.model`** — disable the simulated odometry, LiDAR, and IMU plugins (prevents them from publishing on `/odom` and `/scan`)

After these changes, `sensor_launcher` provides the real `/scan`, `/odom`, `/tf`, and `/tf_static` topics.

## TF Frame Relationships

Expected chain: `map` → `odom` → `base_footprint` → `base_link` → `base_scan`

FAST-LIO publishes the `odom` → `base_footprint` transform. AMCL publishes `map` → `odom`. The TurtleBot3 URDF (still loaded by default) defines the remaining static transforms.
