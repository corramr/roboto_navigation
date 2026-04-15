# Robot Navigation with Nav2 and Real Sensor Integration

This repository provides a Docker-based development environment for robot navigation using ROS 2 Navigation Stack (Nav2). It includes configurations for both simulated TurtleBot3 examples and real sensor integration with LiDAR.

## Table of Contents
- [Branching Strategy](#branching-strategy)
- [Prerequisites](#prerequisites)
- [Launching](#launching)

---

## Branching Strategy

| Branch | Purpose |
|--------|---------|
| `develop` | Feature development and solid implementations |
| `master` | Competition-ready, stable codebase only |

---

## Prerequisites

- Docker installed on your system
- X11 server for GUI applications
- *(Optional)* Livox LiDAR sensor for real sensor integration

### Enable Docker GUI Access

Before running any GUI applications, allow Docker to access your display:

```bash
xhost +local:docker
```

---

## Launching

### 1. Build the Docker Image

From the repository root directory:


```bash
# AMD architecture
sudo docker build -f Dockerfile_AMD -t roboto/navigation:latest .

# ARM architecture
sudo docker build -f Dockerfile_ARM -t roboto/navigation:latest .
```

### 2. Run the Docker Container

```bash
sudo docker run -it --privileged --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="./app/nav2_ws/src:/root/nav2_ws/src" \
  --volume="./app/initialize:/root/initialize" \
  --device /dev/dri:/dev/dri \
  roboto/navigation
```

> **Note:** Some options may need adjustment depending on your system configuration.

### 3. Build Source Code Inside the Container

```bash
# ARM architecture
source ../initialize/build_ARM.bash

# AMD architecture
source ../initialize/build_AMD.bash
```

### 4. Launch

Inside the Docker container:


**Launch Nav2 in simulation** *(AMD only — Gazebo is not supported on ARM)*:
```bash
ros2 launch sentry_navigation launch.py use_simulator:=True
```

**Launch Nav2 with LiDAR**:
```bash
ros2 launch sentry_navigation launch.py
```

> The commands above will start both the Nav2 stack and the sensor stack.

**Launch the sensor stack only**:
```bash
ros2 launch sensor_launcher launch.py
```