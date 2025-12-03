# 🤖 DEXTER: Pick and Place Robotic Arm - Base Version

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress%206.17.0-orange)](https://gazebosim.org/docs/fortress)
![MoveIt2](https://img.shields.io/badge/MoveIt2-Planning-brightgreen)
![ros2_control](https://img.shields.io/badge/ros2_control-Hardware_Interface-lightgrey)
[![License](https://img.shields.io/badge/License-Apache--2.0-yellow)](LICENSE)

**Foundation for Vision-Based Robotic Manipulation**

---

## 📋 Table of Contents

1. [Project Overview](#-project-overview)
2. [Features](#-features)
3. [System Architecture](#-system-architecture)
4. [Project Structure](#-project-structure)
5. [Installation & Setup](#-installation--setup)
6. [Usage & Quick Start](#-usage--quick-start)
7. [System Components](#-system-components)
8. [Troubleshooting](#-troubleshooting)
9. [References](#-references)

---

## 🎯 Project Overview

### Description

This is the **base version** of DEXTER, a 6-DOF robotic arm with parallel gripper designed for pick-and-place operations. This version provides the fundamental infrastructure for robotic manipulation, including physics simulation, motion planning capabilities, and controller integration. It serves as the foundation upon which vision-based object detection and autonomous manipulation features are built in the main version.

### What is This Version?

This **base version** includes:
- ✅ Complete robot model (URDF/SRDF) with 6-DOF arm + parallel gripper
- ✅ Gazebo Fortress physics simulation environment
- ✅ MoveIt2 motion planning integration
- ✅ ROS2 control framework with joint controllers
- ✅ Contact sensing and object attachment mechanics
- ✅ Basic world environment with tables and objects
- ✅ RViz visualization for motion planning

This version does **NOT** include:
- ❌ Vision-based object detection (OpenCV integration)
- ❌ Autonomous pick-and-place sequencing
- ❌ Camera-based spatial reasoning
- ❌ Colored baskets and vision-guided placement
- ❌ Automated task execution scripts

For the full vision-enabled system, see the [main version](main-version-readme.md).

### Key Technologies

- **ROS2 Humble Hawksbill** - Robot Operating System 2
- **Gazebo Fortress 6.17.0** - Physics simulation (ign-gazebo)
- **MoveIt2** - Motion planning framework
- **ros2_control** - Hardware interface and controller management
- **Python 3.10** - Scripting and automation

---

## ✨ Features

- ✅ **6-DOF Robotic Arm** - Full kinematic chain with 6 revolute joints
- ✅ **Parallel Gripper** - 2-finger gripper with position control
- ✅ **Physics Simulation** - Realistic dynamics modeling in Gazebo Fortress
- ✅ **Motion Planning** - MoveIt2 integration with multiple planners (OMPL, CHOMP, Pilz)
- ✅ **ROS2 Control** - Joint trajectory controllers and state broadcasting
- ✅ **RViz Visualization** - Interactive motion planning interface

---

## 🏗️ System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Planning Layer                           │
│      MoveIt2 Motion Planning → Trajectory Generation        │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                    Execution Layer                          │
│    ROS2 Control → Joint Controllers → Gazebo Simulation     │
└─────────────────────────────────────────────────────────────┘
```

### Software Stack

| Component         | Technology | Version                      |
| ----------------- | ---------- | ---------------------------- |
| Operating System  | Ubuntu     | 22.04 LTS                    |
| Middleware        | ROS2       | Humble Hawksbill             |
| Simulation        | Gazebo     | Fortress 6.17.0 (ign-gazebo) |
| Motion Planning   | MoveIt2    | 2.5.5                        |
| Build System      | colcon     | 0.12.1                       |

---

## 📂 Project Structure

```
Dexter-vision-based-pick-place-robotic-arm/
├── src/
│   ├── pick_place_arm/                    # Main Package
│   │   ├── CMakeLists.txt                 # Build configuration
│   │   ├── package.xml                    # Package metadata
│   │   ├── config/
│   │   │   └── ros2_control.yaml          # Controller parameters
│   │   ├── launch/
│   │   │   └── unified_gz_moveit.launch.py  # ⭐ Main launch file
│   │   ├── scripts/
│   │   │   ├── wait_and_spawn_controllers.py  # ⭐ Controller initialization
│   │   │   └── controller_starter.py      # Alternative controller starter
│   │   ├── urdf/
│   │   │   └── arm.urdf.xacro             # ⭐ Robot description
│   │   ├── meshes/                        # 3D models (STL/OBJ)
│   │   └── worlds/
│   │       └── my_world.sdf               # Simulation environment
│   │
│   └── arm_moveit_config/                 # MoveIt Configuration Package
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── config/
│           ├── pick_place_arm.srdf        # ⭐ Semantic robot description
│           ├── pick_place_arm.urdf.xacro  # Robot for MoveIt
│           ├── pick_place_arm.ros2_control.xacro
│           ├── joint_limits.yaml          # Joint constraints
│           ├── kinematics.yaml            # IK solver configuration
│           ├── ompl_planning.yaml         # OMPL planner settings
│           ├── chomp_planning.yaml        # CHOMP planner settings
│           ├── pilz_cartesian_limits.yaml
│           ├── pilz_industrial_motion_planner_planning.yaml
│           ├── moveit_controllers.yaml    # MoveIt controller mapping
│           ├── ros2_controllers.yaml      # Controller configurations
│           └── moveit.rviz                # RViz configuration
├── README.md                              # This file
└── main-version-readme.md                 # Full version documentation
```

### Key Files Explained

- **`unified_gz_moveit.launch.py`**: Main launch file that starts Gazebo, MoveIt2, controllers, and RViz (now in pick_place_arm/launch)
- **`arm.urdf.xacro`**: Robot description with package:// URIs for portable mesh references (works with both Gazebo and RViz)
- **`wait_and_spawn_controllers.py`**: Manages controller lifecycle (load, configure, activate)
- **`pick_place_arm.srdf`**: Defines planning groups and named poses
- **`my_world.sdf`**: Clean Gazebo world with ground plane and lighting

---

## 🛠️ Installation & Setup

### Prerequisites

- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS2**: Humble Hawksbill
- **Gazebo**: Fortress 6.17.0 (ign-gazebo)
- **MoveIt2**: Latest for ROS2 Humble

### Installation Steps

#### 1. Install ROS2 Humble

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-base
sudo apt install ros-dev-tools

# Source ROS2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

#### 2. Install Gazebo Fortress & ROS2 Integration

```bash
# Install Gazebo Fortress and ROS-Gazebo bridge packages
sudo apt-get update
sudo apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ignition-fortress

# Install gz_ros2_control plugin (CRITICAL for joint control)
sudo apt install ros-humble-gz-ros2-control
```

#### 3. Install MoveIt2 & Motion Planning

```bash
# Install MoveIt2 core and plugins
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-simple-controller-manager
```

#### 4. Install ROS2 Controllers & Additional Packages

```bash
# Install controller packages
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro

# Install launch system
sudo apt install -y \
    ros-humble-launch-ros \
    ros-humble-launch
```

#### 5. Install Build Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep

# Initialize rosdep (skip if already done)
sudo rosdep init
rosdep update
```

### Project Installation

#### 6. Clone Repository

```bash
# Navigate to your workspace location
cd ~

# Clone repository
git clone https://github.com/Raj-49/Dexter-vision-based-pick-place-robotic-arm.git
cd Dexter-vision-based-pick-place-robotic-arm
```

#### 7. Install Project Dependencies

```bash
# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y
```

#### 8. Build the Workspace

```bash
# Source ROS2 BEFORE building
source /opt/ros/humble/setup.bash

# Build
colcon build

# Source the workspace overlay
source install/setup.bash
```

> **⚠️ Common Build Error**: If you get `CMake Error: Could not find a package configuration file provided by "ament_cmake"`, you forgot to source `/opt/ros/humble/setup.bash` before running `colcon build`.

#### 9. Setup Auto-Sourcing (Recommended)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Dexter-vision-based-pick-place-robotic-arm/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verification

```bash
# Check ROS2 distro
echo $ROS_DISTRO
# Should output: humble

# Check if workspace packages are visible
ros2 pkg list | grep pick_place_arm
# Should output: pick_place_arm and arm_moveit_config
```

---

## 🚀 Usage & Quick Start

### Launching the System

#### Start the Complete System

Open a terminal and launch the simulation with MoveIt2:

```bash
cd ~/Dexter-vision-based-pick-place-robotic-arm
source install/setup.bash
ros2 launch pick_place_arm unified_gz_moveit.launch.py
```

**What this does:**
- Starts Gazebo Fortress with a clean simulation world
- Spawns the 6-DOF robot arm with gripper
- Launches MoveIt2 move_group for motion planning
- Opens RViz for visualization
- Spawns and activates controllers after 12-second delay

**Wait ~15-20 seconds** for all controllers to initialize before using the robot.

#### Launch Options

```bash
# Launch without RViz
ros2 launch pick_place_arm unified_gz_moveit.launch.py start_rviz:=false

# Launch in headless mode (for SSH/no display)
ros2 launch pick_place_arm unified_gz_moveit.launch.py headless:=true

# Specify custom spawn position
ros2 launch pick_place_arm unified_gz_moveit.launch.py x:=1.0 y:=2.0 z:=0.5

# Use wall clock instead of sim time
ros2 launch pick_place_arm unified_gz_moveit.launch.py use_sim_time:=false
```

### Controlling the Robot

#### Using RViz Motion Planning

1. In RViz, find the **MotionPlanning** panel
2. Under **Planning Group**, select `arm` or `gripper`
3. Drag the interactive markers to set a goal pose
4. Click **Plan** to generate a trajectory
5. Click **Execute** to move the robot

#### Using Named Poses

Named poses are pre-configured positions defined in `pick_place_arm.srdf`. You can:

1. In RViz's **MotionPlanning** panel, go to **Select Goal State**
2. Choose a named pose (e.g., "home", "ready", etc.)
3. Click **Plan & Execute**

### Monitoring System Status

```bash
# Check active controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# View available topics
ros2 topic list

# View system transforms
ros2 run tf2_tools view_frames
```

---

## 🔧 System Components

### Controllers

The system uses the following ros2_control controllers:

1. **joint_state_broadcaster**: Publishes joint states to `/joint_states`
2. **arm_controller**: Joint trajectory controller for the 6-DOF arm (j1-j6)
3. **gripper_controller**: Position controller for parallel gripper (j7l, j7r)

### Planning Groups

Defined in `pick_place_arm.srdf`:

- **arm**: Joints j1 through j6 (base to wrist)
- **gripper**: Joints j7l and j7r (left and right fingers)

### Topics

Key ROS2 topics:

- `/joint_states` - Current joint positions/velocities
- `/arm_controller/follow_joint_trajectory` - Arm motion commands
- `/gripper_controller/follow_joint_trajectory` - Gripper commands
- `/clock` - Simulation time

### Services

- `/controller_manager/*` - Controller lifecycle management
- `/compute_ik` - Inverse kinematics solving
- `/plan_kinematic_path` - Motion planning

---

## 🔍 Troubleshooting

### Controllers Not Starting

**Symptom**: Error messages about controllers not found

**Solution**:
```bash
# Wait for controller manager
ros2 service list | grep controller_manager

# Manually spawn controllers
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner gripper_controller
```

### RViz Crashes or Display Issues

**Symptom**: RViz window closes unexpectedly

**Solution**:
```bash
# Use headless mode if no display available
ros2 launch pick_place_arm unified_gz_moveit.launch.py headless:=true

# Or disable RViz entirely
ros2 launch pick_place_arm unified_gz_moveit.launch.py start_rviz:=false
```

### Build Errors

**Symptom**: CMake or compilation errors

**Solutions**:
```bash
# Clean and rebuild
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Joint Limits Exceeded

**Symptom**: "Joint limits exceeded" warnings in MoveIt2

**Solution**: Check `joint_limits.yaml` and ensure goal poses are within valid ranges.

---

## 📚 References

### Official Documentation

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/index.html)
- [Gazebo Fortress Documentation](https://gazebosim.org/docs/fortress)
- [ros2_control Documentation](https://control.ros.org/humble/index.html)

### Related Projects

- [Main Version (Vision-Based)](main-version-readme.md) - Full system with computer vision
- [MoveIt2 Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
- [ros_gz Examples](https://github.com/gazebosim/ros_gz)

### Community Resources

- [ROS2 Discourse](https://discourse.ros.org/)
- [Gazebo Community](https://community.gazebosim.org/)
- [MoveIt Discourse](https://github.com/ros-planning/moveit2/discussions)

---

## 📝 License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

---

## 🤝 Contributing

This is the base version of the DEXTER project. For contributing to the full vision-enabled system, please refer to the main repository.

---

## 🎓 Project Context

This base version serves as the foundation for the complete DEXTER vision-based pick-and-place system. It provides:

- Proven robot kinematics and dynamics
- Stable controller integration
- Reliable motion planning infrastructure
- Contact sensing framework

The main version extends this base with:
- OpenCV-based object detection
- HSV color space filtering
- Spatial reasoning and table mapping
- Autonomous task sequencing
- Vision-guided manipulation

For the complete system with all features, see [main-version-readme.md](main-version-readme.md).

---

**Developed with ROS2 Humble, MoveIt2, and Gazebo Fortress**

*Base version - Foundation for vision-based robotic manipulation*
