# 🤖 Vision-Based Pick and Place Robot Arm

A complete **ROS2 Humble + Gazebo Harmonic** robotic system capable of autonomous pick-and-place operations using computer vision.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Python](https://img.shields.io/badge/Python-3.10-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

---

## 🎯 Project Overview

This project implements a 6-DOF robotic arm with a parallel gripper that can:

1.  **See**: Detect colored boxes (Red, Green, Blue) using a camera.
2.  **Think**: Determine which table the box is on and plan a pick sequence.
3.  **Act**: Autonomously pick the box and place it in the matching colored basket.

It features a hybrid control system:

- **Vision System**: Detects object presence and location.
- **Hardcoded Precision**: Uses pre-recorded, validated joint positions for reliable grasping.
- **Contact Intelligence**: Adapts gripper closing based on physical contact.

---
## Video

https://github.com/user-attachments/assets/241f8717-51c9-49bb-8f73-4b1a71cf1a6a

---

## ✨ Key Features

- **👁️ Computer Vision**: Real-time color detection and spatial filtering (ROI) to distinguish boxes from baskets.
- **🧠 Smart Logic**: Automatically maps detected X-coordinates to Table 1, 2, or 3.
- **🦾 Reliable Grasping**: "Minimal Close" strategy to prevent object penetration and physics glitches.
- **🛡️ Safety Validation**: Verifies object position hasn't shifted before attempting a pick.
- **🖥️ Clean Visualization**: Custom annotated camera feed with position labels and status.
- **🔄 Flexible Input**: Supports single, multiple, or specific sequences (e.g., `[red, green]`).

---

## 🛠️ Prerequisites

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Middleware**: ROS2 Humble Hawksbill
- **Simulation**: Gazebo Harmonic (gz-sim)
- **Planning**: MoveIt 2

---

## 🆕 Fresh Installation (From Scratch)

If you are starting with a fresh computer (or VM) and don't have ROS2 installed yet, follow these steps:

### 1. Install ROS2 Humble

```bash
# Set locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### 2. Install Gazebo Harmonic

```bash
sudo apt-get update
sudo apt-get install ros-humble-ros-gz
```

### 3. Install MoveIt 2

```bash
sudo apt install ros-humble-moveit
```

### 4. Install Dependencies

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

---

## 📥 Project Installation

Once you have the prerequisites, set up this project:

### 1. Create Workspace

```bash
mkdir -p ~/robo_ws/src
cd ~/robo_ws/src
```

### 2. Clone Repository

```bash
git clone <repository_url> .
```

### 3. Install Dependencies

```bash
cd ~/robo_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build

```bash
colcon build
source install/setup.bash
```

---

## 🚀 Usage Guide

### 1. Launch the System

This starts Gazebo, MoveIt, RViz, and spawns the robot.

```bash
ros2 launch pick_place_arm unified_gz_moveit.launch.py
```

_Wait about 15 seconds for the controllers to spawn and the robot to initialize._

### 2. Run Vision System

Open a new terminal, source the workspace, and run the main script:

```bash
source install/setup.bash
python3 src/pick_place_arm/scripts/vision_pick_place.py
```

### 3. View Camera Feed

To see what the robot sees (with annotations):

```bash
ros2 run rqt_image_view rqt_image_view
```

_Select `/camera/image_annotated` from the dropdown._

### 4. Control the Robot

The script will prompt you for input. Enter a list of colors to pick:

- **Single Box**: `[red]`
- **Two Boxes**: `[green, blue]`
- **All Boxes**: `[red, green, blue]`

The robot will execute the sequence, picking each box from its detected table and placing it in the matching basket.

---

## 📂 Project Structure

```
robo_ws/src/
├── pick_place_arm/             # Main Package
│   ├── launch/                 # Launch files
│   ├── scripts/                # Python control scripts
│   │   ├── vision_pick_place.py       # ⭐ Main Vision Script
│   │   ├── object_detector.py         # Color detection logic
│   │   ├── pick_and_place_contact.py  # Contact-based demo
│   │   └── ...
│   ├── models/                 # Gazebo assets (baskets, tables)
│   └── worlds/                 # Simulation world
├── arm_moveit_config/          # MoveIt Configuration
│   ├── config/                 # SRDF, Controllers, Kinematics
│   └── ...
└── ...
```

---

## 🔧 Troubleshooting

- **"Controllers not active"**: Wait 15-20 seconds after launch. The spawner script has a built-in delay.
- **"Box not detected"**: Ensure the box is in the camera's view (bottom 60% of frame).
- **"Motion Plan Failed"**: The robot might be in a singularity or collision. Restart the simulation.

---

## 👨‍💻 Author

**Raj**
_Vision-Based Robotics Enthusiast_

---

**Branch**: `Vision_based_pick-plce`
**Status**: ✅ Production Ready
