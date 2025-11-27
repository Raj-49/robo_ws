# ROS2 Pick and Place Robot Arm

A complete ROS2 Humble + Gazebo Harmonic pick-and-place system with vision capabilities and hardcoded position control.

## 🎯 Project Overview

This project implements a 6-DOF robotic arm with a parallel gripper for automated pick-and-place operations. The system supports both vision-based object detection and hardcoded position control for reliable operation.

## ✨ Features

- **Hardcoded Pick-and-Place**: Reliable operation using pre-recorded joint positions
- **Contact-Based Grasping**: Intelligent gripper control that adapts to object contact
- **Vision System**: Camera-based object detection and localization (foundation implemented)
- **RViz Visualization**: Real-time visualization with interactive markers
- **MoveIt Integration**: Motion planning and collision avoidance
- **Gazebo Simulation**: Physics-based simulation environment

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Harmonic
- MoveIt2

### Installation

```bash
cd ~/robo_ws
colcon build
source install/setup.bash
```

### Running the Demo

**Launch the simulation:**

```bash
ros2 launch arm_moveit_config unified_gz_moveit.launch.py
```

**Run pick-and-place demo (contact-based):**

```bash
python3 src/pick_place_arm/scripts/pick_and_place_contact.py
```

**Alternative (time-based):**

```bash
python3 src/pick_place_arm/scripts/pick_and_place_demo.py
```

## 📹 Demo Video

See `Pick_&_place.webp` for a demonstration of the system in action.

## 🎮 Menu Options

1. **Pick and Place RED box** - Table 2 → Red Basket → Home
2. **Pick and Place GREEN box** - Table 3 → Green Basket → Home
3. **Pick and Place BLUE box** - Table 1 → Blue Basket → Home
4. **Pick and Place ALL** - Complete RGB sequence

## 📊 Current Status

### ✅ Completed

- [x] Robot arm URDF and MoveIt configuration
- [x] Gazebo simulation with DetachableJoint system
- [x] Hardcoded pick positions from SRDF
- [x] Contact-based gripper control
- [x] 3 working pick-and-place positions (RGB to matching baskets)
- [x] RViz visualization with basket meshes
- [x] Manual control and visualization tools
- [x] Git repository with proper branching

### 🔄 In Progress

- [ ] Record remaining 6 positions (cross-basket placements)
- [ ] Full vision-based pick-and-place integration
- [ ] Complete documentation

## 🗂️ Project Structure

```
robo_ws/
├── src/
│   ├── arm_moveit_config/          # MoveIt configuration
│   │   ├── config/
│   │   │   ├── moveit.rviz         # RViz config
│   │   │   └── pick_place_arm.srdf # Semantic robot description
│   │   └── launch/
│   │       └── unified_gz_moveit.launch.py
│   └── pick_place_arm/             # Main package
│       ├── models/                  # Gazebo models (boxes, baskets, tables)
│       ├── scripts/
│       │   ├── pick_and_place_contact.py    # Contact-based demo
│       │   ├── pick_and_place_demo.py       # Time-based demo
│       │   ├── manual_control_and_viz.py    # Manual control tool
│       │   └── vision_interactive_demo.py   # Vision testing
│       ├── urdf/
│       │   └── arm.urdf.xacro      # Robot description
│       └── worlds/
│           └── my_world.sdf        # Gazebo world
```

## 🔧 Key Scripts

- **pick_and_place_contact.py**: Contact-based grasping with automatic gripper stop
- **pick_and_place_demo.py**: Simple time-based pick-and-place
- **manual_control_and_viz.py**: Manual attachment control and position recording
- **vision_interactive_demo.py**: Vision system testing and development

## 📝 Recorded Positions

### Pick Positions (from SRDF)

```python
blue_pick:  [0.959, 0.394, 0.479, 0.234, 0.451, 0.00005]
red_pick:   [0.0001, 0.445, 0.451, 0.365, 0.411, -0.0000009]
green_pick: [-0.948, 0.268, 0.565, 0.679, 0.171, -0.00009]
```

### Place Positions (recorded)

```python
blue_to_blue:   [-1.908633, -0.285813, 0.672824, 0.187192, 0.450882, 0.000170]
red_to_red:     [-2.436366, 0.065933, 0.303389, 0.197981, 0.411065, -0.000045]
green_to_green: [-1.362681, -0.156609, 0.418936, 0.427446, 0.171263, -0.000047]
```

### Home Position

```python
home: [-0.000090, -0.000069, -0.000047, -0.000100, -0.000024, 0.000027]
```

## 🛠️ Technical Details

### DetachableJoint System

- Uses Gazebo Harmonic's DetachableJoint plugin
- Attach/detach via ROS topics (`/[color]_box/attach`, `/[color]_box/detach`)
- Parent link: `l6` (end effector)

### Gripper Control

- Parallel gripper with 2 fingers (`j7l`, `j7r`)
- Contact-based closing: stops at `-0.010` position
- Extraction movement before detach to prevent sticking

### Vision System (Foundation)

- Camera mounted on end effector
- Object detection pipeline implemented
- Ready for full integration

## 🤝 Contributing

This is a personal project. For questions or collaboration, please open an issue.

## 📄 License

MIT License - See LICENSE file for details

## 🙏 Acknowledgments

- ROS2 Community
- MoveIt2 Team
- Gazebo Development Team

## 📧 Contact

Raj - GitHub: @Raj-49

---

**Branch:** `hardcoded-pick-place-v1`  
**Status:** ✅ Working Demo  
**Next Milestone:** Vision-based pick-and-place integration
