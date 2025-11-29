# Complete Codebase Documentation

## Overview

This workspace contains a vision-based pick and place robotic system with 3 ROS2 packages.

---

## Package 1: `pick_place_arm`

**Purpose**: Main robot package containing URDF, models, worlds, and control scripts.

### Root Files

- **`CMakeLists.txt`**: Build configuration, installs scripts, meshes, models, worlds
- **`package.xml`**: Package metadata and dependencies

### `/config` - Controller Configuration

- **`gripper_params.yaml`**: Gripper controller parameters (joint names, limits)
- **`ros2_control.yaml`**: ROS2 control configuration for arm and gripper

### `/launch` - Launch Files

- **`gazebo.launch.py`**: Standalone Gazebo launch (deprecated, use unified)
- **`gz_with_robot.launch.py`**: Gazebo with robot spawn
- **`test_vision.launch.py`**: Vision testing launch file

### `/scripts` - Python Scripts (9 files)

**Production Scripts:**

1. **`vision_pick_place.py`** ⭐ Main System

   - Vision-based pick and place
   - Camera detection → table mapping → sequence execution
   - User input: `[red]`, `[green, blue]`, etc.

2. **`pick_and_place_contact.py`**

   - Contact-based pick and place with hardcoded positions
   - Uses gripper contact sensors for attachment
   - Demonstrates all 6 recorded positions

3. **`object_detector.py`**
   - Color detection module (HSV-based)
   - Detects Red, Green, Blue, Yellow objects
   - Area filtering: 100-5000 pixels (excludes baskets)

**Critical Startup Scripts:** 4. **`wait_and_spawn_controllers.py`** ⚙️ Critical

- Waits for Gazebo/bridge to be ready
- Spawns arm_controller and gripper_controller
- Runs 12 seconds after launch

5. **`initial_detach.py`** ⚙️ Critical
   - Detaches all boxes at startup
   - Clears any pre-existing attachments
   - Runs 15 seconds after launch

**Utility Scripts:** 6. **`pose_to_tf_relay.py`**

- Relays Gazebo model poses to TF
- Filters and publishes transforms for boxes, tables, baskets
- Enables RViz visualization

7. **`rviz_interactive_control.py`**

   - Creates interactive markers in RViz
   - Manual attach/detach control via markers
   - Visualization of simulation objects

8. **`ik_utils.py`**

   - Inverse kinematics utilities
   - MoveIt integration helpers

9. **`vision_utils.py`**
   - Vision processing utilities
   - Camera calibration helpers

### `/urdf` - Robot Description

- **`robo_arm/`**: Contains URDF files for the robot arm
  - Robot geometry, joints, links
  - Gazebo plugins, sensors
  - ROS2 control integration

### `/meshes` - 3D Models

- **`basket.obj`**: Basket mesh for baskets
- **`*.stl`**: Robot arm link meshes
- Visual and collision geometries

### `/models` - Gazebo Models

- **`basket_red/`**: Red basket model (SDF + meshes)
- **`basket_green/`**: Green basket model
- **`basket_blue/`**: Blue basket model
- **`plate_stand/`**: Table/stand models

### `/worlds` - Gazebo Worlds

- **`my_world.sdf`**: Main simulation world
  - 3 tables (grey, for boxes)
  - 3 colored baskets (red, green, blue)
  - 3 colored boxes (red, green, blue)
  - Camera (overhead, rotated)
  - Lighting, physics settings

---

## Package 2: `arm_moveit_config`

**Purpose**: MoveIt2 configuration for motion planning and control.

### Root Files

- **`CMakeLists.txt`**: Minimal build config
- **`package.xml`**: MoveIt dependencies
- **`.setup_assistant`**: MoveIt Setup Assistant metadata

### `/config` - MoveIt Configuration (16 files)

**Core Configuration:**

- **`pick_place_arm.srdf`** ⭐ Semantic Robot Description

  - Planning groups: `arm`, `gripper`
  - Named poses: Home, Table-1/2/3 Pick, Red/Green/Blue Basket Place
  - Collision matrix, end effectors

- **`pick_place_arm.urdf.xacro`**: URDF entry point
- **`pick_place_arm.ros2_control.xacro`**: ROS2 control hardware interface

**Motion Planning:**

- **`ompl_planning.yaml`**: OMPL planner configuration (RRT, PRM, etc.)
- **`chomp_planning.yaml`**: CHOMP planner (gradient-based)
- **`pilz_industrial_motion_planner_planning.yaml`**: Pilz (LIN, PTP, CIRC)
- **`trajopt_planning.yaml`**: TrajOpt planner
- **`lerp_planning.yaml`**: Linear interpolation planner

**Constraints:**

- **`joint_limits.yaml`**: Joint velocity, acceleration, jerk limits
- **`pilz_cartesian_limits.yaml`**: Cartesian velocity limits
- **`kinematics.yaml`**: IK solver configuration (KDL)

**Controllers:**

- **`moveit_controllers.yaml`**: MoveIt controller manager config
- **`ros2_controllers.yaml`**: Joint trajectory controllers for arm/gripper

**Other:**

- **`initial_positions.yaml`**: Default joint positions
- **`sensors_3d.yaml`**: 3D sensor configuration (camera)
- **`moveit.rviz`**: RViz configuration with MoveIt displays

### `/launch`

- **`unified_gz_moveit.launch.py`** ⭐ Main Launch File
  - Starts Gazebo Harmonic
  - Spawns robot
  - Launches MoveIt move_group
  - Starts RViz
  - Runs controller spawner (12s delay)
  - Runs initial detach (15s delay)
  - Starts TF relay and interactive control

---

## Package 3: `robo_vision`

**Purpose**: Vision processing package (placeholder/future expansion).

### Files

- **`setup.py`**: Python package setup
- **`package.xml`**: Package metadata

---

## Root Workspace Files

### `/home/raj/robo_ws/`

- **`README.md`**: Project overview and usage instructions
- **`TODO.md`**: Future tasks and improvements
- **`HARDWARE_INTERFACE_GUIDE.md`**: Comprehensive guide for real hardware integration

---

## Data Flow

### Startup Sequence

1. **Launch** → `unified_gz_moveit.launch.py`
2. **Gazebo** starts with `my_world.sdf`
3. **Robot** spawned from URDF
4. **Bridges** connect Gazebo ↔ ROS2 (clock, contacts, camera, poses)
5. **Controllers** spawned at 12s (`wait_and_spawn_controllers.py`)
6. **Initial detach** at 15s (`initial_detach.py`)
7. **MoveIt** ready for planning
8. **RViz** shows visualization

### Vision-Based Pick & Place Flow

1. **User** runs `vision_pick_place.py`
2. **Camera** `/camera/image_raw` → `object_detector.py`
3. **Detection** → Color masks → Centroids → Table mapping (X-axis)
4. **ROI Filter** → Ignore top 40% (baskets)
5. **Validation** → Check 5cm tolerance from initial position
6. **User Input** → `[red, green, blue]`
7. **For each color**:
   - Detect table number
   - Move to `table{N}_pick` position
   - Close gripper + attach
   - Move to `{color}_basket` position
   - Detach + open gripper
   - Return home

### Visualization

- **`/camera/image_annotated`**: Processed camera feed
  - Bounding boxes (colored)
  - Position labels: `color: (x, y)`
  - Table labels: `T1`, `T2`, `T3`
  - ROI boundary line
  - Initial position markers (yellow dots)

---

## Key Technologies

**ROS2**: Humble
**Gazebo**: Harmonic (gz-sim)
**MoveIt2**: Motion planning framework
**OpenCV**: Vision processing (cv_bridge, HSV detection)
**ros2_control**: Controller framework
**gz_ros2_control**: Gazebo-ROS2 control bridge

---

## Recorded Positions (in SRDF)

**Pick Positions:**

- Table-1: `[0.959, 0.394, 0.479, 0.234, 0.451, 0.00005]`
- Table-2: `[0.0001, 0.445, 0.451, 0.365, 0.411, -0.0000009]`
- Table-3: `[-0.948, 0.268, 0.565, 0.679, 0.171, -0.00009]`

**Place Positions:**

- Blue Basket: `[-1.909, -0.286, 0.673, 0.187, 0.451, 0.00017]`
- Red Basket: `[-2.436, 0.066, 0.303, 0.198, 0.411, -0.00005]`
- Green Basket: `[-1.363, -0.157, 0.419, 0.427, 0.171, -0.00005]`

**Home**: All zeros

---

## File Count Summary

**Total Files**: ~60+

- **Scripts**: 9 Python files
- **Config**: 18 YAML/XACRO files
- **Launch**: 4 launch files
- **Models**: 4 Gazebo model directories
- **Meshes**: 9+ mesh files
- **Worlds**: 1 SDF world
- **Documentation**: 4 markdown files

---

## Status: Production Ready ✅

- Vision detection working
- Position validation active
- Clean codebase (24 test files removed)
- Ready for Phase 3 testing
