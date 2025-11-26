# Pick and Place Robot Arm

A ROS 2 Humble + Gazebo Ignition pick-and-place demonstration using a 6-DOF robot arm with a 2-finger gripper and dynamic attachment system.

## Features

- **6-DOF Robot Arm** with MoveIt2 motion planning
- **Smart Gripper** with stall detection and dynamic attachment
- **DetachableJoint Plugin** for physics-based object grasping
- **Gazebo Ignition** simulation environment
- **CHOMP, OMPL, and Pilz** motion planners

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Ignition (Fortress/Garden)
- MoveIt2

## Installation

1. **Clone the repository:**
   ```bash
   cd ~/robo_ws/src
   git clone <your-repo-url>
   ```

2. **Install dependencies:**
   ```bash
   cd ~/robo_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

## Usage

### Running the Pick and Place Demo

The demo requires **2 terminals only**:

#### Terminal 1: Launch Gazebo + MoveIt
```bash
source install/setup.bash
ros2 launch arm_moveit_config unified_gz_moveit.launch.py
```

Wait ~15 seconds for all systems to initialize.

#### Terminal 2: Run Integrated Demo
```bash
source install/setup.bash
python3 install/pick_place_arm/lib/pick_place_arm/demo_pick_place.py
```

**That's it!** The demo script now includes:
- ✅ Initial detach sequence (clears pre-attachment)
- ✅ Stall detection and attachment logic
- ✅ Automatic detachment on gripper open

### What Happens

1. **Gripper closes** on the box at home position
2. **Stall detection** triggers → Box **attaches** to gripper
3. **Arm moves** to place_1 position with attached box
4. **Waits 5 seconds**
5. **Gripper opens** → Box **detaches** and is released

## Configuration

### Gripper Attachment Parameters

The gripper attachment node can be configured via `config/gripper_params.yaml`:

```yaml
gripper_attachment_node:
  ros__parameters:
    stall_threshold_min: -0.020  # Lower bound for stall detection
    stall_threshold_max: -0.003  # Upper bound for stall detection
    open_threshold: -0.002       # Position above which gripper is open
    detach_attempts: 10          # Detach commands sent on startup
    gripper_joint: 'j7l'         # Joint to monitor
```

### Tuning Tips

**If gripper doesn't attach:**
- Increase `stall_threshold_max` (e.g., `-0.002`)
- Check gripper actually stalls: `ros2 topic echo /gripper_state`

**If gripper attaches too early:**
- Decrease `stall_threshold_max` (e.g., `-0.005`)

**Monitor state transitions:**
```bash
ros2 topic echo /gripper_state
```

You should see: `OPEN` → `GRASPING` → `ATTACHED` → `OPEN`

### Using Custom Parameters

```bash
ros2 run pick_place_arm gripper_attachment_node.py \
  --ros-args --params-file install/pick_place_arm/share/pick_place_arm/config/gripper_params.yaml
```

## How It Works

### Smart Grasping System

The `gripper_attachment_node.py` implements intelligent grasping:

1. **Stall Detection**: Monitors gripper joint position (`j7l`)
   - If gripper stops mid-close (e.g., `-0.005` instead of commanded `-0.032`), it detected the box!
   
2. **Stop & Hold**: Sends a trajectory command to hold the current position
   - Prevents physics instability from over-squeezing
   
3. **Dynamic Attachment**: Publishes to `/box1/attach` topic
   - `DetachableJoint` plugin creates a fixed joint between gripper and box
   
4. **Auto-Detach**: When gripper opens, publishes to `/box1/detach`
   - Joint is removed, box is free

### Key Components

- **`pick_place_arm`**: Robot URDF, worlds, and scripts
- **`arm_moveit_config`**: MoveIt configuration and launch files
- **`gripper_attachment_node.py`**: Smart grasping logic
- **`demo_pick_place.py`**: Simple pick-and-place demo

## Configuration

### Gripper Joint Limits
- `j7l`: `-0.07` to `0.0` (left finger, moves negative to close)
- `j7r`: `0.0` to `0.07` (right finger, moves positive to close)

### Box Specifications
- Size: `0.06 x 0.06 x 0.06` m
- Mass: `0.2` kg
- Position: `x=0.1412, y=-0.0045, z=0.03`

### Place Position (place_1)
```python
[0.0, 1.57, 0.0, -1.57, 0.0, 0.0]  # j1-j6
```

## Troubleshooting

### Box is pre-attached on startup
- The `gripper_attachment_node` sends 10 detach commands over 5 seconds at startup
- Make sure to start it **before** running the demo

### Gripper doesn't attach
- Check that `gripper_attachment_node.py` is running
- Look for "Grasp Detected (Stall at X) -> STOPPING & ATTACHING" message
- Verify bridges are active: `ros2 topic list | grep box1`

### Physics instability / "spring" effect
- The "Stop & Hold" logic should prevent this
- If it persists, check gripper close position in `demo_pick_place.py`

## License

[Your License Here]

## Acknowledgments

- DetachableJoint implementation inspired by MBZIRC suction gripper plugin
- MoveIt2 and Gazebo Ignition communities
