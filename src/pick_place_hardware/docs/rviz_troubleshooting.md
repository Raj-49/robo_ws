# RViz Robot Disappearing - Troubleshooting Guide

## Issue Description

The green robot visualization disappears in RViz after launching the virtual hardware system.

## Root Cause Analysis

The launch file **already has** `robot_state_publisher` configured correctly. The issue is likely:

### 1. **Timing Issue** (Most Likely)

The controllers are spawning too quickly before the robot_state_publisher is fully ready.

### 2. **use_sim_time Mismatch**

The controller_manager was missing the `use_sim_time` parameter, which could cause TF timing issues.

## Fix Applied

Added `use_sim_time` parameter to controller_manager:

```python
controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        moveit_config.robot_description,
        ros2_control_config,
        {"use_sim_time": LaunchConfiguration('use_sim_time')}  # ← ADDED
    ],
    output="screen",
)
```

## How to Test the Fix

### Step 1: Rebuild (if needed)

```bash
cd ~/pick_place_hardware_implementation/Dexter-vision-based-pick-place-robotic-arm
source install/setup.bash
```

### Step 2: Launch with Verbose Logging

```bash
ros2 launch pick_place_hardware test_fake_hardware.launch.py --ros-args --log-level debug
```

### Step 3: Check TF Tree

In another terminal:

```bash
# Check if TF transforms are being published
ros2 run tf2_tools view_frames

# Check robot_state_publisher is running
ros2 node list | grep robot_state_publisher

# Check /joint_states topic
ros2 topic hz /joint_states
```

### Step 4: Verify in RViz

- Open RViz
- Check "Global Options" → "Fixed Frame" is set to "world" or "base_link"
- Check "RobotModel" display is enabled
- Look for TF errors in the status bar

## Additional Diagnostics

### Check for Multiple Instances

```bash
# Should show only ONE robot_state_publisher
ros2 node list | grep robot_state_publisher

# Should show only ONE controller_manager
ros2 node list | grep controller_manager
```

### Check Joint States

```bash
# Should publish at ~100 Hz
ros2 topic hz /joint_states

# Should show all 8 joints
ros2 topic echo /joint_states --once
```

### Check TF Transforms

```bash
# Should show complete robot chain
ros2 run tf2_ros tf2_echo world base_link
```

## Common Issues & Solutions

### Issue 1: Robot Flickers or Disappears

**Cause**: TF timing mismatch  
**Solution**: Ensure all nodes use same `use_sim_time` setting

### Issue 2: "No transform from X to Y"

**Cause**: robot_state_publisher not receiving joint_states  
**Solution**: Check `joint_state_broadcaster` is active

### Issue 3: Robot Stuck in One Pose

**Cause**: joint_state_broadcaster not publishing  
**Solution**:

```bash
ros2 control list_controllers
# joint_state_broadcaster should be "active"
```

### Issue 4: Multiple Controller Managers

**Cause**: Launch file spawning controllers multiple times  
**Solution**: Our launch file uses standard `spawner` - no duplicates

## Verification Checklist

After launching, verify:

- [ ] `robot_state_publisher` node is running
- [ ] `/joint_states` topic publishing at ~100 Hz
- [ ] `joint_state_broadcaster` controller is active
- [ ] TF tree is complete (world → base_link → ... → l6)
- [ ] RViz shows green robot
- [ ] No TF warnings in RViz status bar

## If Problem Persists

1. **Kill all ROS nodes**:

   ```bash
   killall -9 ros2 rviz2 gz
   ```

2. **Clear ROS logs**:

   ```bash
   rm -rf ~/.ros/log/*
   ```

3. **Relaunch with clean state**:

   ```bash
   ros2 launch pick_place_hardware test_fake_hardware.launch.py
   ```

4. **Check for errors**:
   Look for ERROR or WARN messages related to:
   - robot_state_publisher
   - joint_state_broadcaster
   - TF transforms

## Expected Behavior

When working correctly, you should see:

```
[robot_state_publisher]: Publishing transforms for 9 links
[joint_state_broadcaster]: Configured and activated
[INFO] Joint states publishing at 100 Hz
```

And in RViz:

- Green robot visible
- Smooth motion when executing trajectories
- No TF warnings
