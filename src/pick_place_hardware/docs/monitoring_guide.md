# Hardware Interface Monitoring Guide

## Overview

The `monitor_hardware_interface.py` script provides real-time visibility into the data flow between MoveIt/RViz and the FakeRobotHardware plugin.

## What It Monitors

### READ Operations (Hardware → Controllers)

- Joint states published by the hardware interface
- Position and velocity feedback
- Publication rate (should be ~100 Hz)

### WRITE Operations (Controllers → Hardware)

- Desired joint positions from controllers
- Actual joint positions
- Tracking errors
- Command flow from MoveIt through controllers to hardware

## Usage

### Step 1: Launch Virtual Hardware

```bash
# Terminal 1
ros2 launch pick_place_hardware test_fake_hardware.launch.py
```

### Step 2: Start the Monitor

```bash
# Terminal 2
source install/setup.bash
python3 src/pick_place_hardware/scripts/monitor_hardware_interface.py
```

### Step 3: Send Commands

Use RViz to plan and execute trajectories, and watch the monitor display all interactions in real-time.

## Output Explanation

### System Status (Every 1 second)

```
═══════════════════════════════════════════════════════════════════════════════
SYSTEM STATUS @ 12:34:56
  Uptime: 45.2s
  READ operations:   4520 total |  100.0 Hz avg |  100.2 Hz current
  WRITE operations:    452 total

  Current Joint States (from hardware):
    j1: pos=+0.1234 rad, vel=+0.0012 rad/s
    j2: pos=-0.5678 rad, vel=-0.0034 rad/s
    ...

  Desired Joint States (from controller):
    j1: desired=+0.1250, actual=+0.1234, error=+0.0016
    j2: desired=-0.5700, actual=-0.5678, error=-0.0022
    ...
═══════════════════════════════════════════════════════════════════════════════
```

### Detailed Logs (Every 100 operations)

```
────────────────────────────────────────────────────────────────────────────────
[READ #100] Hardware Interface → Controllers
  Timestamp: 12:34:56.123
  Positions: j1=+0.123 j2=-0.567 j3=+0.890 j4=-0.234 j5=+0.456 j6=-0.789
  Velocities: j1=+0.001 j2=-0.003 j3=+0.002 j4=-0.001 j5=+0.000 j6=-0.002
────────────────────────────────────────────────────────────────────────────────
[WRITE #100] ARM Controller → Hardware Interface
  Timestamp: 12:34:56.124
  Desired: j1=+0.125 j2=-0.570 j3=+0.895 j4=-0.230 j5=+0.460 j6=-0.785
  Actual:  j1=+0.123 j2=-0.567 j3=+0.890 j4=-0.234 j5=+0.456 j6=-0.789
  Error:   j1=+0.002 j2=-0.003 j3=+0.005 j4=+0.004 j5=+0.004 j6=+0.004
```

## What This Reveals

### 1. Control Loop Timing

- **READ rate**: Should be ~100 Hz (matches ros2_control update rate)
- **WRITE rate**: Varies based on trajectory execution

### 2. Tracking Performance

- **Error values**: Show how well the simulated hardware tracks commands
- **Small errors** (< 0.01 rad): Good tracking
- **Large errors**: May indicate aggressive trajectories or low position_gain

### 3. Data Flow

```
MoveIt Plans Trajectory
        ↓
JointTrajectoryController interpolates waypoints @ 100Hz
        ↓
[WRITE] Controller sends position commands
        ↓
FakeRobotHardware simulates delay + lag + noise
        ↓
[READ] Hardware returns simulated joint states
        ↓
Controller calculates error and adjusts
```

### 4. Realistic Behavior

- **Delay**: Commands take ~50ms to affect position
- **Lag**: Position approaches target gradually (first-order dynamics)
- **Noise**: Small random variations in position readings

## Troubleshooting

### No Output

- Check that virtual hardware is running
- Verify topics exist: `ros2 topic list | grep joint`

### Low READ Rate

- Expected: ~100 Hz
- If lower: Check CPU usage or ros2_control configuration

### Large Tracking Errors

- Increase `position_gain` in fake_hardware.ros2_control.xacro
- Reduce trajectory speed in MoveIt

## Advanced: Comparing to Real Hardware

When you implement RealRobotHardware, this same monitor will show:

- **Real communication delays** (USB, CAN latency)
- **Actual motor response** (PID tuning quality)
- **Real encoder noise** (sensor quality)
- **True tracking performance** (mechanical limitations)

The patterns you see with FakeRobotHardware should closely match real hardware behavior!
