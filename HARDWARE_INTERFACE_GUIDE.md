# 🤖 From Simulation to Reality: ROS2 Hardware Interfacing Guide

**Author:** Raj
**Date:** November 28, 2025
**Project:** Pick and Place Robot Arm

---

## 📚 Table of Contents

1. [Introduction & Philosophy](#1-introduction--philosophy)
2. [System Architecture](#2-system-architecture)
3. [Hardware Requirements](#3-hardware-requirements)
4. [Phase 1: The Firmware (Microcontroller)](#4-phase-1-the-firmware-microcontroller)
5. [Phase 2: The ROS2 Hardware Interface](#5-phase-2-the-ros2-hardware-interface)
6. [Phase 3: Integration & Configuration](#6-phase-3-integration--configuration)
7. [Phase 4: Testing & Safety](#7-phase-4-testing--safety)
8. [References & Resources](#8-references--resources)

---

## 1. Introduction & Philosophy

This document serves as a deep-dive manual for transitioning a robot arm from a Gazebo simulation to a physical, real-world machine.

In our simulation, we used **Gazebo Plugins**. These are pieces of software that pretend to be motors. They take a command like "Move Joint 1 to 1.5 radians" and magically move the joint in the simulator.

In the real world, we must replace that "magic" with a concrete software driver that talks to your specific electronics. This is the **Hardware Interface**.

The philosophy of ROS2 Control is **Abstraction**. The "Controllers" (like `joint_trajectory_controller`) do not care if they are talking to a simulation or a real robot. They just send commands to a standard interface. Your job is to build the bridge between that standard interface and your Arduino/Raspberry Pi.

---

## 2. System Architecture

The goal is to replace the **Gazebo Plugin** with a **Real Hardware Driver**.

### 2.1 The Data Flow

```mermaid
graph LR
    MoveIt[MoveIt 2] -->|Trajectory| Controller[Joint Trajectory Controller]
    Controller -->|Commands| HW_Interface[ROS2 Hardware Interface (C++)]
    HW_Interface -->|Serial/USB| Micro[Microcontroller (Arduino/ESP32)]
    Micro -->|PWM/Step| Driver[Motor Drivers]
    Driver -->|Power| Motor[Motors]
    Sensor[Encoders/Pots] -->|Feedback| Micro
    Micro -->|State Data| HW_Interface
    HW_Interface -->|Joint States| Controller
```

### 2.2 The "Brain" (High-Level Control)

- **Device:** PC or Raspberry Pi 4/5.
- **OS:** Ubuntu 22.04 with ROS2 Humble.
- **Role:** Runs MoveIt, calculates Inverse Kinematics, plans paths, and runs the `ros2_control` loop.

### 2.3 The "Muscle Controller" (Low-Level Firmware)

- **Device:** Microcontroller (Arduino Mega, ESP32, Teensy).
- **Role:**
  1. Receives simple target positions (e.g., "Motor 1 -> 1000 steps").
  2. Generates high-speed electrical pulses (PWM or Step/Dir).
  3. Reads sensors (Encoders/Potentiometers) and reports back.

---

## 3. Hardware Requirements

### A. Microcontroller

- **Recommended**: **ESP32** or **Teensy 4.x** (32-bit, fast).
- **Arduino Mega**: Possible but slow for ROS2 communication. Good for simple Serial.
- **Raspberry Pi**: Great for running the "High Level" ROS2 code, connected to the Arduino via USB.

### B. Feedback Sensors (CRITICAL) ⚠️

**MoveIt requires feedback.** It needs to know where the arm _is_, not just where you told it to go.

- **Steppers without Encoders**: You must have **Limit Switches** (Homing) to establish a 0.0 position at startup.
- **Servos/DC Motors**: You need Potentiometers or Encoders.

---

## 4. Phase 1: The Firmware (Microcontroller)

Your microcontroller needs to be a "dumb" slave. It shouldn't calculate inverse kinematics.

### 4.1 Communication Protocol

We need a language for the PC and Arduino to talk. A simple text-based protocol is best for beginners.

**PC to Arduino (Command):**
Format: `C,<Joint1>,<Joint2>,<Joint3>,<Joint4>,<Joint5>,<Joint6>,E`
Example: `C,90,45,0,0,0,0,E`

**Arduino to PC (Feedback):**
Format: `F,<Joint1>,<Joint2>,<Joint3>,<Joint4>,<Joint5>,<Joint6>,E`
Example: `F,89.5,45.1,0,0,0,0,E`

### 4.2 Implementing the Loop (Arduino C++)

```cpp
void loop() {
  // 1. Check for new data from PC
  if (Serial.available() > 0) {
    readCommand(); // Parse "C,90,45..."
    updateMotorTargets();
  }

  // 2. Control Motors (PID or Stepper Logic)
  runMotors();

  // 3. Send Feedback (every ~10-20ms)
  if (millis() - last_print > 20) {
    sendFeedback(); // Print "F,89.5,45..."
    last_print = millis();
  }
}
```

**Critical Requirement:** The Arduino must _never_ block (pause). Do not use `delay()`.

---

## 5. Phase 2: The ROS2 Hardware Interface

This is the C++ bridge on your PC. You must write a class that inherits from `hardware_interface::SystemInterface`.

### 5.1 Key Methods to Implement

#### 1. `on_init()`

- **Action:** Open Serial Port (`/dev/ttyACM0`). Configure baud rate (115200).
- **Check:** Verify Arduino connection.

#### 2. `read()` (Runs at ~50Hz-100Hz)

- **Action:** Read serial buffer. Parse "F,..." string.
- **Update:** Update `hw_states_` vector with new positions (convert Steps -> Radians).
- **Why:** MoveIt needs to know exactly where the robot is.

#### 3. `write()` (Runs at ~50Hz-100Hz)

- **Action:** Take values from `hw_commands_`.
- **Format:** Convert Radians -> Steps. Create "C,..." string.
- **Send:** Write string to Serial Port.

### 5.2 Official Demo

Check the **ros2_control_demos** "RRBot" example:
[RRBot System Interface Example](https://github.com/ros-controls/ros2_control_demos/blob/humble/example_2/hardware/rrbot_system_position_only.cpp)

---

## 6. Phase 3: Integration & Configuration

### 1. Update `ros2_control.xacro`

Switch from Gazebo plugin to your custom plugin.

**New (Real Hardware):**

```xml
<hardware>
    <plugin>my_robot_package/MyRobotHardware</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
</hardware>
```

### 2. Create Real Robot Launch File

Create `real_hardware.launch.py` that:

1. Does **NOT** launch Gazebo.
2. Launches `ros2_control_node`.
3. Launches `robot_state_publisher`.
4. Spawns controllers (`joint_trajectory_controller`, `joint_state_broadcaster`).
5. Launches MoveIt and RViz.

---

## 7. Phase 4: Testing & Safety

### 1. The "Ghost" Test

Run the hardware interface but **disconnect motor power**.

- Move arm in RViz.
- Watch Arduino Serial Monitor.
- Verify numbers match RViz movement.

### 2. Safety Limits

In `joint_limits.yaml`:

- Reduce `max_velocity` and `max_acceleration` to **10%** for the first run.

### 3. Common Pitfalls

- **Jerky Motion**: PC sending too fast or Arduino processing too slow. Increase baud rate.
- **Runaway Robot**: Unit mismatch (Radians vs Steps). Double check math.
- **Interface Not Found**: Check `pluginlib` export in `package.xml`.

---

## 8. References & Resources

1. **ROS2 Control Documentation**: [https://control.ros.org/master/index.html](https://control.ros.org/master/index.html)
2. **MoveIt 2 Tutorials**: [https://moveit.picknik.ai/](https://moveit.picknik.ai/)
3. **Articulated Robotics (YouTube)**: [Making a Mobile Robot with ROS2](https://www.youtube.com/watch?v=4vv382vXw7E)
4. **Serial Library**: [libserial](https://github.com/crayzeewulf/libserial)

---

## 📝 Summary Checklist

1. [ ] **Firmware**: Write Arduino code to read/write Serial strings.
2. [ ] **Communication**: Verify motor movement via Serial Monitor.
3. [ ] **C++ Plugin**: Create `ros2_control` hardware interface with `libserial`.
4. [ ] **URDF Update**: Switch to custom hardware plugin.
5. [ ] **Launch**: Create real robot launch file.
