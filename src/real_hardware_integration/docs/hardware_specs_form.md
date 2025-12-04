# Hardware Specification Form

Please fill out this form with details about your actual hardware. This information will be used to implement the RealRobotHardware plugin.

---

## 1. Motor/Actuator Information

### Motor Type

- [ ] Dynamixel Servos (specify model: ******\_******)
- [ ] Stepper Motors with drivers (specify: ******\_******)
- [ ] DC Motors with encoders (specify: ******\_******)
- [ ] CAN-based actuators (specify: ******\_******)
- [ ] Other: ******\_******

### Motor IDs/Addresses

Please list the motor ID or address for each joint:

| Joint               | Motor ID/Address | Notes |
| ------------------- | ---------------- | ----- |
| j1 (base rotation)  |                  |       |
| j2 (shoulder)       |                  |       |
| j3 (elbow)          |                  |       |
| j4 (wrist 1)        |                  |       |
| j5 (wrist 2)        |                  |       |
| j6 (wrist 3)        |                  |       |
| j7l (gripper left)  |                  |       |
| j7r (gripper right) |                  |       |

---

## 2. Communication Interface

### Connection Type

- [ ] USB (specify port: ******\_******)
- [ ] Serial/UART (specify: ******\_******)
- [ ] CAN bus (specify interface: ******\_******)
- [ ] Ethernet (specify IP: ******\_******)
- [ ] GPIO pins (specify board: ******\_******)
- [ ] Other: ******\_******

### Communication Protocol

- [ ] Dynamixel Protocol 1.0
- [ ] Dynamixel Protocol 2.0
- [ ] Modbus RTU/TCP
- [ ] CANopen
- [ ] Custom protocol (describe: ******\_******)
- [ ] Other: ******\_******

### Baud Rate / Speed

- Baud rate: ******\_****** (e.g., 1000000 for Dynamixel)
- CAN bitrate: ******\_****** (if applicable)

---

## 3. Control Modes

### Available Control Modes (check all that apply)

- [ ] Position control
- [ ] Velocity control
- [ ] Torque/Current control
- [ ] PWM control
- [ ] Other: ******\_******

### Preferred Control Mode for this project

- Primary mode: ******\_******
- Fallback mode: ******\_******

---

## 4. Feedback/Sensing

### Position Feedback

- [ ] Built-in encoders (resolution: ******\_****** counts/rev)
- [ ] External encoders (specify: ******\_******)
- [ ] Potentiometers
- [ ] Hall sensors
- [ ] No position feedback
- [ ] Other: ******\_******

### Velocity Feedback

- [ ] Calculated from position
- [ ] Direct velocity sensor
- [ ] Not available

### Current/Torque Feedback

- [ ] Available (specify range: ******\_******)
- [ ] Not available

---

## 5. Physical Limits

### Joint Limits (in radians or meters)

| Joint | Min Position | Max Position | Max Velocity | Max Torque/Current |
| ----- | ------------ | ------------ | ------------ | ------------------ |
| j1    |              |              |              |                    |
| j2    |              |              |              |                    |
| j3    |              |              |              |                    |
| j4    |              |              |              |                    |
| j5    |              |              |              |                    |
| j6    |              |              |              |                    |
| j7l   |              |              |              |                    |
| j7r   |              |              |              |                    |

---

## 6. Existing Software/Drivers

### Do you have existing driver code?

- [ ] Yes (language: ******\_******)
- [ ] No

### If yes, please specify:

- Driver library/SDK: ******\_******
- Version: ******\_******
- GitHub repo (if public): ******\_******

### Operating System

- [ ] Ubuntu 22.04 (recommended for ROS2 Humble)
- [ ] Other Linux: ******\_******
- [ ] Windows (will need WSL2)
- [ ] Other: ******\_******

---

## 7. Safety Requirements

### Emergency Stop

- [ ] Hardware E-stop button available
- [ ] Software E-stop only
- [ ] Not implemented yet

### Safety Features Needed

- [ ] Joint limit enforcement
- [ ] Velocity limiting
- [ ] Torque/current limiting
- [ ] Collision detection
- [ ] Watchdog timer
- [ ] Other: ******\_******

---

## 8. Real-Time Requirements

### Control Loop Frequency

- Desired frequency: ******\_****** Hz
- Minimum acceptable: ******\_****** Hz

### Latency Tolerance

- Maximum acceptable latency: ******\_****** ms

### Hard Real-Time Required?

- [ ] Yes (need RT_PREEMPT kernel)
- [ ] No (soft real-time is acceptable)

---

## 9. Power Supply

### Motor Power

- Voltage: ******\_****** V
- Current capacity: ******\_****** A
- Power supply type: ******\_******

### Control Board Power

- Voltage: ******\_****** V
- Separate from motor power? [ ] Yes [ ] No

---

## 10. Additional Information

### Calibration/Homing

- [ ] Motors have absolute encoders (no homing needed)
- [ ] Need homing sequence on startup
- [ ] Manual calibration required
- [ ] Other: ******\_******

### Special Considerations

Please describe any special requirements, quirks, or important notes about your hardware:

```
[Your notes here]
```

---

## 11. Testing Environment

### Available for Testing

- [ ] Full robot assembled and ready
- [ ] Individual motors can be tested
- [ ] Need to order/build hardware first
- [ ] Other: ******\_******

### Testing Safety

- [ ] Robot is mounted securely
- [ ] Clear workspace around robot
- [ ] Emergency stop accessible
- [ ] Other safety measures: ******\_******

---

## Submission

Once completed, save this file as `hardware_specs.md` in your project directory and share it with me. I will use this information to:

1. Create the RealRobotHardware plugin
2. Configure communication interfaces
3. Set up safety limits
4. Create a staged testing plan
5. Provide integration instructions

**Next Steps After Submission:**

1. I'll create a custom hardware integration package
2. Implement the driver based on your specs
3. Provide step-by-step testing procedures
4. Guide you through safe hardware integration
