# Real Hardware Integration - Execution Prompt

## When to Use This Prompt

Use this prompt when you are ready to integrate your actual physical hardware and want me to guide you through the process.

---

## Copy This Entire Section When Ready

```
I am ready to integrate my real hardware with the robotic arm system.

CONTEXT:
- I have completed virtual hardware testing successfully
- All software components are validated and working
- I have filled out the hardware specification form
- My hardware is assembled, powered, and ready for testing
- I have implemented all safety measures

HARDWARE SPECIFICATIONS:
[Paste your completed hardware_specs.md content here, or reference the file location]

CURRENT STATUS:
- Virtual hardware system: ✅ Working
- Real hardware: 🔌 Connected and ready
- Safety measures: ✅ In place
- Testing environment: ✅ Prepared

REQUEST:
Please execute the Real Hardware Integration Plan:

1. Review my hardware specifications
2. Implement the RealRobotHardware plugin based on my specs
3. Create all necessary configuration files
4. Set up communication interfaces
5. Implement safety limits and monitoring
6. Create staged testing procedures
7. Guide me through each testing stage step-by-step

SAFETY REQUIREMENTS:
- I understand this is real hardware that can cause damage
- I will follow all safety procedures exactly
- I will not skip any testing stages
- I have emergency stop readily accessible
- I will report any unexpected behavior immediately

INTEGRATION APPROACH:
Please follow the staged testing plan:
- Stage 1: Communication verification (no motion)
- Stage 2: Individual motor tests (limited range, slow speed)
- Stage 3: Multi-motor coordination (reduced speed)
- Stage 4: Full system validation (normal operation)
- Stage 5: Production deployment

After each stage, I will:
1. Execute the tests you provide
2. Report results and any issues
3. Wait for your approval before proceeding
4. Only move to next stage when current stage passes

EXPECTED DELIVERABLES:
1. Complete RealRobotHardware implementation
2. Motor-specific configuration files
3. Safety-validated launch files
4. Stage-by-stage testing scripts
5. Troubleshooting procedures
6. Emergency procedures documentation

Please begin by reviewing my hardware specifications and creating the implementation plan.
```

---

## What Happens Next

After you provide this prompt with your hardware specs, I will:

### Phase 1: Planning (15 minutes)

1. Analyze your hardware specifications
2. Create detailed implementation plan
3. Identify potential challenges
4. Request your approval to proceed

### Phase 2: Implementation (30-60 minutes)

1. Create `real_hardware_integration` package structure
2. Implement `RealRobotHardware` plugin
3. Create motor driver/communication layer
4. Set up configuration files
5. Implement safety monitoring
6. Create launch files

### Phase 3: Testing Preparation (15 minutes)

1. Create Stage 1 test scripts (communication only)
2. Create Stage 2 test scripts (single motor)
3. Create Stage 3 test scripts (coordination)
4. Create Stage 4 test scripts (full system)
5. Document expected results for each stage

### Phase 4: Guided Execution (Interactive)

1. Guide you through Stage 1 testing
2. Analyze results and troubleshoot if needed
3. Get your approval to proceed
4. Repeat for each subsequent stage
5. Validate full system operation

### Phase 5: Production Deployment

1. Create production launch configuration
2. Document operational procedures
3. Provide maintenance guidelines
4. Create troubleshooting guide

---

## Important Notes

### Before Using This Prompt

✅ **Complete these prerequisites:**

- [ ] Virtual hardware tests all passing
- [ ] Hardware specification form filled out completely
- [ ] Physical hardware assembled and powered
- [ ] Communication interface connected (USB/CAN/etc.)
- [ ] Emergency stop accessible
- [ ] Clear workspace around robot
- [ ] Safety measures in place

### During Integration

⚠️ **Safety rules:**

- Never skip testing stages
- Always have emergency stop ready
- Report unexpected behavior immediately
- Don't increase speed/range without approval
- Keep clear of robot workspace during motion
- Power off between major changes

### After Integration

📋 **Validation checklist:**

- [ ] All motors respond correctly
- [ ] Position feedback is accurate
- [ ] Safety limits are enforced
- [ ] Emergency stop works
- [ ] MoveIt can plan and execute
- [ ] No unexpected behaviors
- [ ] Performance meets requirements

---

## Example Hardware Specs Summary

When you use the prompt, include a summary like this:

```
HARDWARE SUMMARY:
- Motors: 6x Dynamixel XM430-W350 + 2x XL320 (gripper)
- Interface: USB (U2D2 adapter)
- Protocol: Dynamixel Protocol 2.0
- Baud Rate: 1000000
- Control Mode: Position control
- Feedback: Built-in encoders (4096 counts/rev)
- Power: 12V, 5A supply
- OS: Ubuntu 22.04
```

---

## Support During Integration

I will provide:

- ✅ Step-by-step instructions
- ✅ Code implementation
- ✅ Testing procedures
- ✅ Troubleshooting help
- ✅ Safety guidance
- ✅ Performance optimization

You should provide:

- ✅ Hardware specifications
- ✅ Test results and observations
- ✅ Error messages if any occur
- ✅ Confirmation before proceeding to next stage
- ✅ Questions about anything unclear

---

## Ready to Begin?

When you're ready:

1. Fill out `hardware_specs_form.md`
2. Ensure all prerequisites are met
3. Copy the execution prompt above
4. Paste it in a new message with your specs
5. I'll create your custom integration plan!
