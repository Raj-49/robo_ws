#!/usr/bin/env python3
"""
Hardware Interface Logger - Monitor read/write interactions between
controller manager and FakeRobotHardware plugin.

This script subscribes to joint states and controller commands to visualize
the complete data flow through the ros2_control stack.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Header
import time
from datetime import datetime
from collections import deque

class HardwareInterfaceLogger(Node):
    def __init__(self):
        super().__init__('hardware_interface_logger')
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.arm_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.arm_state_callback,
            10
        )
        
        self.gripper_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/gripper_controller/controller_state',
            self.gripper_state_callback,
            10
        )
        
        # Data storage
        self.latest_joint_state = None
        self.latest_arm_desired = None
        self.latest_gripper_desired = None
        
        # Statistics
        self.read_count = 0
        self.write_count = 0
        self.start_time = time.time()
        
        # Rate tracking
        self.joint_state_times = deque(maxlen=100)
        
        # Create timer for periodic logging
        self.timer = self.create_timer(1.0, self.log_summary)
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('Hardware Interface Logger Started')
        self.get_logger().info('Monitoring read/write interactions between controllers and hardware')
        self.get_logger().info('=' * 80)
    
    def joint_state_callback(self, msg):
        """Called when hardware interface publishes joint states (READ operation)"""
        self.latest_joint_state = msg
        self.read_count += 1
        self.joint_state_times.append(time.time())
        
        # Log detailed read every 100 messages
        if self.read_count % 100 == 0:
            self.log_read_operation(msg)
    
    def arm_state_callback(self, msg):
        """Called when arm controller publishes desired states (WRITE operation)"""
        self.latest_arm_desired = msg
        self.write_count += 1
        
        # Log detailed write every 100 messages
        if self.write_count % 100 == 0:
            self.log_write_operation('arm', msg)
    
    def gripper_state_callback(self, msg):
        """Called when gripper controller publishes desired states"""
        self.latest_gripper_desired = msg
    
    def log_read_operation(self, msg):
        """Log hardware READ operation (hardware -> controller)"""
        self.get_logger().info('─' * 80)
        self.get_logger().info(f'[READ #{self.read_count}] Hardware Interface → Controllers')
        self.get_logger().info(f'  Timestamp: {datetime.now().strftime("%H:%M:%S.%f")[:-3]}')
        
        # Display joint positions
        joint_str = '  Positions: '
        for name, pos in zip(msg.name[:6], msg.position[:6]):
            joint_str += f'{name}={pos:+.3f} '
        self.get_logger().info(joint_str)
        
        # Display velocities if available
        if msg.velocity:
            vel_str = '  Velocities: '
            for name, vel in zip(msg.name[:6], msg.velocity[:6]):
                vel_str += f'{name}={vel:+.3f} '
            self.get_logger().info(vel_str)
    
    def log_write_operation(self, controller_name, msg):
        """Log hardware WRITE operation (controller -> hardware)"""
        self.get_logger().info('─' * 80)
        self.get_logger().info(f'[WRITE #{self.write_count}] {controller_name.upper()} Controller → Hardware Interface')
        self.get_logger().info(f'  Timestamp: {datetime.now().strftime("%H:%M:%S.%f")[:-3]}')
        
        # Display desired positions
        if msg.desired.positions:
            des_str = '  Desired: '
            for name, pos in zip(msg.joint_names[:6], msg.desired.positions[:6]):
                des_str += f'{name}={pos:+.3f} '
            self.get_logger().info(des_str)
        
        # Display actual positions
        if msg.actual.positions:
            act_str = '  Actual:  '
            for name, pos in zip(msg.joint_names[:6], msg.actual.positions[:6]):
                act_str += f'{name}={pos:+.3f} '
            self.get_logger().info(act_str)
        
        # Display error
        if msg.error.positions:
            err_str = '  Error:   '
            for name, err in zip(msg.joint_names[:6], msg.error.positions[:6]):
                err_str += f'{name}={err:+.3f} '
            self.get_logger().info(err_str)
    
    def log_summary(self):
        """Log periodic summary of system status"""
        elapsed = time.time() - self.start_time
        
        # Calculate rates
        read_rate = self.read_count / elapsed if elapsed > 0 else 0
        
        # Calculate instantaneous rate from recent timestamps
        if len(self.joint_state_times) >= 2:
            time_diff = self.joint_state_times[-1] - self.joint_state_times[0]
            instant_rate = (len(self.joint_state_times) - 1) / time_diff if time_diff > 0 else 0
        else:
            instant_rate = 0
        
        self.get_logger().info('═' * 80)
        self.get_logger().info(f'SYSTEM STATUS @ {datetime.now().strftime("%H:%M:%S")}')
        self.get_logger().info(f'  Uptime: {elapsed:.1f}s')
        self.get_logger().info(f'  READ operations:  {self.read_count:6d} total | {read_rate:6.1f} Hz avg | {instant_rate:6.1f} Hz current')
        self.get_logger().info(f'  WRITE operations: {self.write_count:6d} total')
        
        if self.latest_joint_state and self.latest_arm_desired:
            self.get_logger().info('')
            self.get_logger().info('  Current Joint States (from hardware):')
            for i, name in enumerate(self.latest_joint_state.name[:6]):
                pos = self.latest_joint_state.position[i]
                vel = self.latest_joint_state.velocity[i] if self.latest_joint_state.velocity else 0.0
                self.get_logger().info(f'    {name}: pos={pos:+.4f} rad, vel={vel:+.4f} rad/s')
            
            if self.latest_arm_desired.desired.positions:
                self.get_logger().info('')
                self.get_logger().info('  Desired Joint States (from controller):')
                for i, name in enumerate(self.latest_arm_desired.joint_names[:6]):
                    des = self.latest_arm_desired.desired.positions[i]
                    act = self.latest_arm_desired.actual.positions[i]
                    err = self.latest_arm_desired.error.positions[i]
                    self.get_logger().info(f'    {name}: desired={des:+.4f}, actual={act:+.4f}, error={err:+.4f}')
        
        self.get_logger().info('═' * 80)

def main():
    rclpy.init()
    
    logger = HardwareInterfaceLogger()
    
    print("\n" + "=" * 80)
    print("HARDWARE INTERFACE LOGGER")
    print("=" * 80)
    print("\nMonitoring:")
    print("  • /joint_states                        - Hardware READ operations")
    print("  • /arm_controller/controller_state     - Controller WRITE operations")
    print("  • /gripper_controller/controller_state - Gripper commands")
    print("\nPress Ctrl+C to stop\n")
    print("=" * 80 + "\n")
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("\n" + "=" * 80)
        print("Shutting down logger...")
        print("=" * 80)
    
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
