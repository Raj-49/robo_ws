#!/usr/bin/env python3
"""
Test script to validate JointTrajectoryController behavior with FakeRobotHardware.
Sends a simple trajectory and monitors tracking accuracy.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time

class ControllerTester(Node):
    def __init__(self):
        super().__init__('controller_tester')
        
        # Action client for arm controller
        self.arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Subscribe to joint states for monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.latest_joint_state = None
        self.get_logger().info('Controller Tester initialized')
    
    def joint_state_callback(self, msg):
        self.latest_joint_state = msg
    
    def send_simple_trajectory(self):
        """Send a simple trajectory: move j1 from 0 to 1.0 rad in 2 seconds"""
        self.get_logger().info('Waiting for action server...')
        self.arm_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        # Point 1: Start position (0,0,0,0,0,0)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start.sec = 0
        point1.time_from_start.nanosec = 0
        
        # Point 2: Move j1 to 1.0 rad
        point2 = JointTrajectoryPoint()
        point2.positions = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point2.time_from_start.sec = 2
        point2.time_from_start.nanosec = 0
        
        # Point 3: Return to zero
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point3.time_from_start.sec = 4
        point3.time_from_start.nanosec = 0
        
        goal_msg.trajectory.points = [point1, point2, point3]
        
        self.get_logger().info('Sending trajectory goal...')
        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f'Result: {result.error_code}')
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✓ Trajectory executed successfully!')
            return True
        else:
            self.get_logger().error(f'✗ Trajectory failed with error code: {result.error_code}')
            return False
    
    def monitor_joint_states(self, duration=5.0):
        """Monitor joint state publication rate"""
        self.get_logger().info(f'Monitoring joint states for {duration}s...')
        
        count = 0
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is not None:
                count += 1
        
        elapsed = time.time() - start_time
        rate = count / elapsed
        
        self.get_logger().info(f'Joint state publication rate: {rate:.1f} Hz')
        
        if rate > 50:  # Expect at least 50Hz
            self.get_logger().info('✓ Joint state rate is acceptable')
            return True
        else:
            self.get_logger().warn('✗ Joint state rate is too low!')
            return False

def main():
    rclpy.init()
    tester = Node('controller_tester')
    
    # Wait for joint states to start publishing
    tester.get_logger().info('Waiting for system to initialize...')
    time.sleep(3)
    
    tester = ControllerTester()
    
    # Test 1: Monitor joint state rate
    tester.get_logger().info('=== Test 1: Joint State Broadcaster ===')
    test1_pass = tester.monitor_joint_states(duration=3.0)
    
    # Test 2: Send trajectory
    tester.get_logger().info('\n=== Test 2: Trajectory Execution ===')
    test2_pass = tester.send_simple_trajectory()
    
    # Summary
    tester.get_logger().info('\n=== Test Summary ===')
    tester.get_logger().info(f'Joint State Test: {"PASS" if test1_pass else "FAIL"}')
    tester.get_logger().info(f'Trajectory Test: {"PASS" if test2_pass else "FAIL"}')
    
    if test1_pass and test2_pass:
        tester.get_logger().info('✓ All tests passed!')
    else:
        tester.get_logger().error('✗ Some tests failed')
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
