#!/usr/bin/env python3
"""
System-wide integration test using MoveIt action interface.
Tests end-to-end functionality without Gazebo.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from sensor_msgs.msg import JointState
import time

class MoveItIntegrationTest(Node):
    def __init__(self):
        super().__init__('moveit_integration_test')
        self.get_logger().info('Initializing MoveIt Integration Test...')
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.latest_joint_state = None
        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action server available!')
    
    def joint_state_callback(self, msg):
        self.latest_joint_state = msg
    
    def create_joint_goal(self, joint_positions, joint_names=None):
        """Create a motion plan request with joint goals"""
        if joint_names is None:
            joint_names = self.joint_names
        
        goal_msg = MoveGroup.Goal()
        
        # Set planning group
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Set joint constraints
        goal_msg.request.goal_constraints.append(Constraints())
        for name, position in zip(joint_names, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_msg.request.goal_constraints[0].joint_constraints.append(joint_constraint)
        
        # Set workspace parameters
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        # Plan and execute
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        return goal_msg
    
    def send_goal_and_wait(self, goal_msg, description):
        """Send goal and wait for result"""
        self.get_logger().info(f'Sending goal: {description}')
        
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'✗ Goal rejected: {description}')
            return False
        
        self.get_logger().info(f'Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info(f'✓ {description} succeeded!')
            return True
        else:
            self.get_logger().error(f'✗ {description} failed with error code: {result.error_code.val}')
            return False

def main():
    rclpy.init()
    
    # Wait for system to be ready
    print('Waiting for system to initialize...')
    time.sleep(3)
    
    tester = MoveItIntegrationTest()
    
    # Wait for joint states
    print('Waiting for joint states...')
    for i in range(20):
        rclpy.spin_once(tester, timeout_sec=0.5)
        if tester.latest_joint_state is not None:
            break
    
    if tester.latest_joint_state is None:
        tester.get_logger().error('No joint states received!')
        tester.destroy_node()
        rclpy.shutdown()
        return
    
    # Run tests
    results = []
    
    tester.get_logger().info('\n=== Test 1: Move to Home (zero) ===')
    zero_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal = tester.create_joint_goal(zero_positions)
    results.append(tester.send_goal_and_wait(goal, 'Move to home'))
    time.sleep(2)
    
    tester.get_logger().info('\n=== Test 2: Move to Pick Position ===')
    pick_positions = [-1.8909, 1.57, -1.4139, -0.3383, 0.1995, 0.2949]
    goal = tester.create_joint_goal(pick_positions)
    results.append(tester.send_goal_and_wait(goal, 'Move to pick'))
    time.sleep(2)
    
    tester.get_logger().info('\n=== Test 3: Return to Home ===')
    goal = tester.create_joint_goal(zero_positions)
    results.append(tester.send_goal_and_wait(goal, 'Return to home'))
    
    # Summary
    tester.get_logger().info('\n=== Test Summary ===')
    for i, result in enumerate(results, 1):
        tester.get_logger().info(f'Test {i}: {"PASS" if result else "FAIL"}')
    
    if all(results):
        tester.get_logger().info('✓ All integration tests passed!')
    else:
        tester.get_logger().error('✗ Some integration tests failed')
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
