#!/usr/bin/env python3
"""
Simple gripper test - just sends one command to close the gripper.
Run this AFTER the system is fully launched and stable.

Usage:
  python3 src/pick_place_arm/scripts/test_gripper.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():
    rclpy.init()
    node = Node("test_gripper")
    
    # Create action client
    client = ActionClient(node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
    
    node.get_logger().info("Waiting for gripper action server...")
    client.wait_for_server()
    node.get_logger().info("✓ Connected to gripper!")
    
    # Create goal
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = ['j7l', 'j7r']
    
    point = JointTrajectoryPoint()
    point.positions = [-0.02, 0.02]  # Close
    point.time_from_start = Duration(sec=2, nanosec=0)
    
    goal_msg.trajectory.points = [point]
    
    node.get_logger().info("Sending close gripper command...")
    future = client.send_goal_async(goal_msg)
    
    rclpy.spin_until_future_complete(node, future)
    
    goal_handle = future.result()
    if goal_handle.accepted:
        node.get_logger().info("✓ Goal accepted! Gripper should be closing...")
    else:
        node.get_logger().error("✗ Goal rejected!")
    
    node.get_logger().info("Test complete!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
