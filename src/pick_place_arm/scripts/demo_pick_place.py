#!/usr/bin/env python3
"""
Simple pick and place demo - spawns a test box and demonstrates pick-and-place.
Sequence:
1. Spawn test box below gripper
2. Close gripper (pick)
3. Move to place_1 position
4. Open gripper (release)

Usage:
  python3 install/pick_place_arm/share/pick_place_arm/scripts/demo_pick_place.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
import time


class DemoPickPlace(Node):
    def __init__(self):
        super().__init__("demo_pick_place")

        # Action clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )

        # Joint names
        self.arm_joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.gripper_joints = ['j7l', 'j7r']

        # Publishers for attach/detach
        self.attach_pub = self.create_publisher(Empty, '/box1/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/box1/detach', 10)

        # Positions
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # User provided: j1=0, j2=1.57, j3=0, j4=-1.57, j5=0, j6=0
        self.place_1_position = [0.0, 1.57, 0.0, -1.57, 0.0, 0.0]

        self.get_logger().info("Demo Pick and Place Starting...")
        self.get_logger().info("Waiting for action servers...")
        
        # Wait for servers
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        self.get_logger().info("✓ Action servers ready!")
        
        # Execute demo
        self.run_demo()

    def move_arm(self, positions, duration_sec=3.0, description="position"):
        """Move arm to joint positions."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f"Moving arm to {description}: {positions}")
        self.arm_client.send_goal_async(goal_msg)

    def open_gripper(self):
        """Open gripper."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # Open (both at zero)
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info("Opening gripper")
        self.gripper_client.send_goal_async(goal_msg)

    def close_gripper(self):
        """Close gripper (pick) - closes to box thickness."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        # Close around 0.06m box (like Panda)
        # j7l: moves negative (limit: -0.07 to 0)
        # j7r: moves positive (limit: 0 to 0.07)
        point.positions = [-0.032, 0.032]  # Grips 0.06m box (slightly tighter)
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info("Closing gripper (PICK) - closing to box thickness")
        self.gripper_client.send_goal_async(goal_msg)

    def run_demo(self):
        """Execute the demo sequence."""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("DEMO PICK AND PLACE SEQUENCE")
        self.get_logger().info("="*50)
        self.get_logger().info("Assuming: Arm at HOME, Gripper OPEN, Box below gripper")
        
        # Step 1: Close gripper (PICK)
        self.get_logger().info("\n[1/5] PICKING - Closing gripper on box1")
        self.close_gripper()
        time.sleep(3.0)
        
        # Note: Attachment is now handled by gripper_attachment_node.py
        # which watches for contact + closed gripper
        
        # Step 2: Move to place_1
        self.get_logger().info("\n[2/5] Moving to PLACE_1 position with box")
        self.move_arm(self.place_1_position, 5.0, "place_1")
        self.get_logger().info("Waiting 5s after reaching position...")
        time.sleep(10.0) # 5s move + 5s wait
        
        # Step 3: Open gripper (RELEASE)
        self.get_logger().info("\n[3/5] RELEASING - Opening gripper")
        self.open_gripper()
        time.sleep(1.0)
        
        # Note: Detachment handled by gripper_attachment_node.py
        
        # Done
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("✓ DEMO COMPLETE!")
        self.get_logger().info("="*50 + "\n")
        
        self.get_logger().info("Demo finished. Shutting down...")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = DemoPickPlace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
