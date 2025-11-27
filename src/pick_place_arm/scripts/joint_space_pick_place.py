#!/usr/bin/env python3
"""
Joint-Space Pick and Place
Uses predefined joint configurations instead of Cartesian IK
Run this in RViz to manually teach positions, then use them in the script
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class JointSpacePickPlace(Node):
    def __init__(self):
        super().__init__('joint_space_pick_place')
        
        # Action clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )
        
        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        
        self.current_joints = None
        
        # Wait for servers
        self.get_logger().info("Waiting for action servers...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info("✓ Action servers ready!")
        
        # Predefined joint configurations (TEACH THESE IN RVIZ FIRST!)
        # Use Option 7 to print current joint positions
        self.home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Example positions - REPLACE THESE with your taught positions
        self.above_red_box = [0.5, -0.5, 0.5, -1.0, 0.0, 0.5]  # Placeholder
        self.at_red_box = [0.5, -0.3, 0.3, -1.2, 0.0, 0.5]     # Placeholder
        self.above_red_plate = [-0.5, -0.5, 0.5, -1.0, 0.0, 0.5]  # Placeholder
        self.at_red_plate = [-0.5, -0.3, 0.3, -1.2, 0.0, 0.5]     # Placeholder
        
    def joint_callback(self, msg):
        """Store current joint positions"""
        try:
            # Extract arm joints (j1-j6)
            arm_joints = []
            for joint_name in ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']:
                idx = msg.name.index(joint_name)
                arm_joints.append(msg.position[idx])
            self.current_joints = arm_joints
        except (ValueError, IndexError):
            pass
    
    def move_to_joints(self, positions, duration=3.0):
        """Move arm to joint configuration"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        self.arm_client.send_goal_async(goal_msg)
        
    def open_gripper(self):
        """Open gripper"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['j7l', 'j7r']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        self.gripper_client.send_goal_async(goal_msg)
    
    def close_gripper(self):
        """Close gripper"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['j7l', 'j7r']
        
        point = JointTrajectoryPoint()
        point.positions = [-0.032, 0.032]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        self.gripper_client.send_goal_async(goal_msg)
    
    def pick_and_place_red(self):
        """Execute pick and place using joint waypoints"""
        self.get_logger().info("\n🤖 Starting Joint-Space Pick and Place")
        
        # 1. Home
        self.get_logger().info("[1/10] Moving to home...")
        self.move_to_joints(self.home)
        time.sleep(3.5)
        
        # 2. Open gripper
        self.get_logger().info("[2/10] Opening gripper...")
        self.open_gripper()
        time.sleep(1.5)
        
        # 3. Move above red box
        self.get_logger().info("[3/10] Moving above red box...")
        self.move_to_joints(self.above_red_box)
        time.sleep(3.5)
        
        # 4. Descend to red box
        self.get_logger().info("[4/10] Descending to red box...")
        self.move_to_joints(self.at_red_box)
        time.sleep(3.5)
        
        # 5. Close gripper
        self.get_logger().info("[5/10] Closing gripper...")
        self.close_gripper()
        time.sleep(2.5)
        
        # 6. Lift
        self.get_logger().info("[6/10] Lifting...")
        self.move_to_joints(self.above_red_box)
        time.sleep(3.5)
        
        # 7. Move above red plate
        self.get_logger().info("[7/10] Moving above red plate...")
        self.move_to_joints(self.above_red_plate)
        time.sleep(4.0)
        
        # 8. Descend to red plate
        self.get_logger().info("[8/10] Descending to red plate...")
        self.move_to_joints(self.at_red_plate)
        time.sleep(3.5)
        
        # 9. Open gripper
        self.get_logger().info("[9/10] Opening gripper...")
        self.open_gripper()
        time.sleep(1.5)
        
        # 10. Return home
        self.get_logger().info("[10/10] Returning home...")
        self.move_to_joints(self.above_red_plate)
        time.sleep(2.0)
        self.move_to_joints(self.home)
        time.sleep(3.5)
        
        self.get_logger().info("✅ Pick and place complete!\n")
    
    def print_current_joints(self):
        """Print current joint positions for teaching"""
        if self.current_joints:
            joints_str = "[" + ", ".join([f"{j:.3f}" for j in self.current_joints]) + "]"
            self.get_logger().info(f"Current joints: {joints_str}")
            print(f"\nCopy this: {joints_str}\n")
        else:
            self.get_logger().warn("No joint state received yet")
    
    def run_menu(self):
        """Interactive menu"""
        while True:
            print("\n" + "="*60)
            print("🤖 JOINT-SPACE PICK AND PLACE")
            print("="*60)
            print("\nOptions:")
            print("  1. Execute pick and place (red box)")
            print("  2. Move to HOME")
            print("  3. Move ABOVE red box (teach this in RViz first!)")
            print("  4. Move AT red box (teach this in RViz first!)")
            print("  5. Move ABOVE red plate (teach this in RViz first!)")
            print("  6. Move AT red plate (teach this in RViz first!)")
            print("  7. Print current joint positions")
            print("  0. Exit")
            print()
            
            choice = input("Enter your choice: ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                self.pick_and_place_red()
            elif choice == '2':
                self.move_to_joints(self.home)
                time.sleep(3.5)
            elif choice == '3':
                self.move_to_joints(self.above_red_box)
                time.sleep(3.5)
            elif choice == '4':
                self.move_to_joints(self.at_red_box)
                time.sleep(3.5)
            elif choice == '5':
                self.move_to_joints(self.above_red_plate)
                time.sleep(3.5)
            elif choice == '6':
                self.move_to_joints(self.at_red_plate)
                time.sleep(3.5)
            elif choice == '7':
                self.print_current_joints()
            else:
                print("Invalid choice!")


def main():
    rclpy.init()
    node = JointSpacePickPlace()
    
    # Give time for joint state to arrive
    time.sleep(1.0)
    
    node.run_menu()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
