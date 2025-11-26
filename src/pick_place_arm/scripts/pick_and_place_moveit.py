#!/usr/bin/env python3
"""
Pick and place using standard MoveIt Python API for custom arm.
Works with joints: j1-j6 (arm) + j7l, j7r (gripper)

Usage:
  python3 install/pick_place_arm/share/pick_place_arm/scripts/pick_and_place_moveit.py --ros-args -p target_color:=R
  python3 install/pick_place_arm/share/pick_place_arm/scripts/pick_and_place_moveit.py --ros-args -p target_color:=ALL
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class PickAndPlaceMoveIt(Node):
    def __init__(self):
        super().__init__("pick_and_place_moveit")

        # Parameters
        self.declare_parameter("target_color", "R")
        target_param = self.get_parameter("target_color").value.upper()
        
        if target_param == "ALL":
            self.target_colors = ["R", "G", "B"]
        else:
            self.target_colors = [target_param]

        # State tracking
        self.current_target_index = 0
        self.detected_boxes = {}
        self.detected_plates = {}

        # Known positions from world file
        self.box_positions = {
            "R": [0.093550, 0.192664, 0.453342],
            "G": [0.150000, 0.192664, 0.453342],
            "B": [0.200000, 0.192664, 0.453342],
        }
        
        self.plate_positions = {
            "R": [-0.112, -0.181, 0.05],
            "G": [0.000, -0.181, 0.05],
            "B": [0.112, -0.181, 0.05],
        }

        # Action clients for arm and gripper
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )

        # Joint names
        self.arm_joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.gripper_joints = ['j7l', 'j7r']

        # Home position (all zeros)
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Subscribe to vision
        self.sub = self.create_subscription(
            String, "/box_coordinates", self.coords_callback, 10
        )
        
        self.get_logger().info(f"Pick and Place initialized for colors: {self.target_colors}")
        self.get_logger().info("Waiting for action servers...")
        
        # Wait for action servers
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        self.get_logger().info("Action servers ready!")
        
        # Move to home
        self.move_arm_to_joints(self.home_position, 3.0)
        time.sleep(3.5)
        
        # Timer to check detections
        self.create_timer(2.0, self.check_and_execute)

    def coords_callback(self, msg):
        """Store detected coordinates."""
        try:
            parts = msg.data.split(",")
            obj_id = parts[0]
            
            if "_" in obj_id:
                obj_type, color = obj_id.split("_")
                
                if obj_type == "box":
                    self.detected_boxes[color] = True
                elif obj_type == "plate":
                    self.detected_plates[color] = True
                        
        except Exception as e:
            self.get_logger().error(f"Error parsing: {e}")

    def check_and_execute(self):
        """Check if target detected and execute pick-and-place."""
        if self.current_target_index >= len(self.target_colors):
            return
            
        target_color = self.target_colors[self.current_target_index]
        
        if target_color in self.detected_boxes and target_color in self.detected_plates:
            self.get_logger().info(f"Starting pick-and-place for {target_color}")
            self.execute_pick_and_place(target_color)
            self.current_target_index += 1

    def move_arm_to_joints(self, positions, duration_sec=2.0):
        """Move arm to joint positions."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f"Moving arm to: {positions}")
        future = self.arm_client.send_goal_async(goal_msg)
        return future

    def move_gripper(self, open_gripper=True, duration_sec=1.0):
        """Open or close gripper."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        if open_gripper:
            point.positions = [-0.05, 0.05]  # Open
        else:
            point.positions = [0.0, 0.0]  # Close
            
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        action = "Opening" if open_gripper else "Closing"
        self.get_logger().info(f"{action} gripper")
        future = self.gripper_client.send_goal_async(goal_msg)
        return future

    def execute_pick_and_place(self, color):
        """Execute full pick-and-place sequence."""
        self.get_logger().info(f"=== Picking {color} box ===")
        
        # NOTE: This is a simplified version using joint space control
        # For Cartesian control, you would need to use MoveIt's move_group interface
        # For now, we'll just demonstrate the sequence with home position
        
        # 1. Move to home
        self.move_arm_to_joints(self.home_position, 2.0)
        time.sleep(2.5)
        
        # 2. Open gripper
        self.move_gripper(open_gripper=True, duration_sec=1.0)
        time.sleep(1.5)
        
        # 3. Move to approach position (example - would need inverse kinematics)
        # For a real implementation, you'd use MoveIt's compute_cartesian_path
        self.get_logger().info(f"Would move to box at {self.box_positions[color]}")
        
        # 4. Close gripper
        time.sleep(1.0)
        self.move_gripper(open_gripper=False, duration_sec=1.0)
        time.sleep(1.5)
        
        # 5. Return to home
        self.move_arm_to_joints(self.home_position, 2.0)
        time.sleep(2.5)
        
        # 6. Move to plate (would need IK)
        self.get_logger().info(f"Would move to plate at {self.plate_positions[color]}")
        time.sleep(1.0)
        
        # 7. Open gripper
        self.move_gripper(open_gripper=True, duration_sec=1.0)
        time.sleep(1.5)
        
        # 8. Return home
        self.move_arm_to_joints(self.home_position, 2.0)
        time.sleep(2.5)
        
        self.get_logger().info(f"=== Completed {color} sequence ===")
        
        if self.current_target_index >= len(self.target_colors):
            self.get_logger().info("All operations completed!")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PickAndPlaceMoveIt()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
