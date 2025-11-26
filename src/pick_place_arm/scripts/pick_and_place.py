#!/usr/bin/env python3
"""
Pick and place node for colored boxes to matching colored plates.
Uses vision for color detection and predefined positions for motion planning.

Usage:
  ros2 run pick_place_arm pick_and_place.py --ros-args -p target_color:=R
  ros2 run pick_place_arm pick_and_place.py --ros-args -p target_color:=G
  ros2 run pick_place_arm pick_and_place.py --ros-args -p target_color:=B
  ros2 run pick_place_arm pick_and_place.py --ros-args -p target_color:=ALL
"""

from threading import Thread, Lock
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
import sys
import os

# Add pymoveit2 to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../Panda/install/pymoveit2/lib/pymoveit2'))

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        target_param = self.get_parameter("target_color").value.upper()
        
        # Determine which colors to pick
        if target_param == "ALL":
            self.target_colors = ["R", "G", "B"]
        else:
            self.target_colors = [target_param]

        # Flags and state
        self.current_target_index = 0
        self.detected_boxes = {}  # Store detected box positions
        self.detected_plates = {}  # Store detected plate positions
        self.lock = Lock()

        self.callback_group = ReentrantCallbackGroup()

        # Known positions from world file (fallback if vision coordinates not available)
        self.box_positions = {
            "R": [0.093550, 0.192664, 0.453342],
            "G": [0.150000, 0.192664, 0.453342],  # Estimate
            "B": [0.200000, 0.192664, 0.453342],  # Estimate
        }
        
        self.plate_positions = {
            "R": [-0.112, -0.181, 0.009],  # Estimate
            "G": [0.000, -0.181, 0.009],   # Estimate
            "B": [0.112, -0.181, 0.009],   # Estimate
        }

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set velocity & acceleration
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Subscriber for vision coordinates
        self.sub = self.create_subscription(
            String, "/box_coordinates", self.coords_callback, 10
        )
        
        self.get_logger().info(f"Pick and Place initialized for colors: {self.target_colors}")
        self.get_logger().info("Waiting for vision detection...")

        # Predefined joint positions
        self.home_joints = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]

        # Move to home position
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()
        
        # Timer to check if we have enough detections
        self.create_timer(2.0, self.check_and_execute)

    def coords_callback(self, msg):
        """Store detected coordinates from vision system."""
        with self.lock:
            try:
                parts = msg.data.split(",")
                obj_id = parts[0]  # e.g., "box_R" or "plate_G"
                
                # Parse object type and color
                if "_" in obj_id:
                    obj_type, color = obj_id.split("_")
                    
                    # Store detection (currently pixel coords, but we'll use known positions)
                    if obj_type == "box":
                        self.detected_boxes[color] = True
                        self.get_logger().info(f"Detected box: {color}")
                    elif obj_type == "plate":
                        self.detected_plates[color] = True
                        self.get_logger().info(f"Detected plate: {color}")
                        
            except Exception as e:
                self.get_logger().error(f"Error parsing coordinates: {e}")

    def check_and_execute(self):
        """Check if we have detected the required objects and execute pick-and-place."""
        with self.lock:
            # Check if current target is detected
            if self.current_target_index >= len(self.target_colors):
                return  # All targets completed
                
            target_color = self.target_colors[self.current_target_index]
            
            # Check if both box and plate are detected
            if target_color in self.detected_boxes and target_color in self.detected_plates:
                self.get_logger().info(f"Starting pick-and-place for {target_color}")
                # Execute in separate thread to avoid blocking
                Thread(target=self.execute_pick_and_place, args=(target_color,), daemon=True).start()
                self.current_target_index += 1

    def execute_pick_and_place(self, color):
        """Execute pick and place sequence for a specific color."""
        self.get_logger().info(f"=== Picking {color} box ===")
        
        # Get positions
        box_pos = self.box_positions[color]
        plate_pos = self.plate_positions[color]
        
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]  # Gripper pointing down

        # 1. Move to home
        self.get_logger().info("Moving to home position")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 2. Open gripper
        self.get_logger().info("Opening gripper")
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 3. Move above box
        above_box = [box_pos[0], box_pos[1], box_pos[2] + 0.2]
        self.get_logger().info(f"Moving above box at {above_box}")
        self.moveit2.move_to_pose(position=above_box, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 4. Move down to box
        self.get_logger().info(f"Moving to box at {box_pos}")
        self.moveit2.move_to_pose(position=box_pos, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 5. Close gripper
        self.get_logger().info("Closing gripper")
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 6. Lift box
        self.get_logger().info("Lifting box")
        self.moveit2.move_to_pose(position=above_box, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 7. Move to home
        self.get_logger().info("Moving to home")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 8. Move above plate
        above_plate = [plate_pos[0], plate_pos[1], plate_pos[2] + 0.2]
        self.get_logger().info(f"Moving above plate at {above_plate}")
        self.moveit2.move_to_pose(position=above_plate, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 9. Move down to plate
        self.get_logger().info(f"Placing on plate at {plate_pos}")
        place_pos = [plate_pos[0], plate_pos[1], plate_pos[2] + 0.05]  # Slightly above plate
        self.moveit2.move_to_pose(position=place_pos, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 10. Open gripper
        self.get_logger().info("Releasing box")
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 11. Lift up
        self.get_logger().info("Lifting gripper")
        self.moveit2.move_to_pose(position=above_plate, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 12. Return to home
        self.get_logger().info("Returning to home")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        self.get_logger().info(f"=== Completed {color} box ===")
        
        # Check if all targets completed
        if self.current_target_index >= len(self.target_colors):
            self.get_logger().info("All pick-and-place operations completed!")
            if len(self.target_colors) == 1:
                rclpy.shutdown()


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
