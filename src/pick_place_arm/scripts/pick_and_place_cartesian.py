#!/usr/bin/env python3
"""
Pick and place using MoveIt Commander for Cartesian control.
Moves gripper to actual XYZ positions of boxes and plates.

Usage:
  python3 install/pick_place_arm/share/pick_place_arm/scripts/pick_and_place_cartesian.py --ros-args -p target_color:=R
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time

# MoveIt imports
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory


class PickAndPlaceCartesian(Node):
    def __init__(self):
        super().__init__("pick_and_place_cartesian")

        # Parameters
        self.declare_parameter("target_color", "R")
        target_param = self.get_parameter("target_color").value.upper()
        
        if target_param == "ALL":
            self.target_colors = ["R", "G", "B"]
        else:
            self.target_colors = [target_param]

        # State
        self.current_target_index = 0
        self.detected_boxes = {}
        self.detected_plates = {}

        # Known positions
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

        # Initialize MoveIt
        self.get_logger().info("Initializing MoveIt Commander...")
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Arm group
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        self.arm_group.set_max_acceleration_scaling_factor(0.1)
        
        # Gripper group
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        self.get_logger().info(f"Planning frame: {self.arm_group.get_planning_frame()}")
        self.get_logger().info(f"End effector: {self.arm_group.get_end_effector_link()}")
        
        # Subscribe to vision
        self.sub = self.create_subscription(
            String, "/box_coordinates", self.coords_callback, 10
        )
        
        self.get_logger().info(f"Pick and Place initialized for colors: {self.target_colors}")
        
        # Move to home
        self.get_logger().info("Moving to home position...")
        self.arm_group.set_named_target("home")
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        # Timer
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
        """Check and execute pick-and-place."""
        if self.current_target_index >= len(self.target_colors):
            return
            
        target_color = self.target_colors[self.current_target_index]
        
        if target_color in self.detected_boxes and target_color in self.detected_plates:
            self.get_logger().info(f"Starting pick-and-place for {target_color}")
            self.execute_pick_and_place(target_color)
            self.current_target_index += 1

    def move_to_pose(self, x, y, z, description="target"):
        """Move end effector to XYZ position."""
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # Gripper pointing down
        pose_goal.orientation.x = 0.0
        pose_goal.orientation.y = 1.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.0
        
        self.arm_group.set_pose_target(pose_goal)
        
        self.get_logger().info(f"Planning to {description} at ({x:.3f}, {y:.3f}, {z:.3f})")
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if success:
            self.get_logger().info(f"✓ Reached {description}")
        else:
            self.get_logger().warn(f"✗ Failed to reach {description}")
        
        return success

    def open_gripper(self):
        """Open gripper."""
        self.get_logger().info("Opening gripper")
        self.gripper_group.set_joint_value_target([0.05, -0.05])
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()

    def close_gripper(self):
        """Close gripper."""
        self.get_logger().info("Closing gripper")
        self.gripper_group.set_joint_value_target([0.0, 0.0])
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()

    def execute_pick_and_place(self, color):
        """Execute full pick-and-place sequence."""
        self.get_logger().info(f"=== Picking {color} box ===")
        
        box_pos = self.box_positions[color]
        plate_pos = self.plate_positions[color]
        
        # 1. Move to home
        self.get_logger().info("Moving to home")
        self.arm_group.set_named_target("home")
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        # 2. Open gripper
        self.open_gripper()
        time.sleep(0.5)
        
        # 3. Move above box
        above_box = [box_pos[0], box_pos[1], box_pos[2] + 0.15]
        if self.move_to_pose(*above_box, "above box"):
            time.sleep(0.5)
            
            # 4. Move to box
            if self.move_to_pose(*box_pos, "box"):
                time.sleep(0.5)
                
                # 5. Close gripper
                self.close_gripper()
                time.sleep(1.0)
                
                # 6. Lift box
                self.move_to_pose(*above_box, "lift")
                time.sleep(0.5)
        
        # 7. Move to home
        self.get_logger().info("Returning to home")
        self.arm_group.set_named_target("home")
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        time.sleep(0.5)
        
        # 8. Move above plate
        above_plate = [plate_pos[0], plate_pos[1], plate_pos[2] + 0.15]
        if self.move_to_pose(*above_plate, "above plate"):
            time.sleep(0.5)
            
            # 9. Move to plate
            place_height = [plate_pos[0], plate_pos[1], plate_pos[2] + 0.03]
            if self.move_to_pose(*place_height, "plate"):
                time.sleep(0.5)
                
                # 10. Open gripper
                self.open_gripper()
                time.sleep(1.0)
                
                # 11. Lift
                self.move_to_pose(*above_plate, "lift from plate")
                time.sleep(0.5)
        
        # 12. Return home
        self.get_logger().info("Final return to home")
        self.arm_group.set_named_target("home")
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        self.get_logger().info(f"=== Completed {color} ===")
        
        if self.current_target_index >= len(self.target_colors):
            self.get_logger().info("All operations completed!")
            moveit_commander.roscpp_shutdown()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PickAndPlaceCartesian()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
