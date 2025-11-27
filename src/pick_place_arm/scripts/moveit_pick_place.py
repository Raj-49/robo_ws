#!/usr/bin/env python3
"""
Pick and Place using moveit_commander (ROS 2 Humble compatible)
Adapted from Panda workspace approach
"""

import rclpy
from rclpy.node import Node
import moveit_commander
import sys
import time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveItPickPlace(Node):
    def __init__(self):
        super().__init__('moveit_pick_place')
        
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Robot and scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Move groups
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # Set planning parameters
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_num_planning_attempts(10)
        
        # Gripper action client (for detachable joint)
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info("MoveIt Commander initialized successfully!")
        self.get_logger().info(f"Planning frame: {self.arm_group.get_planning_frame()}")
        self.get_logger().info(f"End effector: {self.arm_group.get_end_effector_link()}")
        
        # Predefined joint configurations
        self.home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
    def move_to_joints(self, joint_positions):
        """Move arm to joint configuration"""
        self.get_logger().info(f"Moving to joints: {joint_positions}")
        self.arm_group.go(joint_positions, wait=True)
        self.arm_group.stop()
        
    def move_to_pose(self, position, orientation=[0.0, 1.0, 0.0, 0.0]):
        """Move to Cartesian pose"""
        pose_goal = Pose()
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]
        
        self.get_logger().info(f"Planning to pose: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        
        self.arm_group.set_pose_target(pose_goal)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if not success:
            self.get_logger().error(f"Failed to reach pose {position}")
        
        return success
    
    def open_gripper(self):
        """Open gripper"""
        self.get_logger().info("Opening gripper...")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['j7l', 'j7r']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        self.gripper_client.send_goal_async(goal_msg)
        time.sleep(1.5)
    
    def close_gripper(self):
        """Close gripper"""
        self.get_logger().info("Closing gripper...")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['j7l', 'j7r']
        
        point = JointTrajectoryPoint()
        point.positions = [-0.032, 0.032]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        self.gripper_client.send_goal_async(goal_msg)
        time.sleep(2.5)
    
    def pick_and_place(self, box_pos, plate_pos):
        """Execute pick and place sequence"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🤖 PICK AND PLACE SEQUENCE")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"📦 Box: {box_pos}")
        self.get_logger().info(f"🍽️  Plate: {plate_pos}")
        
        # 1. Home
        self.get_logger().info("\n[1/10] Moving to home...")
        self.move_to_joints(self.home_joints)
        time.sleep(1.0)
        
        # 2. Open gripper
        self.get_logger().info("[2/10] Opening gripper...")
        self.open_gripper()
        
        # 3. Approach box (higher offset to ensure reachability)
        approach_pos = [box_pos[0], box_pos[1], box_pos[2] + 0.10]
        self.get_logger().info(f"[3/10] Approaching box at {approach_pos}...")
        if not self.move_to_pose(approach_pos):
            self.get_logger().error("Failed to approach box!")
            return False
        time.sleep(1.0)
        
        # 4. Descend to pick (lower offset for gripper)
        pick_pos = [box_pos[0], box_pos[1], box_pos[2] + 0.08]
        self.get_logger().info(f"[4/10] Descending to pick at {pick_pos}...")
        if not self.move_to_pose(pick_pos):
            self.get_logger().error("Failed to descend to pick!")
            return False
        time.sleep(1.0)
        
        # 5. Close gripper
        self.get_logger().info("[5/10] Closing gripper...")
        self.close_gripper()
        
        # 6. Lift
        self.get_logger().info("[6/10] Lifting box...")
        if not self.move_to_pose(approach_pos):
            self.get_logger().error("Failed to lift!")
            return False
        time.sleep(1.0)
        
        # 7. Move to plate approach
        plate_approach = [plate_pos[0], plate_pos[1], plate_pos[2] + 0.10]
        self.get_logger().info(f"[7/10] Moving to plate at {plate_approach}...")
        if not self.move_to_pose(plate_approach):
            self.get_logger().error("Failed to move to plate!")
            return False
        time.sleep(1.0)
        
        # 8. Descend to place
        place_pos = [plate_pos[0], plate_pos[1], plate_pos[2] + 0.08]
        self.get_logger().info(f"[8/10] Placing at {place_pos}...")
        if not self.move_to_pose(place_pos):
            self.get_logger().error("Failed to descend to place!")
            return False
        time.sleep(1.0)
        
        # 9. Open gripper
        self.get_logger().info("[9/10] Releasing...")
        self.open_gripper()
        
        # 10. Return home
        self.get_logger().info("[10/10] Returning home...")
        self.move_to_pose(plate_approach)
        time.sleep(1.0)
        self.move_to_joints(self.home_joints)
        time.sleep(1.0)
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("✅ Pick and place complete!")
        self.get_logger().info(f"{'='*60}\n")
        return True


def main():
    rclpy.init()
    node = MoveItPickPlace()
    
    # Wait for action server
    node.get_logger().info("Waiting for gripper action server...")
    node.gripper_client.wait_for_server()
    
    # Test with predefined coordinates from SDF
    box_pos = [0.093550, 0.192664, 0.453342]
    plate_pos = [-0.112035, -0.180573, 0.009269]
    
    node.pick_and_place(box_pos, plate_pos)
    
    moveit_commander.roscpp_shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
