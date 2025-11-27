#!/usr/bin/env python3
"""
Inverse Kinematics Utilities
----------------------------
Helper class to calculate joint angles from Cartesian poses
using MoveIt's /compute_ik service.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
import math

class IKSolver:
    """
    Inverse Kinematics Solver using MoveIt's /compute_ik service
    """
    def __init__(self, node, group_name="arm", base_frame="world", ik_timeout=1.0):
        self.node = node
        self.group_name = group_name
        self.base_frame = base_frame
        self.ik_timeout = ik_timeout
        
        # Service client
        self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        
        # Wait for service (non-blocking check)
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("/compute_ik service not available yet")

    def get_ik(self, target_pose, current_joint_state=None, avoid_collisions=True):
        """
        Calculate joint angles for a target pose.
        Tries multiple gripper orientations (yaw angles) if the default fails.
        
        Args:
            target_pose: [x, y, z] list or geometry_msgs/Pose
            current_joint_state: Optional sensor_msgs/JointState (seed)
            avoid_collisions: Whether to check for collisions (default: True)
            
        Returns:
            list of joint positions (radians), or None if no solution found
        """
        if not self.ik_client.service_is_ready():
            self.node.get_logger().error("IK service not ready")
            return None
            
        # Define a list of orientations to try (Quaternions)
        # All pointing DOWN (rotation around Y by 180), but with different Yaw rotations
        # 1. Default: 180 deg around Y. q = [0, 1, 0, 0]
        # 2. 90 deg yaw: q = [0.707, 0.707, 0, 0]
        # 3. -90 deg yaw: q = [-0.707, 0.707, 0, 0]
        # 4. 180 deg yaw: q = [1, 0, 0, 0] (Wait, 180 yaw + 180 pitch = ?)
        
        # Let's use scipy or just hardcode common ones for "pointing down"
        # Pointing down means local Z is -World Z.
        # Standard: X forward, Y left, Z up.
        # Down: X forward, Y right, Z down (Roll 180) -> q=[1, 0, 0, 0] (x,y,z,w)
        # Down: X backward, Y left, Z down (Pitch 180) -> q=[0, 1, 0, 0]
        
        orientations = [
            # x, y, z, w
            [0.0, 1.0, 0.0, 0.0],       # Pitch 180 (Standard "down")
            [1.0, 0.0, 0.0, 0.0],       # Roll 180
            [0.707, 0.707, 0.0, 0.0],   # Pitch 180 + Yaw 90
            [-0.707, 0.707, 0.0, 0.0],  # Pitch 180 - Yaw 90
        ]
        
        # If target_pose is already a Pose object with orientation, try that first/only?
        # The user code passes [x,y,z] list mostly.
        
        target_point = None
        if isinstance(target_pose, list) or isinstance(target_pose, tuple):
            target_point = Point(x=float(target_pose[0]), y=float(target_pose[1]), z=float(target_pose[2]))
        else:
            # If it's a Pose, extract position and try our orientations
            # Or should we respect the input orientation?
            # For this specific pick-and-place task, we want "down", so we override.
            target_point = target_pose.position

        for i, q_list in enumerate(orientations):
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.group_name
            req.ik_request.pose_stamped.header.frame_id = self.base_frame
            req.ik_request.timeout.sec = 0
            req.ik_request.timeout.nanosec = int(0.1 * 1e9) # Fast timeout for retries
            req.ik_request.avoid_collisions = avoid_collisions
            
            pose = Pose()
            pose.position = target_point
            pose.orientation.x = q_list[0]
            pose.orientation.y = q_list[1]
            pose.orientation.z = q_list[2]
            pose.orientation.w = q_list[3]
            
            req.ik_request.pose_stamped.pose = pose
            
            if current_joint_state:
                req.ik_request.robot_state.joint_state = current_joint_state
                
            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.result() is not None:
                response = future.result()
                if response.error_code.val == response.error_code.SUCCESS:
                    # self.node.get_logger().info(f"IK found with orientation {i}")
                    return response.solution.joint_state
            
        self.node.get_logger().warn(f"IK failed for all {len(orientations)} orientations at {target_point}")
        return None

    def get_ik_for_box(self, box_pos, approach_offset=0.1):
        """
        Get IK solution for picking a box
        
        Args:
            box_pos: [x, y, z] of box center
            approach_offset: distance above box to approach
            
        Returns:
            (approach_joints, pick_joints) tuple of joint states
        """
        # Pick pose (at box height)
        # Adjust z to grab the box (box height is 0.06, center is 0.03)
        # Gripper frame is usually at the tool tip.
        # We want tool tip to be slightly above box center to grip it.
        pick_z = box_pos[2] + 0.12  # Gripper length offset
        
        pick_pose = [box_pos[0], box_pos[1], pick_z]
        
        # Approach pose (above box)
        approach_pose = [box_pos[0], box_pos[1], pick_z + approach_offset]
        
        # Calculate IK
        approach_sol = self.get_ik(approach_pose)
        if not approach_sol:
            return None, None
            
        # Use approach solution as seed for pick solution
        pick_sol = self.get_ik(pick_pose, current_joint_state=approach_sol)
        
        return approach_sol, pick_sol
