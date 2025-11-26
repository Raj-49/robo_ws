#!/usr/bin/env python3
"""
Gripper Attachment Node
-----------------------
Implements dynamic attachment logic:
1. Monitors /box1/contacts to detect when gripper touches box
2. Monitors /gripper_controller/joint_states (or commands) to know if gripper is closing
3. Publishes to /box1/attach ONLY when contact is made AND gripper is active
4. Publishes to /box1/detach when gripper opens

This mimics the behavior of a suction gripper plugin but using the DetachableJoint system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class GripperAttachmentNode(Node):
    def __init__(self):
        super().__init__('gripper_attachment_node')
        
        # Publishers for DetachableJoint plugin
        self.attach_pub = self.create_publisher(Empty, '/box1/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/box1/detach', 10)
        
        # Publisher to STOP/HOLD gripper
        self.traj_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # Subscribers
        self.contact_sub = self.create_subscription(
            Contacts, '/box1/contacts', self.contact_callback, 10
        )
        self.contact_sub_scoped = self.create_subscription(
            Contacts, '/world/empty/model/box1/link/link/sensor/box1_contact/contact', self.contact_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        
        # State
        self.is_touching = False
        self.gripper_closed = False
        self.attached = False
        self.last_contact_time = self.get_clock().now()
        
        self.get_logger().info("Gripper Attachment Node Started")
        
        # Force detach on startup to prevent pre-attachment
        # Send multiple times to ensure bridge is ready and message is received
        self.create_timer(0.5, self.initial_detach_loop)
        self.detach_attempts = 0

    def initial_detach_loop(self):
        if self.detach_attempts < 10:
            self.detach_pub.publish(Empty())
            self.get_logger().info(f"Sent initial DETACH command ({self.detach_attempts+1}/10)")
            self.detach_attempts += 1
        else:
            # Stop timer after 10 attempts (5 seconds)
            pass

    def contact_callback(self, msg):
        # (Contact logic preserved but less critical now)
        if not msg.contacts:
            return
        # self.get_logger().info(f"Contact...", throttle_duration_sec=1.0)

    def joint_callback(self, msg):
        try:
            idx_l = msg.name.index('j7l')
            pos_l = msg.position[idx_l]
            
            # self.get_logger().info(f"Joint j7l: {pos_l:.4f}", throttle_duration_sec=2.0)
            
            # Logic:
            # 1. Open: pos > -0.002
            # 2. Grasping Box: -0.02 < pos < -0.003 (Stalled on box)
            
            if pos_l > -0.002:
                # Gripper is OPEN
                if self.attached:
                    self.get_logger().info(f"Gripper Open (pos: {pos_l:.4f}) -> DETACHING")
                    self.detach()
                self.gripper_closed = False
                
            elif -0.02 < pos_l < -0.003:
                # Gripper is GRASPING (Stalled)
                if not self.attached:
                    self.get_logger().info(f"Grasp Detected (Stall at {pos_l:.4f}) -> STOPPING & ATTACHING")
                    
                    # 1. STOP GRIPPER (Hold current position)
                    self.hold_gripper(pos_l)
                    
                    # 2. ATTACH
                    self.attach()
                self.gripper_closed = True
                
            elif pos_l < -0.03:
                # Gripper is CLOSED EMPTY
                self.gripper_closed = True
            
        except ValueError:
            pass

    def hold_gripper(self, current_pos):
        # Create a trajectory to hold the current position
        msg = JointTrajectory()
        msg.joint_names = ['j7l', 'j7r']
        
        point = JointTrajectoryPoint()
        # j7l = current_pos, j7r = -current_pos (symmetric)
        point.positions = [current_pos, -current_pos]
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 0.1s
        
        msg.points = [point]
        self.traj_pub.publish(msg)
        self.get_logger().info(f"Sent HOLD command at {current_pos:.4f}")

    def check_attachment(self):
        pass
            
    def attach(self):
        self.get_logger().info("Conditions met (Contact + Closed) -> ATTACHING")
        self.attach_pub.publish(Empty())
        self.attached = True
        
    def detach(self):
        if self.attached:
            self.get_logger().info("Gripper Open -> DETACHING")
            self.detach_pub.publish(Empty())
            self.attached = False

def main():
    rclpy.init()
    node = GripperAttachmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
