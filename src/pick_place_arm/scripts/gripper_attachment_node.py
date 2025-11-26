#!/usr/bin/env python3
"""
Enhanced Gripper Attachment Node
---------------------------------
Monitors gripper state and manages dynamic attachment/detachment.

States:
  OPEN      -> Gripper fully open (pos > -0.002)
  CLOSING   -> Gripper commanded to close, moving
  GRASPING  -> Gripper stalled on object (-0.02 < pos < -0.003)
  ATTACHED  -> Object attached to gripper

Transitions:
  OPEN -> CLOSING: Close command received
  CLOSING -> GRASPING: Stall detected
  GRASPING -> ATTACHED: Attach command sent
  ATTACHED -> OPEN: Gripper opens
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from enum import Enum

class GripperState(Enum):
    OPEN = 0
    CLOSING = 1
    GRASPING = 2
    ATTACHED = 3

class GripperAttachmentNode(Node):
    def __init__(self):
        super().__init__('gripper_attachment_node')
        
        # Declare parameters
        self.declare_parameter('stall_threshold_min', -0.02)
        self.declare_parameter('stall_threshold_max', -0.003)
        self.declare_parameter('open_threshold', -0.002)
        self.declare_parameter('detach_attempts', 10)
        self.declare_parameter('gripper_joint', 'j7l')
        
        # Get parameters
        self.stall_min = self.get_parameter('stall_threshold_min').value
        self.stall_max = self.get_parameter('stall_threshold_max').value
        self.open_thresh = self.get_parameter('open_threshold').value
        self.detach_attempts_max = self.get_parameter('detach_attempts').value
        self.gripper_joint_name = self.get_parameter('gripper_joint').value
        
        # Publishers
        self.attach_pub = self.create_publisher(Empty, '/box1/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/box1/detach', 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
        )
        self.state_pub = self.create_publisher(String, '/gripper_state', 10)
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        
        # State
        self.state = GripperState.OPEN
        self.last_position = 0.0
        
        # Initial detach loop
        self.detach_attempts = 0
        self.detach_timer = self.create_timer(0.5, self.initial_detach_loop)
        
        self.get_logger().info("Enhanced Gripper Attachment Node Started")
        self.get_logger().info(f"Thresholds: stall=[{self.stall_min}, {self.stall_max}], open={self.open_thresh}")

    def initial_detach_loop(self):
        """Send detach commands on startup to clear any pre-attachment"""
        if self.detach_attempts < self.detach_attempts_max:
            self.detach_pub.publish(Empty())
            self.get_logger().info(
                f"Initial DETACH ({self.detach_attempts+1}/{self.detach_attempts_max})"
            )
            self.detach_attempts += 1
        else:
            self.detach_timer.cancel()
            self.get_logger().info("Initial detach sequence complete")

    def joint_callback(self, msg):
        """Monitor gripper joint and manage state transitions"""
        try:
            idx = msg.name.index(self.gripper_joint_name)
            pos = msg.position[idx]
        except (ValueError, IndexError):
            return
        
        # Determine current physical state
        is_open = pos > self.open_thresh
        is_stalled = self.stall_min < pos < self.stall_max
        
        # State machine
        old_state = self.state
        
        if is_open:
            # Gripper is OPEN
            if self.state == GripperState.ATTACHED:
                self.detach()
            self.state = GripperState.OPEN
            
        elif is_stalled:
            # Gripper is GRASPING (stalled on object)
            if self.state != GripperState.ATTACHED:
                self.state = GripperState.GRASPING
                self.grasp(pos)
        
        # Publish state for diagnostics
        if old_state != self.state:
            state_msg = String()
            state_msg.data = self.state.name
            self.state_pub.publish(state_msg)
            self.get_logger().info(f"State: {old_state.name} -> {self.state.name}")
        
        self.last_position = pos

    def grasp(self, current_pos):
        """Execute grasp sequence: stop gripper, then attach"""
        self.get_logger().info(
            f"Grasp detected (stall at {current_pos:.4f}) -> STOPPING & ATTACHING"
        )
        
        # 1. STOP gripper at current position
        self.hold_gripper(current_pos)
        
        # 2. ATTACH object
        self.attach_pub.publish(Empty())
        self.state = GripperState.ATTACHED

    def hold_gripper(self, current_pos):
        """Send trajectory to hold gripper at current position"""
        msg = JointTrajectory()
        msg.joint_names = ['j7l', 'j7r']
        
        point = JointTrajectoryPoint()
        point.positions = [current_pos, -current_pos]  # Symmetric
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s
        
        msg.points = [point]
        self.traj_pub.publish(msg)
        self.get_logger().debug(f"Sent HOLD command at {current_pos:.4f}")

    def detach(self):
        """Detach object from gripper"""
        self.get_logger().info("Gripper opened -> DETACHING")
        self.detach_pub.publish(Empty())
        self.state = GripperState.OPEN

def main():
    rclpy.init()
    node = GripperAttachmentNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
