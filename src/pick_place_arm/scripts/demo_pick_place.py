#!/usr/bin/env python3
"""
Integrated Pick and Place Demo
-------------------------------
All-in-one demo with built-in attachment logic.

Sequence:
1. Send initial detach commands (clear pre-attachment)
2. Close gripper → Monitor for stall → Attach
3. Move to place_1 position
4. Open gripper → Detach

Usage (2 terminals only!):
  Terminal 1: ros2 launch arm_moveit_config unified_gz_moveit.launch.py
  Terminal 2: python3 install/pick_place_arm/lib/pick_place_arm/demo_pick_place.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
import time
from enum import Enum
import threading


class GripperState(Enum):
    OPEN = 0
    CLOSING = 1
    GRASPING = 2
    ATTACHED = 3


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
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
        )

        # Subscriber for joint states (for attachment logic)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # Gripper state tracking
        self.gripper_state = GripperState.OPEN
        self.last_gripper_pos = 0.0
        
        # Attachment parameters
        self.stall_min = -0.02
        self.stall_max = -0.003
        self.open_thresh = -0.002

        # Positions
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.place_1_position = [0.0, 1.57, 0.0, -1.57, 0.0, 0.0]

        self.get_logger().info("🤖 Integrated Pick and Place Demo Starting...")
        self.get_logger().info("Waiting for action servers...")
        
        # Wait for servers
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        self.get_logger().info("✓ Action servers ready!")
        
        # Start spinning in background thread to process callbacks
        self.executor_thread = threading.Thread(target=self.spin_thread, daemon=True)
        self.executor_thread.start()
        
        # Note: Initial detach now handled by launch file
        self.get_logger().info("Waiting for system to stabilize...")
        time.sleep(2.0)
        
        # Execute demo
        self.run_demo()

    def spin_thread(self):
        """Background thread to process callbacks"""
        rclpy.spin(self)

    def joint_callback(self, msg):
        """Monitor gripper joint for attachment logic"""
        try:
            idx = msg.name.index('j7l')
            pos = msg.position[idx]
        except (ValueError, IndexError):
            return
        
        # Determine physical state
        is_open = pos > self.open_thresh
        is_stalled = self.stall_min < pos < self.stall_max
        
        # Debug: Log position periodically
        if abs(pos - self.last_gripper_pos) > 0.001:  # Position changed
            self.get_logger().info(
                f"  Gripper pos: {pos:.4f} | Open: {is_open} | Stalled: {is_stalled} | State: {self.gripper_state.name}"
            )
        
        old_state = self.gripper_state
        
        if is_open:
            if self.gripper_state == GripperState.ATTACHED:
                self.detach()
            self.gripper_state = GripperState.OPEN
            
        elif is_stalled:
            if self.gripper_state != GripperState.ATTACHED:
                self.gripper_state = GripperState.GRASPING
                self.grasp(pos)
        
        if old_state != self.gripper_state:
            self.get_logger().info(f"  Gripper: {old_state.name} → {self.gripper_state.name}")
        
        self.last_gripper_pos = pos

    def grasp(self, current_pos):
        """Execute grasp: stop gripper + attach"""
        self.get_logger().info(f"  🎯 Stall detected at {current_pos:.4f} → ATTACHING")
        
        # Stop gripper at current position
        msg = JointTrajectory()
        msg.joint_names = ['j7l', 'j7r']
        point = JointTrajectoryPoint()
        point.positions = [current_pos, -current_pos]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        msg.points = [point]
        self.traj_pub.publish(msg)
        
        # Attach
        self.attach_pub.publish(Empty())
        self.gripper_state = GripperState.ATTACHED
        self.get_logger().info("  ✅ Box ATTACHED to gripper")

    def detach(self):
        """Detach object"""
        self.get_logger().info("  🔓 Gripper opened → DETACHING")
        self.detach_pub.publish(Empty())
        self.gripper_state = GripperState.OPEN
        self.get_logger().info("  ✅ Box RELEASED")

    def move_arm(self, positions, duration_sec=3.0, description="position"):
        """Move arm to joint positions."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f"  Moving arm to {description}")
        self.arm_client.send_goal_async(goal_msg)

    def open_gripper(self):
        """Open gripper."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info("  Opening gripper...")
        self.gripper_client.send_goal_async(goal_msg)

    def close_gripper(self):
        """Close gripper (will trigger attachment on stall)"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [-0.032, 0.032]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info("  Closing gripper...")
        self.gripper_client.send_goal_async(goal_msg)

    def run_demo(self):
        """Execute the demo sequence."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🚀 INTEGRATED PICK AND PLACE DEMO")
        self.get_logger().info("="*60)
        
        # Step 1: Close gripper (PICK)
        self.get_logger().info("\n[1/4] 📦 PICKING - Closing gripper on box")
        self.close_gripper()
        self.get_logger().info("  Waiting for stall detection and attachment...")
        time.sleep(3.0)
        
        if self.gripper_state == GripperState.ATTACHED:
            self.get_logger().info("  ✅ Box successfully attached!")
        else:
            self.get_logger().warn("  ⚠️  Attachment may have failed - check gripper position")
        
        # Step 2: Move to place_1
        self.get_logger().info("\n[2/4] 🚚 TRANSPORTING - Moving to place_1 with box")
        self.move_arm(self.place_1_position, 5.0, "place_1")
        self.get_logger().info("  Waiting for movement to complete...")
        time.sleep(10.0)
        
        # Step 3: Open gripper (RELEASE)
        self.get_logger().info("\n[3/4] 📤 RELEASING - Opening gripper")
        self.open_gripper()
        self.get_logger().info("  Waiting for detachment...")
        time.sleep(2.0)
        
        if self.gripper_state == GripperState.OPEN:
            self.get_logger().info("  ✅ Box successfully released!")
        
        # Done
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("✅ DEMO COMPLETE!")
        self.get_logger().info("="*60 + "\n")
        
        self.get_logger().info("Shutting down...")
        
        # Give time for final messages to process
        time.sleep(0.5)
        
        # Shutdown ROS (executor thread will stop automatically as daemon)
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore shutdown errors


def main():
    rclpy.init()
    node = DemoPickPlace()
    # Note: Spinning happens in background thread
    # Demo runs in main thread and shuts down when complete


if __name__ == "__main__":
    main()
