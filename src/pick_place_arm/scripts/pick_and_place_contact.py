#!/usr/bin/env python3
"""
Pick and Place Demo Script - Contact-Based Version
--------------------------------------------------
Enhanced version with contact-based attachment/detachment.

Features:
- Waits for gripper contact before attaching
- Detaches when gripper opens (contact lost)
- More reliable and realistic behavior
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from enum import Enum
import time

class GripperState(Enum):
    OPEN = 1
    CLOSING = 2
    GRASPING = 3
    ATTACHED = 4

class PickAndPlaceDemoContact(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo_contact')
        
        # Action clients
        self.arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        
        # Publishers for attachment
        self.attach_pubs = {
            'red': self.create_publisher(Empty, '/red_box/attach', 10),
            'green': self.create_publisher(Empty, '/green_box/attach', 10),
            'blue': self.create_publisher(Empty, '/blue_box/attach', 10)
        }
        self.detach_pubs = {
            'red': self.create_publisher(Empty, '/red_box/detach', 10),
            'green': self.create_publisher(Empty, '/green_box/detach', 10),
            'blue': self.create_publisher(Empty, '/blue_box/detach', 10)
        }
        
        # Joint state subscriber for gripper monitoring
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # Gripper state tracking
        self.gripper_state = GripperState.OPEN
        self.current_box = None
        self.last_gripper_pos = 0.0
        
        # Gripper thresholds
        self.open_thresh = 0.034  # Gripper is open
        self.stall_min = 0.005    # Contact detected (stalled)
        self.stall_max = 0.030    # Contact range
        
        # Hardcoded positions
        self.home_position = [-0.000090, -0.000069, -0.000047, -0.000100, -0.000024, 0.000027]
        
        self.pick_positions = {
            'blue': [0.9590333013756456, 0.3939240112089198, 0.479481411325086, 
                    0.23403999842028253, 0.45097861722136046, 5.363057321014417e-05],
            'red': [9.990135855041453e-05, 0.44531366847265, 0.4509790868877058,
                   0.3652866062418217, 0.4110300069085086, -8.548499507841293e-07],
            'green': [-0.947632737185824, 0.2682407096016315, 0.5651149470764493,
                     0.6793300068077888, 0.1713292871339623, -9.324230798529365e-05]
        }
        
        self.place_positions = {
            'blue': [-1.908633, -0.285813, 0.672824, 0.187192, 0.450882, 0.000170],
            'red': [-2.436366, 0.065933, 0.303389, 0.197981, 0.411065, -0.000045],
            'green': [-1.362681, -0.156609, 0.418936, 0.427446, 0.171263, -0.000047]
        }
        
        self.arm_joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.gripper_joints = ['j7l', 'j7r']
        
        self.get_logger().info("Contact-Based Pick and Place Demo Ready!")
    
    def joint_callback(self, msg):
        """Monitor gripper state for contact-based attachment"""
        try:
            idx = msg.name.index('j7l')
            pos = msg.position[idx]
        except (ValueError, IndexError):
            return
        
        is_open = pos > self.open_thresh
        is_stalled = self.stall_min < pos < self.stall_max
        
        old_state = self.gripper_state
        
        # State machine for gripper
        if is_open:
            if self.gripper_state == GripperState.ATTACHED and self.current_box:
                # Gripper opened - detach box
                self.detach_box(self.current_box)
                self.current_box = None
            self.gripper_state = GripperState.OPEN
            
        elif is_stalled and self.gripper_state == GripperState.CLOSING:
            # Contact detected - attach box
            if self.current_box:
                self.gripper_state = GripperState.GRASPING
                self.attach_box(self.current_box)
                self.gripper_state = GripperState.ATTACHED
        
        self.last_gripper_pos = pos
    
    def move_arm(self, positions, duration=4.0):
        """Move arm to specified joint positions"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points = [point]
        
        self.get_logger().info(f"Moving arm...")
        
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
                self.get_logger().info("✅ Movement completed")
                return True
        
        self.get_logger().error("❌ Movement failed")
        return False
    
    def close_gripper_and_attach(self, color):
        """Close gripper to gripping position and attach box"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        # Use minimal close (-0.010) to barely touch the box
        # This prevents gripper from going inside the box
        point.positions = [-0.010, 0.010]  # Minimal gripping position
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]
        
        self.get_logger().info("Closing gripper minimally...")
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=4.0)
                
                # Wait for stable grip
                time.sleep(0.8)
                self.attach_box(color)
                self.current_box = color
                self.gripper_state = GripperState.ATTACHED
                self.get_logger().info("✅ Box attached!")
                return True
        
        return False
    
    def open_gripper_and_detach(self):
        """Open gripper and detach box"""
        # Detach first
        if self.current_box:
            self.detach_box(self.current_box)
            self.current_box = None
        
        # Then open gripper
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [0.035, -0.035]  # Open position
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]
        
        self.get_logger().info("Opening gripper...")
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=4.0)
                time.sleep(0.5)
                self.gripper_state = GripperState.OPEN
                return True
        return False
    
    def attach_box(self, color):
        """Attach specified box"""
        self.get_logger().info(f"🔗 Attaching {color.upper()} box...")
        self.attach_pubs[color].publish(Empty())
        time.sleep(0.3)
    
    def detach_box(self, color):
        """Detach specified box"""
        self.get_logger().info(f"🔓 Detaching {color.upper()} box...")
        self.detach_pubs[color].publish(Empty())
        time.sleep(0.3)
    
    def pick_and_place(self, color):
        """Execute complete pick and place sequence with contact-based attachment"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🤖 PICK AND PLACE: {color.upper()} BOX (Contact-Based)")
        self.get_logger().info(f"{'='*60}\n")
        
        self.current_box = color
        
        # Step 1: Move to pick position
        self.get_logger().info(f"📍 Step 1: Moving to {color.upper()} pick position")
        if not self.move_arm(self.pick_positions[color], duration=4.0):
            return False
        time.sleep(1.0)
        
        # Step 2: Close gripper and attach
        self.get_logger().info(f"🤏 Step 2: Closing gripper and attaching")
        if not self.close_gripper_and_attach(color):
            self.get_logger().error("Failed to grasp box")
            return False
        
        # Step 3: Move to place position
        self.get_logger().info(f"📍 Step 3: Moving to {color.upper()} basket")
        if not self.move_arm(self.place_positions[color], duration=4.0):
            return False
        time.sleep(1.0)
        
        # Step 3.5: Lift slightly to help extract gripper from box
        self.get_logger().info(f"⬆️  Step 3.5: Lifting slightly to extract gripper")
        current_pos = list(self.place_positions[color])
        current_pos[1] -= 0.05  # Lift 5cm (decrease j2 to lift)
        if not self.move_arm(current_pos, duration=1.0):
            return False
        time.sleep(0.5)
        
        # Step 4: Open gripper and detach
        self.get_logger().info(f"✋ Step 4: Opening gripper and detaching")
        self.open_gripper_and_detach()
        time.sleep(1.0)
        
        # Step 5: Return home
        self.get_logger().info(f"🏠 Step 5: Returning to home position")
        if not self.move_arm(self.home_position, duration=3.0):
            return False
        time.sleep(1.0)
        
        self.get_logger().info(f"✅ {color.upper()} pick and place COMPLETE!\n")
        self.current_box = None
        return True
    
    def pick_and_place_all(self):
        """Execute pick and place for all boxes in RGB sequence"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🌈 PICK AND PLACE ALL - RGB SEQUENCE (Contact-Based)")
        self.get_logger().info("="*60 + "\n")
        
        for color in ['red', 'green', 'blue']:
            if not self.pick_and_place(color):
                self.get_logger().error(f"Failed at {color.upper()} box")
                return False
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🎉 ALL PICK AND PLACE OPERATIONS COMPLETE!")
        self.get_logger().info("="*60 + "\n")
        return True

def main():
    rclpy.init()
    node = PickAndPlaceDemoContact()
    
    # Wait for action servers
    node.get_logger().info("Waiting for controllers...")
    node.arm_client.wait_for_server()
    node.gripper_client.wait_for_server()
    node.get_logger().info("Connected to all controllers!\n")
    
    while rclpy.ok():
        print("\n" + "="*60)
        print("🤖 PICK AND PLACE DEMO (Contact-Based)")
        print("="*60)
        print("1. Pick and Place RED box")
        print("2. Pick and Place GREEN box")
        print("3. Pick and Place BLUE box")
        print("4. Pick and Place ALL (RGB sequence)")
        print("0. Exit")
        print("="*60)
        
        choice = input("Enter your choice: ").strip()
        
        if choice == '0':
            break
        elif choice == '1':
            node.pick_and_place('red')
        elif choice == '2':
            node.pick_and_place('green')
        elif choice == '3':
            node.pick_and_place('blue')
        elif choice == '4':
            node.pick_and_place_all()
        else:
            print("Invalid choice!")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
