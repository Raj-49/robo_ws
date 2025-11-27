#!/usr/bin/env python3
"""
Pick and Place Demo Script
--------------------------
Demonstrates hardcoded pick and place operations using recorded positions.

Options:
1. Pick and Place RED box
2. Pick and Place GREEN box
3. Pick and Place BLUE box
4. Pick and Place ALL (RGB sequence)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
import time

class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')
        
        # Action client for arm control
        self.arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
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
        
        self.get_logger().info("Pick and Place Demo Ready!")
        
    def move_arm(self, positions, duration=4.0):
        """Move arm to specified joint positions"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points = [point]
        
        self.get_logger().info(f"Moving arm to position: {[f'{p:.3f}' for p in positions]}")
        
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
    
    def attach_box(self, color):
        """Attach specified box"""
        self.get_logger().info(f"Attaching {color.upper()} box...")
        self.attach_pubs[color].publish(Empty())
        time.sleep(0.5)
    
    def detach_box(self, color):
        """Detach specified box"""
        self.get_logger().info(f"Detaching {color.upper()} box...")
        self.detach_pubs[color].publish(Empty())
        time.sleep(0.5)
    
    def pick_and_place(self, color):
        """Execute complete pick and place sequence for specified color"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🤖 PICK AND PLACE: {color.upper()} BOX")
        self.get_logger().info(f"{'='*60}\n")
        
        # Step 1: Move to pick position
        self.get_logger().info(f"📍 Step 1: Moving to {color.upper()} pick position")
        if not self.move_arm(self.pick_positions[color], duration=4.0):
            return False
        time.sleep(1.0)
        
        # Step 2: Attach box
        self.get_logger().info(f"🔗 Step 2: Attaching {color.upper()} box")
        self.attach_box(color)
        time.sleep(1.0)
        
        # Step 3: Move to place position
        self.get_logger().info(f"📍 Step 3: Moving to {color.upper()} basket")
        if not self.move_arm(self.place_positions[color], duration=4.0):
            return False
        time.sleep(1.0)
        
        # Step 4: Detach box
        self.get_logger().info(f"🔓 Step 4: Detaching {color.upper()} box")
        self.detach_box(color)
        time.sleep(1.0)
        
        # Step 5: Return home
        self.get_logger().info(f"🏠 Step 5: Returning to home position")
        if not self.move_arm(self.home_position, duration=3.0):
            return False
        time.sleep(1.0)
        
        self.get_logger().info(f"✅ {color.upper()} pick and place COMPLETE!\n")
        return True
    
    def pick_and_place_all(self):
        """Execute pick and place for all boxes in RGB sequence"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🌈 PICK AND PLACE ALL - RGB SEQUENCE")
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
    node = PickAndPlaceDemo()
    
    # Wait for action server
    node.get_logger().info("Waiting for arm controller...")
    node.arm_client.wait_for_server()
    node.get_logger().info("Connected to arm controller!\n")
    
    while rclpy.ok():
        print("\n" + "="*60)
        print("🤖 PICK AND PLACE DEMO")
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
