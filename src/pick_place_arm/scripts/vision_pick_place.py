#!/usr/bin/env python3
"""
Vision-Based Pick and Place System
-----------------------------------
Detects colored boxes via camera, determines which table they're on,
and executes pick-and-place operations based on user-specified sequence.

Usage:
    python3 vision_pick_place.py
    
    Enter sequence like: [red] or [green, blue] or [red, green, blue]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
from sensor_msgs.msg import Image, JointState
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from enum import Enum
import cv2
import time
import threading
import numpy as np
import sys
import os

# Add scripts directory to path
sys.path.append(os.path.dirname(__file__))
from object_detector import ColorObjectDetector

class GripperState(Enum):
    OPEN = 0
    CLOSING = 1
    ATTACHED = 2

class VisionPickPlace(Node):
    def __init__(self):
        super().__init__('vision_pick_place')
        
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
        
        # Publishers for box attachment
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
        
        # State tracking
        self.current_attached_box = None
        self.gripper_state = GripperState.OPEN
        
        # Initial box positions (recorded on first detection)
        self.initial_box_positions = {}  # {color: (x, y, table_num)}
        self.latest_box_positions = {}   # Shared state for visualization thread
        self.position_tolerance = 5.0  # cm tolerance for position validation
        
        # Camera
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.detector = ColorObjectDetector()
        # Use Best Effort QoS for camera to ensure we get images
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        
        # Hardcoded positions from SRDF
        self.positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'table1_pick': [0.9590333013756456, 0.3939240112089198, 0.479481411325086,
                           0.23403999842028253, 0.45097861722136046, 5.363057321014417e-05],
            'table2_pick': [9.990135855041453e-05, 0.44531366847265, 0.4509790868877058,
                           0.3652866062418217, 0.4110300069085086, -8.548499507841293e-07],
            'table3_pick': [-0.947632737185824, 0.2682407096016315, 0.5651149470764493,
                           0.6793300068077888, 0.1713292871339623, -9.324230798529365e-05],
            'blue_basket': [-1.908633, -0.285813, 0.672824, 0.187192, 0.450882, 0.000170],
            'red_basket': [-2.436366, 0.065933, 0.303389, 0.197981, 0.411065, -0.000045],
            'green_basket': [-1.362681, -0.156609, 0.418936, 0.427446, 0.171263, -0.000047]
        }
        
        self.get_logger().info("Vision Pick and Place System Initialized")
        
        # Wait for servers
        self.get_logger().info("Waiting for action servers...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info("✓ Action servers ready!")
        
        # Start background thread for ROS callbacks
        self.executor_thread = threading.Thread(target=self.spin_thread, daemon=True)
        self.executor_thread.start()
        
        # Start visualization thread
        self.vis_thread = threading.Thread(target=self.visualization_loop, daemon=True)
        self.vis_thread.start()
        
        # Wait for camera
        self.get_logger().info("Waiting for camera...")
        self.wait_for_camera()
        
    def spin_thread(self):
        """Background thread to process callbacks"""
        rclpy.spin(self)
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        if self.latest_rgb is None:
            self.get_logger().info(f"📸 Received first image! Size: {msg.width}x{msg.height}")
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
    
    def wait_for_camera(self, timeout=10.0):
        """Wait for camera images"""
        start = time.time()
        while self.latest_rgb is None:
            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for camera!")
                return False
            time.sleep(0.1)
        self.get_logger().info("✓ Camera ready!")
        return True
    
    def detect_box_positions(self):
        """
        Return the latest detected box positions from the visualization loop.
        """
        if not self.latest_box_positions:
            self.get_logger().warn("No boxes detected yet!")
            return {}
        return self.latest_box_positions

    def visualization_loop(self):
        """
        Continuous loop to process images, detect boxes, and publish visualization.
        Runs in a background thread.
        """

        self.get_logger().info("Visualization loop started")
        while rclpy.ok():
            if self.latest_rgb is None:
                time.sleep(0.1)
                continue

            try:
                # Get image dimensions
                img_width = self.latest_rgb.shape[1]
                img_height = self.latest_rgb.shape[0]
                left_threshold = img_width / 3
                right_threshold = 2 * img_width / 3
                
                # Define table ROI (bottom 60% of image, baskets in top 40%)
                table_roi_top = int(img_height * 0.4)
                
                # Create visualization image
                vis_img = self.latest_rgb.copy()
                
                # Draw ROI boundary only (shows table zone starts here)
                cv2.line(vis_img, (0, table_roi_top), (img_width, table_roi_top), (0, 0, 0), 1)
                cv2.putText(vis_img, "Table Zone", (10, table_roi_top + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                # Detect boxes
                detections = self.detector.detect_all(self.latest_rgb)
                current_positions = {}
                
                # Approximate conversion: 640px width ~= 100cm workspace -> ~6.4 px/cm
                pixels_per_cm = 6.4 
                threshold_px = self.position_tolerance * pixels_per_cm
                
                for color, detection in detections.items():
                    if detection is not None:
                        x, y = detection['center']
                        
                        # SPATIAL FILTER: Ignore detections in basket zone (top 40%)
                        if y < table_roi_top:
                            continue  # Skip baskets in upper part of image
                        
                        # Draw detection (bounding box and center)
                        vis_img = self.detector.visualize(vis_img, detection, color)
                        
                        # Record initial position if not set
                        if color not in self.initial_box_positions:
                            self.initial_box_positions[color] = (x, y)
                        
                        # Validate position
                        init_x, init_y = self.initial_box_positions[color]
                        dist = np.sqrt((x - init_x)**2 + (y - init_y)**2)
                        
                        # Draw initial position marker (visual only, no text)
                        cv2.circle(vis_img, (int(init_x), int(init_y)), 5, (0, 255, 255), -1)
                        
                        if dist > threshold_px:
                            # Skip if moved too much (no text warning)
                            continue 
                        
                        # Determine table based on X position
                        if x < left_threshold:
                            table = 1  # Left = Table 1
                        elif x < right_threshold:
                            table = 2  # Middle = Table 2
                        else:
                            table = 3  # Right = Table 3
                        
                        current_positions[color] = table
                        
                        # Add table number next to the position label (above bbox)
                        bbox_x, bbox_y, bbox_w, bbox_h = detection['bbox']
                        cv2.putText(vis_img, f"T{table}", (bbox_x + bbox_w + 5, bbox_y), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                
                # Update shared state
                self.latest_box_positions = current_positions
                
                # Publish annotated image
                msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
                self.image_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f"Visualization loop error: {e}")
                pass
                
            time.sleep(0.1)  # 10Hz
    
    def move_arm(self, positions, duration=4.0):
        """Move arm to specified joint positions"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points = [point]
        
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
                return True
        return False
    
    def close_gripper_and_attach(self, color):
        """Close gripper minimally and attach box"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [-0.010, 0.010]  # Minimal close
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]
        
        self.get_logger().info("Closing gripper...")
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=4.0)
                time.sleep(0.8)
                
                # Detach all boxes first
                for box_color in ['red', 'green', 'blue']:
                    self.detach_pubs[box_color].publish(Empty())
                time.sleep(0.3)
                
                # Attach specified box
                self.attach_pubs[color].publish(Empty())
                self.current_attached_box = color
                self.gripper_state = GripperState.ATTACHED
                self.get_logger().info(f"✅ {color.upper()} box attached!")
                return True
        return False
    
    def open_gripper_and_detach(self):
        """Detach box and open gripper"""
        if self.current_attached_box:
            self.detach_pubs[self.current_attached_box].publish(Empty())
            self.get_logger().info(f"🔓 Detaching {self.current_attached_box.upper()} box...")
            self.current_attached_box = None
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [0.035, -0.035]  # Open
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
    
    def pick_and_place_box(self, color, table_num):
        """
        Pick a box from specified table and place in matching colored basket
        
        Args:
            color: 'red', 'green', or 'blue'
            table_num: 1, 2, or 3
        """
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🎯 PICKING {color.upper()} BOX FROM TABLE {table_num}")
        self.get_logger().info(f"{'='*60}")
        
        # Step 1: Move to pick position
        pick_key = f'table{table_num}_pick'
        self.get_logger().info(f"📍 Moving to {pick_key}...")
        if not self.move_arm(self.positions[pick_key], duration=4.0):
            self.get_logger().error("Failed to move to pick position")
            return False
        time.sleep(1.5)
        
        # Step 2: Close gripper and attach
        self.get_logger().info(f"🤏 Picking {color} box...")
        if not self.close_gripper_and_attach(color):
            self.get_logger().error("Failed to attach box")
            return False
        time.sleep(1.0)
        
        # Step 3: Move to basket
        basket_key = f'{color}_basket'
        self.get_logger().info(f"📍 Moving to {basket_key}...")
        if not self.move_arm(self.positions[basket_key], duration=4.0):
            self.get_logger().error("Failed to move to basket")
            return False
        time.sleep(1.0)
        
        # Step 4: Detach and open gripper
        self.get_logger().info(f"🔓 Placing {color} box...")
        self.open_gripper_and_detach()
        time.sleep(1.0)
        
        # Step 5: Return home
        self.get_logger().info("🏠 Returning home...")
        if not self.move_arm(self.positions['home'], duration=3.0):
            self.get_logger().error("Failed to return home")
            return False
        time.sleep(1.0)
        
        self.get_logger().info(f"✅ {color.upper()} box placed successfully!\n")
        return True
    
    def execute_sequence(self, color_sequence):
        """
        Execute pick-and-place for a sequence of colors
        
        Args:
            color_sequence: list of colors like ['red'] or ['green', 'blue']
        """
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🚀 STARTING SEQUENCE: {color_sequence}")
        self.get_logger().info(f"{'='*60}\n")
        
        # Detect all box positions
        self.get_logger().info("📷 Detecting box positions...")
        box_positions = self.detect_box_positions()
        
        if not box_positions:
            self.get_logger().error("No boxes detected!")
            return False
        
        # Verify all requested colors are detected
        for color in color_sequence:
            if color not in box_positions:
                self.get_logger().error(f"❌ {color.upper()} box not detected!")
                return False
        
        self.get_logger().info(f"✓ All boxes detected: {box_positions}\n")
        
        # Execute pick-and-place for each color in sequence
        for i, color in enumerate(color_sequence, 1):
            table_num = box_positions[color]
            self.get_logger().info(f"[{i}/{len(color_sequence)}] Processing {color.upper()} box...")
            
            if not self.pick_and_place_box(color, table_num):
                self.get_logger().error(f"Failed to process {color} box!")
                return False
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🎉 SEQUENCE COMPLETE!")
        self.get_logger().info(f"{'='*60}\n")
        return True
    
    def run(self):
        """Main interactive loop"""
        while rclpy.ok():
            print("\n" + "="*60)
            print("🤖 VISION-BASED PICK AND PLACE SYSTEM")
            print("="*60)
            print("\nEnter sequence of colors to pick:")
            print("  Examples:")
            print("    [red]")
            print("    [green, blue]")
            print("    [red, green, blue]")
            print("\n  0. Exit")
            print("="*60)
            
            user_input = input("\nEnter sequence: ").strip()
            
            if user_input == '0':
                break
            
            # Parse input
            try:
                # Remove brackets and split by comma
                user_input = user_input.strip('[]')
                colors = [c.strip().lower() for c in user_input.split(',')]
                
                # Validate colors
                valid_colors = ['red', 'green', 'blue']
                for color in colors:
                    if color not in valid_colors:
                        print(f"❌ Invalid color: {color}")
                        continue
                
                # Execute sequence
                self.execute_sequence(colors)
                
            except Exception as e:
                print(f"❌ Error parsing input: {e}")
                print("Please use format: [red] or [green, blue] or [red, green, blue]")

def main():
    rclpy.init()
    node = VisionPickPlace()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
