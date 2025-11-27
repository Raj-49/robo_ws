#!/usr/bin/env python3
"""
Interactive Vision-Based Pick and Place
----------------------------------------
User selects which colored box(es) to pick, robot detects positions
via camera and places each box on matching colored plate.

Usage:
  python3 vision_interactive_demo.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import cv2
import time
import threading
from enum import Enum
import numpy as np
import sys
import os

# Add scripts directory to path
sys.path.append(os.path.dirname(__file__))

from camera_calibration import parse_camera_sdf
from vision_utils import PixelTo3DConverter, get_depth_at_pixel
from object_detector import ColorObjectDetector
from ik_utils import IKSolver


class GripperState(Enum):
    OPEN = 0
    CLOSING = 1
    GRASPING = 2
    ATTACHED = 3


class VisionInteractiveDemo(Node):
    def __init__(self):
        super().__init__("vision_interactive_demo")

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

        # Publishers for box attachment (DetachableJoint)
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
        self.current_attached_box = None  # Track which box is currently attached
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
        )
        # Publisher for annotated image
        self.image_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        # Publisher for debug mask
        self.mask_pub = self.create_publisher(Image, '/camera/debug_mask', 10)

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
        
        # Known good configuration near the shelf (from user's RViz)
        # This helps IK find solutions by providing a good starting point
        self.shelf_seed = [0.0, 1.027, 0.976, 0.599, 0.0, 0.0]  # j1,j2,j3,j4,j5,j6 reordered
        
        # Hardcoded pick positions from SRDF (actual tested positions)
        # Table 1 (Magenta) -> Blue Box -> blue_pick
        # Table 2 (Yellow) -> Red Box -> red_pick
        # Table 3 (Cyan) -> Green Box -> green_pick
        self.pick_positions = {
            'table1': [0.9590333013756456, 0.3939240112089198, 0.479481411325086, 
                      0.23403999842028253, 0.45097861722136046, 5.363057321014417e-05],  # blue_pick
            'table2': [9.990135855041453e-05, 0.44531366847265, 0.4509790868877058,
                      0.3652866062418217, 0.4110300069085086, -8.548499507841293e-07],  # red_pick
            'table3': [-0.947632737185824, 0.2682407096016315, 0.5651149470764493,
                      0.6793300068077888, 0.1713292871339623, -9.324230798529365e-05],  # green_pick
        }
        
        # Track current target box for attachment
        self.current_target_box = None

        # Vision setup
        self.bridge = CvBridge()
        sdf_path = os.path.expanduser('~/robo_ws/src/pick_place_arm/worlds/my_world.sdf')
        cam_params = parse_camera_sdf(sdf_path, 'camera')
        self.pixel_to_3d = PixelTo3DConverter(cam_params)
        self.detector = ColorObjectDetector()
        
        # IK Solver
        self.ik_solver = IKSolver(self)
        
        # Camera subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10
        )
        
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_joint_state = None
        self.running = True

        self.get_logger().info("🤖 Vision-Based Pick and Place Demo Starting...")
        self.get_logger().info("Waiting for action servers...")
        
        # Wait for servers
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        self.get_logger().info("✓ Action servers ready!")
        
        # Start spinning in background thread to process callbacks
        self.executor_thread = threading.Thread(target=self.spin_thread, daemon=True)
        self.executor_thread.start()
        
        # Start visualization thread
        self.vis_thread = threading.Thread(target=self.visualization_loop, daemon=True)
        self.vis_thread.start()
        
        # Wait for camera images
        self.get_logger().info("Waiting for camera images...")
        self.wait_for_images()
        
        # Run interactive menu
        self.run_interactive_menu()

    def spin_thread(self):
        """Background thread to process callbacks"""
        rclpy.spin(self)
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            pass
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            pass
            
    def visualization_loop(self):
        """Show camera feed with detections and publish to ROS"""
        while self.running:
            if self.latest_rgb is not None:
                img = self.latest_rgb.copy()
                
                # Detect and draw all objects
                detections = self.detector.detect_all(img)
                for color, det in detections.items():
                    img = self.detector.visualize(img, det, color)
                    
                    # Add 3D coordinates if depth available
                    if self.latest_depth is not None:
                        u, v = det['center']
                        depth = get_depth_at_pixel(self.latest_depth, u, v)
                        if depth and depth > 0.1:
                            pos = self.pixel_to_3d.pixel_to_3d(u, v, depth)
                            text = f"[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
                            cv2.putText(img, text, (u-50, v+30), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # Publish annotated image
                try:
                    msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                    self.image_pub.publish(msg)
                    
                    # Publish debug mask (Red by default for debugging)
                    mask = self.detector.get_mask(self.latest_rgb, 'red')
                    if mask is not None:
                        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
                        self.mask_pub.publish(mask_msg)
                    
                    # Debug HSV at center (optional, enabled for diagnosis)
                    # self.detector.debug_hsv_at_center(self.latest_rgb)
                        
                except Exception as e:
                    self.get_logger().warn(f"Failed to publish image: {e}")

                # Optional: Keep local display if running with GUI
                # cv2.imshow("Robot Camera View", img)
                # cv2.waitKey(1)
                time.sleep(0.1) # 10Hz
            else:
                time.sleep(0.1)
    
    def find_nearest_reachable_pose(self, target_pos, max_offset=0.05, step=0.01):
        """
        Find the nearest reachable position to the target.
        Samples positions in a grid around the target.
        
        Args:
            target_pos: [x, y, z] target position
            max_offset: maximum distance to search (meters)
            step: step size for sampling (meters)
        
        Returns:
            Nearest reachable position or None if nothing found
        """
        self.get_logger().info(f"Searching for nearest reachable position to {target_pos}")
        
        # Try the exact position first
        if self.ik_solver.get_ik(target_pos, avoid_collisions=False) is not None:
            self.get_logger().info("Target position is reachable!")
            return target_pos
        
        # Sample positions in increasing distance from target
        best_pos = None
        best_distance = float('inf')
        
        # Create offsets to try
        offsets = []
        for dx in np.arange(-max_offset, max_offset + step, step):
            for dy in np.arange(-max_offset, max_offset + step, step):
                for dz in np.arange(-max_offset, max_offset + step, step):
                    distance = np.sqrt(dx**2 + dy**2 + dz**2)
                    if distance <= max_offset and distance > 0:
                        offsets.append((dx, dy, dz, distance))
        
        # Sort by distance
        offsets.sort(key=lambda x: x[3])
        
        # Try each offset
        for dx, dy, dz, distance in offsets:
            test_pos = [
                target_pos[0] + dx,
                target_pos[1] + dy,
                target_pos[2] + dz
            ]
            
            # Test if this position is reachable
            if self.ik_solver.get_ik(test_pos, avoid_collisions=False) is not None:
                if distance < best_distance:
                    best_pos = test_pos
                    best_distance = distance
                    self.get_logger().info(f"Found reachable position at offset [{dx:.3f}, {dy:.3f}, {dz:.3f}], distance: {distance:.3f}m")
                    return best_pos  # Return first found (closest)
        
        if best_pos:
            self.get_logger().info(f"Best reachable position: {best_pos}, offset: {best_distance:.3f}m")
        else:
            self.get_logger().warn("No reachable position found within search radius")
        
        return best_pos
        
        cv2.destroyAllWindows()
    
    def wait_for_images(self, timeout=10.0):
        """Wait for camera images"""
        start = time.time()
        while (self.latest_rgb is None or self.latest_depth is None):
            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for camera images!")
                return False
            time.sleep(0.1)
        self.get_logger().info("✓ Camera images received!")
        return True
    
    def detect_object_pose(self, color):
        """Detect object and return 3D pose"""
        if self.latest_rgb is None or self.latest_depth is None:
            return None
        
        detection = self.detector.detect(self.latest_rgb, color)
        if detection is None:
            return None
        
        u, v = detection['center']
        depth = get_depth_at_pixel(self.latest_depth, u, v, window_size=5)
        
        if depth is None or depth <= 0.1:
            return None
        
        world_pos = self.pixel_to_3d.pixel_to_3d(u, v, depth)
        return world_pos
    
    def detect_all_objects(self):
        """Detect all boxes and plates"""
        all_detections = self.detector.detect_all(self.latest_rgb)
        
        boxes = {}
        plates = {}
        
        for color in ['red', 'blue', 'green']:
            if color not in all_detections:
                continue
            
            detection = all_detections[color]
            u, v = detection['center']
            depth = get_depth_at_pixel(self.latest_depth, u, v)
            
            if depth is None or depth <= 0.1:
                continue
            
            world_pos = self.pixel_to_3d.pixel_to_3d(u, v, depth)
            
            # Heuristic: boxes on shelf (>20cm), plates on floor (<20cm)
            # Adjusted threshold to 0.05m (5cm) as plates are very thin
            if world_pos[2] > 0.05:
                boxes[color] = world_pos
            else:
                plates[color] = world_pos
        
        return boxes, plates

    def joint_callback(self, msg):
        """Monitor gripper joint for attachment logic"""
        self.latest_joint_state = msg
        try:
            idx = msg.name.index('j7l')
            pos = msg.position[idx]
        except (ValueError, IndexError):
            return
        
        is_open = pos > self.open_thresh
        is_stalled = self.stall_min < pos < self.stall_max
        
        old_state = self.gripper_state
        
        if is_open:
            if self.gripper_state == GripperState.ATTACHED:
                self.detach()
            self.gripper_state = GripperState.OPEN
            
        elif is_stalled:
            if self.gripper_state != GripperState.ATTACHED:
                self.gripper_state = GripperState.GRASPING
                # Use current_target_box instead of defaulting to 'red'
                self.grasp(pos, self.current_target_box if self.current_target_box else 'red')
        
        self.last_gripper_pos = pos

    def grasp(self, current_pos, box_color='red'):
        """Execute grasp: stop gripper + attach colored box"""
        msg = JointTrajectory()
        msg.joint_names = ['j7l', 'j7r']
        point = JointTrajectoryPoint()
        point.positions = [current_pos, -current_pos]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        msg.points = [point]
        self.traj_pub.publish(msg)
        
        # Attach the specified colored box
        if box_color in self.attach_pubs:
            self.attach_pubs[box_color].publish(Empty())
            self.current_attached_box = box_color
            self.gripper_state = GripperState.ATTACHED
            self.get_logger().info(f"  ✅ {box_color.upper()} Box ATTACHED")
        else:
            self.get_logger().error(f"Unknown box color: {box_color}")

    def detach(self):
        """Detach currently attached object"""
        if self.current_attached_box and self.current_attached_box in self.detach_pubs:
            self.detach_pubs[self.current_attached_box].publish(Empty())
            self.get_logger().info(f"  ✅ {self.current_attached_box.upper()} Box RELEASED")
            self.current_attached_box = None
        
        self.gripper_state = GripperState.OPEN

    def move_arm_joints(self, positions, duration_sec=3.0):
        """Move arm to joint positions and wait for completion"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints
        
        # If 8 positions provided (including gripper), extract only arm joints (first 6)
        if len(positions) == 8:
            arm_positions = positions[:6]
            self.get_logger().info(f"Extracted arm joints from 8-joint array: {[f'{p:.3f}' for p in arm_positions]}")
        elif len(positions) == 6:
            arm_positions = positions
        else:
            self.get_logger().error(f"Invalid number of joint positions: {len(positions)}, expected 6 or 8")
            return
        
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        # Send goal and wait for completion
        goal_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=duration_sec + 2.0)
        
        if goal_future.result() is not None:
            goal_handle = goal_future.result()
            if goal_handle.accepted:
                # Wait for the result
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 2.0)
                self.get_logger().info("Arm movement completed")
            else:
                self.get_logger().warn("Arm movement goal rejected")
        else:
            self.get_logger().warn("Arm movement goal failed")

    def move_arm_to_pose(self, pose, approach_offset=0.0, use_seed=True, avoid_collisions=True):
        """Move arm to Cartesian pose using IK"""
        # Apply offset if needed
        target_pose = list(pose)
        target_pose[2] += approach_offset
        
        # Create seed state if requested
        seed_state = None
        if use_seed:
            from sensor_msgs.msg import JointState
            seed_state = JointState()
            seed_state.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
            seed_state.position = self.shelf_seed
        
        # Get IK solution
        joint_state = self.ik_solver.get_ik(target_pose, current_joint_state=seed_state, avoid_collisions=avoid_collisions)
        
        if joint_state:
            # Extract positions in the correct order
            # The IK service returns joint_state with names and positions
            # We need to reorder them to match our arm controller's expected order: j1,j2,j3,j4,j5,j6
            try:
                positions = []
                for joint_name in ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']:
                    idx = joint_state.name.index(joint_name)
                    positions.append(joint_state.position[idx])
                
                self.get_logger().info(f"IK solution: {[f'{p:.3f}' for p in positions]}")
                self.move_arm_joints(positions, duration_sec=3.0)
                return True
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Failed to extract joint positions: {e}")
                return False
        else:
            self.get_logger().error(f"Failed to find IK solution for {target_pose}")
            return False

    def open_gripper(self):
        """Open gripper"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        self.gripper_client.send_goal_async(goal_msg)

    def close_gripper(self, attach_box_color=None):
        """Close gripper and optionally attach a box"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [-0.035, 0.035]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        self.gripper_client.send_goal_async(goal_msg)
        
        # If a box color is specified, attach it after gripper closes
        if attach_box_color:
            time.sleep(2.5)  # Wait for gripper to close
            
            # First, detach ALL boxes to ensure only one is attached
            self.get_logger().info("Detaching all boxes before attaching new one...")
            for color in ['red', 'green', 'blue']:
                if color in self.detach_pubs:
                    self.detach_pubs[color].publish(Empty())
            time.sleep(0.5)
            
            # Now attach the specified box
            if attach_box_color in self.attach_pubs:
                self.attach_pubs[attach_box_color].publish(Empty())
                self.current_attached_box = attach_box_color
                self.gripper_state = GripperState.ATTACHED
                self.get_logger().info(f"  ✅ {attach_box_color.upper()} Box ATTACHED")

    def pick_and_place(self, box_color):
        """Pick box and place on matching plate"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🎯 PICKING {box_color.upper()} BOX")
        self.get_logger().info(f"{'='*60}")
        
        # 1. Detect Box
        boxes, plates = self.detect_all_objects()
        
        if box_color not in boxes:
            self.get_logger().error(f"❌ {box_color} box not detected!")
            return False
            
        box_pos = boxes[box_color]
        self.get_logger().info(f"📦 {box_color} box found at: {box_pos}")
        
        # 2. Detect Plate (Optional - fallback to fixed offset if not found)
        plate_pos = plates.get(box_color)
        if plate_pos is None:
            self.get_logger().warn(f"⚠️ {box_color} plate not detected, using default offset")
            # Default offset: 20cm to the right of box
            plate_pos = box_pos.copy()
            plate_pos[1] -= 0.2 
        else:
            self.get_logger().info(f"🍽️ {box_color} plate found at: {plate_pos}")

        # 3. Move to Home
        self.get_logger().info("Moving to home...")
        self.move_arm_joints(self.home_position, 3.0)
        time.sleep(3.5)
        
        # 4. Approach Box
        self.get_logger().info("Approaching box...")
        # Move 5cm above box (reduced from 15cm to stay in workspace)
        # Disable collision checking as requested to allow picking from tables
        if not self.move_arm_to_pose(box_pos, approach_offset=0.05, avoid_collisions=False):
            return False
        time.sleep(3.5)
        
        # 5. Descend to Pick
        self.get_logger().info("Descend to pick...")
        # Move to grasp height (offset for gripper length)
        if not self.move_arm_to_pose(box_pos, approach_offset=0.08, avoid_collisions=False):
            return False
        time.sleep(3.5)
        
        # 6. Close Gripper (Pick)
        self.get_logger().info("Closing gripper...")
        self.close_gripper()
        time.sleep(3.0) # Wait for attach
        
        # 7. Lift Box
        self.get_logger().info("Lifting box...")
        if not self.move_arm_to_pose(box_pos, approach_offset=0.05, avoid_collisions=False):
            return False
        time.sleep(3.5)
        
        # 8. Move to Plate Approach
        self.get_logger().info("Moving to plate...")
        if not self.move_arm_to_pose(plate_pos, approach_offset=0.05, avoid_collisions=False):
            return False
        time.sleep(4.0)
        
        # 9. Descend to Place
        self.get_logger().info("Placing...")
        if not self.move_arm_to_pose(plate_pos, approach_offset=0.08, avoid_collisions=False):
            return False
        time.sleep(3.5)
        
        # 10. Open Gripper (Release)
        self.get_logger().info("Releasing...")
        self.open_gripper()
        time.sleep(2.0) # Wait for detach
        
        # 11. Return Home
        self.get_logger().info("Returning home...")
        self.move_arm_to_pose(plate_pos, approach_offset=0.05) # Lift first
        time.sleep(2.0)
        self.move_arm_joints(self.home_position, 3.0)
        time.sleep(3.5)
        
        self.get_logger().info(f"✅ {box_color} box placed!")
        return True

    def test_predefined_red(self):
        """Simplified test: just pick and attach red box, then stop"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🧪 TESTING RED BOX ATTACH ONLY")
        self.get_logger().info("="*60)
        
        # Hardcoded joint positions from SRDF (red_pick)
        red_pick_joints = [
            9.990135855041453e-05,    # j1
            0.44531366847265,          # j2
            0.4509790868877058,        # j3
            0.3652866062418217,        # j4
            0.4110300069085086,        # j5
            -8.548499507841293e-07,   # j6
            0.0,                       # j7l (gripper open)
            0.0                        # j7r (gripper open)
        ]
        
        # ========== STEP 1: Move to pick position ==========
        self.get_logger().info("\n🔵 STEP 1: Moving to red_pick position")
        self.move_arm_joints(red_pick_joints, 4.0)
        time.sleep(2.0)
        
        # ========== STEP 2: Close gripper and wait for completion ==========
        self.get_logger().info("\n🔵 STEP 2: Closing gripper")
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [-0.035, 0.035]
        point.time_from_start = Duration(sec=3, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        # Send goal and wait for completion
        self.get_logger().info("Sending gripper close command...")
        goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        
        if goal_future.result() is not None:
            goal_handle = goal_future.result()
            if goal_handle.accepted:
                self.get_logger().info("Gripper goal accepted, waiting for completion...")
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
                self.get_logger().info("✅ Gripper close command completed")
            else:
                self.get_logger().error("❌ Gripper goal rejected")
        else:
            self.get_logger().error("❌ Gripper goal failed")
        
        # Wait a bit more for contact detection
        self.get_logger().info("Waiting for contact detection...")
        time.sleep(2.0)
        
        # ========== STEP 3: Check actual gripper state ==========
        self.get_logger().info("\n🔵 STEP 3: Checking gripper state")
        self.get_logger().info(f"Current gripper state: {self.gripper_state}")
        self.get_logger().info(f"Last gripper position: {self.last_gripper_pos:.4f}")
        
        if self.gripper_state == GripperState.ATTACHED:
            self.get_logger().info("✅ RED BOX ATTACHED via contact detection!")
        elif self.gripper_state == GripperState.GRASPING:
            self.get_logger().info("⚠️  Gripper is GRASPING (contact detected but not attached)")
            self.get_logger().info("Manually triggering attach...")
            if 'red' in self.attach_pubs:
                self.attach_pubs['red'].publish(Empty())
                self.current_attached_box = 'red'
                self.gripper_state = GripperState.ATTACHED
                self.get_logger().info("✅ RED Box manually ATTACHED")
                time.sleep(1.0)
        elif self.gripper_state == GripperState.OPEN:
            self.get_logger().warn("❌ Gripper is still OPEN - no contact detected")
            self.get_logger().info("Manually triggering attach anyway...")
            if 'red' in self.attach_pubs:
                self.attach_pubs['red'].publish(Empty())
                self.current_attached_box = 'red'
                self.gripper_state = GripperState.ATTACHED
                self.get_logger().info("✅ RED Box manually ATTACHED")
                time.sleep(1.0)
        else:
            self.get_logger().warn(f"❌ Unknown gripper state: {self.gripper_state}")
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🛑 TEST STOPPED")
        self.get_logger().info("Check in Gazebo if red box is attached to gripper")
        self.get_logger().info("="*60)
        
        return True

    def test_table_pick(self, table_num):
        """Test pick from specific table with contact-based attachment"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🧪 TESTING TABLE {table_num} PICK")
        self.get_logger().info(f"{'='*60}")
        
        # Get pick position for this table
        pick_pos = self.pick_positions[f'table{table_num}']
        
        # Determine which box color to attach based on user observation
        # Table 1 (Magenta) -> Blue Box
        # Table 2 (Yellow) -> Red Box
        # Table 3 (Cyan) -> Green Box
        box_color_map = {'table1': 'blue', 'table2': 'red', 'table3': 'green'}
        box_color = box_color_map[f'table{table_num}']
        self.current_target_box = box_color
        
        # Step 1: Move to pick position
        self.get_logger().info(f"\n🔵 STEP 1: Moving to Table {table_num} pick position")
        self.move_arm_joints(pick_pos, 4.0)
        time.sleep(4.5)
        
        # Step 2: Close gripper (contact detection will handle attachment)
        self.get_logger().info(f"\n🔵 STEP 2: Closing gripper to pick {box_color.upper()} box")
        self.gripper_state = GripperState.CLOSING
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [-0.035, 0.035]
        point.time_from_start = Duration(sec=3, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        # Send goal and wait for completion
        self.get_logger().info("Sending gripper close command...")
        goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        
        if goal_future.result() is not None:
            goal_handle = goal_future.result()
            if goal_handle.accepted:
                self.get_logger().info("Gripper goal accepted, waiting for completion...")
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
                self.get_logger().info("✅ Gripper close command completed")
        
        # Wait for contact detection
        self.get_logger().info("Waiting for contact detection...")
        time.sleep(2.0)
        
        # Step 3: Check if attached, if not manually attach
        self.get_logger().info(f"\n🔵 STEP 3: Checking attachment status")
        self.get_logger().info(f"Current gripper state: {self.gripper_state}")
        
        if self.gripper_state != GripperState.ATTACHED:
            self.get_logger().info("⚠️  AUTO-ATTACH DISABLED - Use manual_control_and_viz.py to attach")
            # DISABLED: Use manual_control_and_viz.py for attachment instead
            # if box_color in self.attach_pubs:
            #     self.attach_pubs[box_color].publish(Empty())
            #     self.current_attached_box = box_color
            #     self.gripper_state = GripperState.ATTACHED
        else:
            self.get_logger().info(f"✅ {box_color.upper()} Box ATTACHED")
            time.sleep(1.0)
        
        # Step 4: Hold and wait for user
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🛑 HOLDING - Box attached to gripper")
        self.get_logger().info("="*60)
        input("\nPress Enter to show detach option...")
        
        # Step 5: Detach option
        choice = input("\nDetach box now? (y/n): ").strip().lower()
        if choice == 'y':
            self.get_logger().info("Opening gripper and detaching...")
            self.open_gripper()
            time.sleep(2.0)
            self.get_logger().info("✅ Box detached")
        else:
            self.get_logger().info("Box still attached. You can manually move arm in RViz.")
            input("Press Enter when done...")
        
        return True
    
    def print_current_joints(self):
        """Print current joint positions for recording"""
        if self.latest_joint_state is None:
            self.get_logger().warn("No joint state received yet")
            return
            
        try:
            # Extract arm joints in order
            arm_positions = []
            for name in self.arm_joints:
                if name in self.latest_joint_state.name:
                    idx = self.latest_joint_state.name.index(name)
                    arm_positions.append(self.latest_joint_state.position[idx])
            
            if len(arm_positions) == 6:
                print("\n" + "="*60)
                print("📍 CURRENT JOINT POSITIONS")
                print("="*60)
                print(f"[{', '.join([f'{p:.6f}' for p in arm_positions])}]")
                print("="*60 + "\n")
            else:
                self.get_logger().warn(f"Could not find all arm joints. Found: {len(arm_positions)}")
                
        except Exception as e:
            self.get_logger().error(f"Error printing joints: {e}")

    def run_interactive_menu(self):
        """Interactive menu for user selection"""
        while self.running:
            print("\n" + "="*60)
            print("🤖 HARDCODED PICK AND PLACE SYSTEM")
            print("="*60)
            print("\n🧪 PICK TESTING (Contact-Based Attachment):")
            print("  1. Test Table 1 (MAGENTA) - Pick only, then hold")
            print("  2. Test Table 2 (YELLOW) - Pick only, then hold")
            print("  3. Test Table 3 (CYAN) - Pick only, then hold")
            print("\n📊 DETECTION & RECORDING:")
            print("  4. Show detected boxes and tables")
            print("  5. Print Current Joint Positions (for recording)")
            print("\n  0. Exit")
            print()
            
            choice = input("Enter your choice: ").strip()
            
            if choice == '0':
                print("Exiting...")
                self.running = False
                break
            elif choice == '1':
                self.test_table_pick(1)
            elif choice == '2':
                self.test_table_pick(2)
            elif choice == '3':
                self.test_table_pick(3)
            elif choice == '4':
                boxes, plates = self.detect_all_objects()
                print("\n📦 Detected Boxes:")
                for color, pos in boxes.items():
                    print(f"  {color}: {pos}")
                print("\n🎯 Detected Plates:")
                for color, pos in plates.items():
                    print(f"  {color}: {pos}")
            elif choice == '5':
                self.print_current_joints()
            else:
                print("Invalid choice!")


def main():
    rclpy.init()
    node = VisionInteractiveDemo()
    # Spinning happens in background thread


if __name__ == "__main__":
    main()
