#!/usr/bin/env python3
"""
Vision Pick and Place Node
---------------------------
Detects colored boxes and plates using RGB-D camera.
Provides pose estimation for pick and place operations.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import sys
import os

# Add scripts directory to path
sys.path.append(os.path.dirname(__file__))

from camera_calibration import parse_camera_sdf
from vision_utils import PixelTo3DConverter, get_depth_at_pixel
from object_detector import ColorObjectDetector


class VisionPickPlaceNode(Node):
    """Vision node for detecting boxes and plates"""
    
    def __init__(self):
        super().__init__('vision_pick_place_node')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Load camera calibration
        sdf_path = os.path.expanduser('~/robo_ws/src/pick_place_arm/worlds/my_world.sdf')
        try:
            cam_params = parse_camera_sdf(sdf_path, 'camera')
            self.get_logger().info("Camera calibration loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load camera calibration: {e}")
            raise
        
        # Initialize converters
        self.pixel_to_3d = PixelTo3DConverter(cam_params)
        self.detector = ColorObjectDetector()
        
        # Subscribe to camera topics
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10
        )
        
        # Latest images
        self.latest_rgb = None
        self.latest_depth = None
        
        self.get_logger().info("Vision Pick Place Node Started")
        self.get_logger().info("Waiting for camera images...")
    
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            # Depth is R_FLOAT32 format
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
    
    def detect_object_pose(self, color):
        """
        Detect object of specified color and return 3D pose
        
        Args:
            color: 'red', 'blue', or 'green'
        
        Returns:
            np.array([x, y, z]) in world frame, or None if not found
        """
        if self.latest_rgb is None or self.latest_depth is None:
            self.get_logger().warn("No camera images available yet")
            return None
        
        # Detect in RGB image
        detection = self.detector.detect(self.latest_rgb, color)
        
        if detection is None:
            self.get_logger().warn(f"No {color} object detected")
            return None
        
        u, v = detection['center']
        
        # Get depth at detected pixel
        depth = get_depth_at_pixel(self.latest_depth, u, v, window_size=5)
        
        if depth is None or depth <= 0.1:
            self.get_logger().warn(f"Invalid depth at pixel ({u}, {v})")
            return None
        
        # Convert to 3D world coordinates
        world_pos = self.pixel_to_3d.pixel_to_3d(u, v, depth)
        
        self.get_logger().info(
            f"Detected {color} at pixel ({u}, {v}), depth={depth:.3f}m → world {world_pos}"
        )
        
        return world_pos
    
    def detect_all_objects(self):
        """
        Detect all colored boxes and plates
        
        Returns:
            dict with 'boxes' and 'plates', each mapping color -> pose
        """
        if self.latest_rgb is None or self.latest_depth is None:
            return {'boxes': {}, 'plates': {}}
        
        # Detect all colors
        all_detections = self.detector.detect_all(self.latest_rgb)
        
        results = {'boxes': {}, 'plates': {}}
        
        for color, detection in all_detections.items():
            u, v = detection['center']
            depth = get_depth_at_pixel(self.latest_depth, u, v)
            
            if depth is None or depth <= 0.1:
                continue
            
            world_pos = self.pixel_to_3d.pixel_to_3d(u, v, depth)
            
            # Heuristic: boxes are higher (smaller z), plates are lower (larger z)
            # Adjust based on your setup
            if world_pos[2] > 0.02:  # Above 2cm - likely a box
                results['boxes'][color] = world_pos
            else:  # On ground - likely a plate
                results['plates'][color] = world_pos
        
        return results
    
    def wait_for_images(self, timeout=5.0):
        """Wait for camera images to be available"""
        import time
        start = time.time()
        
        while (self.latest_rgb is None or self.latest_depth is None):
            if time.time() - start > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return True


def main():
    rclpy.init()
    node = VisionPickPlaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
