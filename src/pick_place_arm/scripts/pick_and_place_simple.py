#!/usr/bin/env python3
"""
Simple pick and place test - just moves to home position to verify MoveIt connection.
This is a simplified version to test the setup before full pick-and-place.

Usage:
  python3 install/pick_place_arm/share/pick_place_arm/scripts/pick_and_place_simple.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SimplePickAndPlace(Node):
    def __init__(self):
        super().__init__("simple_pick_and_place")

        # Subscribe to vision coordinates
        self.sub = self.create_subscription(
            String, "/box_coordinates", self.coords_callback, 10
        )
        
        self.detected_boxes = set()
        self.detected_plates = set()
        
        self.get_logger().info("Simple Pick and Place Node Started")
        self.get_logger().info("Listening for detections on /box_coordinates")
        self.get_logger().info("This is a test version - just monitors detections")

    def coords_callback(self, msg):
        """Process detected coordinates."""
        try:
            parts = msg.data.split(",")
            obj_id = parts[0]  # e.g., "box_R" or "plate_G"
            
            if "_" in obj_id:
                obj_type, color = obj_id.split("_")
                
                if obj_type == "box":
                    if color not in self.detected_boxes:
                        self.detected_boxes.add(color)
                        self.get_logger().info(f"✓ Detected BOX: {color} at pixel ({parts[1]}, {parts[2]})")
                elif obj_type == "plate":
                    if color not in self.detected_plates:
                        self.detected_plates.add(color)
                        self.get_logger().info(f"✓ Detected PLATE: {color} at pixel ({parts[1]}, {parts[2]})")
                
                # Show status
                if len(self.detected_boxes) > 0 or len(self.detected_plates) > 0:
                    self.get_logger().info(
                        f"Status: Boxes={sorted(self.detected_boxes)}, Plates={sorted(self.detected_plates)}"
                    )
                    
        except Exception as e:
            self.get_logger().error(f"Error parsing: {e}")


def main():
    rclpy.init()
    node = SimplePickAndPlace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
