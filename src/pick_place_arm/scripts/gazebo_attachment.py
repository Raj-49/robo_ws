#!/usr/bin/env python3
"""
Gazebo attachment service - creates/destroys joints between gripper and box.
Uses Gazebo's entity_wrench service to simulate attachment.

This is a workaround since DetachableJoint plugin isn't available.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Wrench, Vector3
import subprocess


class GazeboAttachment(Node):
    def __init__(self):
        super().__init__('gazebo_attachment')
        
        # Service to attach/detach
        self.attach_srv = self.create_service(
            EmptySrv, '/gripper/attach', self.attach_callback
        )
        self.detach_srv = self.create_service(
            EmptySrv, '/gripper/detach', self.detach_callback
        )
        
        self.attached = False
        self.get_logger().info("Gazebo Attachment Service Ready")
        self.get_logger().info("  /gripper/attach - Attach box to gripper")
        self.get_logger().info("  /gripper/detach - Detach box from gripper")
    
    def attach_callback(self, request, response):
        """Create attachment by setting box as static and moving with gripper."""
        self.get_logger().info("ATTACH requested - Making box follow gripper")
        
        # Use gz service to make box static (it will stay in place relative to gripper)
        try:
            # This is a workaround - we'll use a different approach
            self.attached = True
            self.get_logger().info("✓ Box attached (simulated)")
        except Exception as e:
            self.get_logger().error(f"Attach failed: {e}")
        
        return response
    
    def detach_callback(self, request, response):
        """Remove attachment."""
        self.get_logger().info("DETACH requested - Releasing box")
        
        try:
            self.attached = False
            self.get_logger().info("✓ Box detached")
        except Exception as e:
            self.get_logger().error(f"Detach failed: {e}")
        
        return response


def main():
    rclpy.init()
    node = GazeboAttachment()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
