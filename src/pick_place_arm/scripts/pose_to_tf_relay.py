#!/usr/bin/env python3
"""
Pose to TF Relay
----------------
Subscribes to /model_poses (from Gazebo bridge), filters for relevant objects,
ensures valid frame_ids, and republishes to /tf.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class PoseToTFRelay(Node):
    def __init__(self):
        super().__init__("pose_to_tf_relay")
        
        # Objects we want to visualize in RViz
        self.target_frames = {
            'red_box', 'green_box', 'blue_box',
            'pillar_table-1', 'pillar_table-2', 'pillar_table-3',
            'basket_red', 'basket_green', 'basket_blue'
        }
        
        self.sub = self.create_subscription(
            TFMessage,
            '/model_poses',
            self.callback,
            10
        )
        
        self.pub = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )
        
        self.get_logger().info("Pose to TF Relay Started")

    def callback(self, msg):
        self.get_logger().info(f"Received pose message with {len(msg.transforms)} transforms", throttle_duration_sec=2.0)
        clean_tf_msg = TFMessage()
        
        for transform in msg.transforms:
            self.get_logger().debug(f"Processing transform: child_frame_id={transform.child_frame_id}, frame_id={transform.header.frame_id}")
            # Check if this is a model root transform we care about
            # Gazebo bridge usually publishes model names as child_frame_id
            if transform.child_frame_id in self.target_frames:
                self.get_logger().info(f"Found target frame: {transform.child_frame_id}", throttle_duration_sec=5.0)
                
                # Fix frame_id if empty (Gazebo often leaves it empty for world-relative poses)
                if not transform.header.frame_id:
                    self.get_logger().debug(f"Fixing empty frame_id for {transform.child_frame_id} to 'world'.")
                    transform.header.frame_id = 'world'
                
                # Update timestamp to current time
                transform.header.stamp = self.get_clock().now().to_msg()
                
                clean_tf_msg.transforms.append(transform)
                self.get_logger().debug(f"Added transform for {transform.child_frame_id} to clean_tf_msg.")
            else:
                self.get_logger().debug(f"Transform {transform.child_frame_id} is not a target frame. Skipping.")
        
        if clean_tf_msg.transforms:
            self.get_logger().info(f"Publishing {len(clean_tf_msg.transforms)} clean transforms to /tf", throttle_duration_sec=2.0)
            self.pub.publish(clean_tf_msg)
        else:
            self.get_logger().warn("No target transforms found to publish. Check if models exist in Gazebo.", throttle_duration_sec=5.0)

def main():
    rclpy.init()
    node = PoseToTFRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
