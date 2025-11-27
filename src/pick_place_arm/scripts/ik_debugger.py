#!/usr/bin/env python3
"""
IK Debugging Tool
Tests IK service with various parameters to understand failures
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState


class IKDebugger(Node):
    def __init__(self):
        super().__init__('ik_debugger')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        self.get_logger().info("Waiting for /compute_ik service...")
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK service not available!")
            return
        
        self.get_logger().info("✓ IK service ready!")
    
    def test_ik(self, position, orientation=[0.0, 1.0, 0.0, 0.0], seed=None, group="arm"):
        """Test IK with given parameters"""
        req = GetPositionIK.Request()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped.header.frame_id = "world"
        req.ik_request.timeout.sec = 1
        req.ik_request.avoid_collisions = True
        
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        
        req.ik_request.pose_stamped.pose = pose
        
        if seed:
            req.ik_request.robot_state.joint_state = seed
        
        self.get_logger().info(f"\nTesting IK:")
        self.get_logger().info(f"  Position: {position}")
        self.get_logger().info(f"  Orientation: {orientation}")
        self.get_logger().info(f"  Seed: {'Yes' if seed else 'No'}")
        
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                self.get_logger().info(f"  ✅ SUCCESS!")
                self.get_logger().info(f"  Solution: {response.solution.joint_state.position[:6]}")
                return True
            else:
                self.get_logger().error(f"  ❌ FAILED with error code: {response.error_code.val}")
                return False
        else:
            self.get_logger().error(f"  ❌ Service call failed")
            return False
    
    def run_tests(self):
        """Run a series of IK tests"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("IK DEBUGGING TESTS")
        self.get_logger().info("="*60)
        
        # Test 1: Home position (should always work)
        self.get_logger().info("\n[Test 1] Home position")
        self.test_ik([0.0, 0.0, 0.3])
        
        # Test 2: Red box position (from SDF)
        self.get_logger().info("\n[Test 2] Red box position (SDF)")
        self.test_ik([0.093550, 0.192664, 0.453342])
        
        # Test 3: Red box + 5cm offset
        self.get_logger().info("\n[Test 3] Red box + 5cm offset")
        self.test_ik([0.093550, 0.192664, 0.503342])
        
        # Test 4: Same with seed state
        self.get_logger().info("\n[Test 4] Red box + 5cm offset WITH SEED")
        seed = JointState()
        seed.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        seed.position = [0.0, 1.027, 0.976, 0.599, 0.0, 0.0]
        self.test_ik([0.093550, 0.192664, 0.503342], seed=seed)
        
        # Test 5: Lower target
        self.get_logger().info("\n[Test 5] Lower target (z=0.4)")
        self.test_ik([0.093550, 0.192664, 0.4], seed=seed)
        
        # Test 6: Different orientation
        self.get_logger().info("\n[Test 6] Different orientation (45° tilt)")
        self.test_ik([0.093550, 0.192664, 0.453342], 
                     orientation=[0.0, 0.924, 0.0, 0.383], seed=seed)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Tests complete!")
        self.get_logger().info("="*60 + "\n")


def main():
    rclpy.init()
    node = IKDebugger()
    node.run_tests()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
