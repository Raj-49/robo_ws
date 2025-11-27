#!/usr/bin/env python3
"""
Vision Utilities - Pixel to 3D conversion using depth
"""

import numpy as np
from scipy.spatial.transform import Rotation


class PixelTo3DConverter:
    """Convert pixel coordinates to 3D world coordinates using depth"""
    
    def __init__(self, camera_params):
        """
        Initialize converter with camera parameters
        
        Args:
            camera_params: dict from camera_calibration.parse_camera_sdf()
        """
        self.fx = camera_params['fx']
        self.fy = camera_params['fy']
        self.cx = camera_params['cx']
        self.cy = camera_params['cy']
        
        # Camera pose in world frame (from SDF)
        # This is the pose of the camera LINK
        self.cam_position = camera_params['position']
        self.cam_rotation = camera_params['rotation']
        
        # Transform from Optical Frame (Z forward, X right, Y down) 
        # to Link Frame (X forward, Y left, Z up)
        # R_link_optical:
        # Optical X (Right) -> Link -Y (Right is -Y in standard ROS link? No, Y is Left)
        # Let's use standard Gazebo/ROS convention:
        # Optical: X=Right, Y=Down, Z=Forward
        # Link: X=Forward, Y=Left, Z=Up
        # Rotation: Roll=-90, Yaw=-90
        
        # Constructing R_link_optical explicitly:
        # col 0 (Optical X) -> Link -Y  [0, -1, 0]
        # col 1 (Optical Y) -> Link -Z  [0, 0, -1]
        # col 2 (Optical Z) -> Link X   [1, 0, 0]
        self.R_link_optical = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ])
        
        # Combined rotation: World <- Link <- Optical
        self.R_world_optical = self.cam_rotation @ self.R_link_optical
    
    def pixel_to_3d(self, u, v, depth):
        """
        Convert pixel coordinates to 3D world coordinates
        
        Uses pinhole camera model with depth:
          X_cam = (u - cx) * Z / fx
          Y_cam = (v - cy) * Z / fy
          Z_cam = depth
        
        Then transforms to world frame:
          P_world = R_cam * P_cam + t_cam
        
        Args:
            u: pixel x-coordinate (column)
            v: pixel y-coordinate (row)
            depth: depth value at pixel (meters)
        
        Returns:
            np.array([x, y, z]) in world frame
        """
        # Convert pixel to 3D point in camera frame
        x_cam = (u - self.cx) * depth / self.fx
        y_cam = (v - self.cy) * depth / self.fy
        z_cam = depth
        
        point_cam = np.array([x_cam, y_cam, z_cam])
        
        # Transform to world frame
        # P_world = R_world_optical * P_optical + T_world_link
        point_world = self.R_world_optical @ point_cam + self.cam_position
        
        return point_world
    
    def pixels_to_3d_batch(self, pixels, depths):
        """
        Convert multiple pixels to 3D (vectorized)
        
        Args:
            pixels: Nx2 array of (u, v) coordinates
            depths: N array of depth values
        
        Returns:
            Nx3 array of (x, y, z) world coordinates
        """
        pixels = np.array(pixels)
        depths = np.array(depths)
        
        # Camera frame coordinates
        x_cam = (pixels[:, 0] - self.cx) * depths / self.fx
        y_cam = (pixels[:, 1] - self.cy) * depths / self.fy
        z_cam = depths
        
        points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
        
        # Transform to world frame
        points_world = (self.R_world_optical @ points_cam.T).T + self.cam_position
        
        return points_world


def get_depth_at_pixel(depth_image, u, v, window_size=5):
    """
    Get depth value at pixel with averaging to reduce noise
    
    Args:
        depth_image: HxW depth image (float32, meters)
        u, v: pixel coordinates
        window_size: size of averaging window (odd number)
    
    Returns:
        Average depth value (meters)
    """
    h, w = depth_image.shape
    
    # Ensure pixel is in bounds
    u = int(np.clip(u, 0, w - 1))
    v = int(np.clip(v, 0, h - 1))
    
    # Extract window around pixel
    half = window_size // 2
    v_min = max(0, v - half)
    v_max = min(h, v + half + 1)
    u_min = max(0, u - half)
    u_max = min(w, u + half + 1)
    
    window = depth_image[v_min:v_max, u_min:u_max]
    
    # Filter out invalid depths (0 or inf)
    valid = (window > 0.1) & (window < 10.0)
    
    if not np.any(valid):
        return None
    
    # Return median of valid depths (more robust than mean)
    return np.median(window[valid])


if __name__ == "__main__":
    # Test with example camera parameters
    from camera_calibration import parse_camera_sdf
    import os
    
    sdf_path = os.path.expanduser('~/robo_ws/src/pick_place_arm/worlds/my_world.sdf')
    
    try:
        params = parse_camera_sdf(sdf_path)
        converter = PixelTo3DConverter(params)
        
        # Test conversion
        u, v = 320, 240  # Center pixel
        depth = 1.5  # 1.5 meters
        
        world_pos = converter.pixel_to_3d(u, v, depth)
        print(f"Pixel ({u}, {v}) at depth {depth}m")
        print(f"  → World position: {world_pos}")
        
    except Exception as e:
        print(f"Error: {e}")
