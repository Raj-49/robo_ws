#!/usr/bin/env python3
"""
Camera Calibration - Extract intrinsics from SDF
"""

import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation


def parse_camera_sdf(sdf_path, camera_name='camera'):
    """
    Extract camera parameters from SDF file
    
    Args:
        sdf_path: Path to SDF file
        camera_name: Name of camera model in SDF
    
    Returns:
        dict with camera parameters
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    
    # Find camera model
    camera_model = None
    for model in root.findall('.//model'):
        if model.get('name') == camera_name:
            camera_model = model
            break
    
    if camera_model is None:
        raise ValueError(f"Camera '{camera_name}' not found in SDF")
    
    # Get camera sensor (RGB)
    sensor = camera_model.find('.//sensor[@type="camera"]')
    if sensor is None:
        raise ValueError(f"Camera sensor not found in model '{camera_name}'")
    
    camera_elem = sensor.find('camera')
    
    # Extract image parameters
    width = int(camera_elem.find('image/width').text)
    height = int(camera_elem.find('image/height').text)
    hfov = float(camera_elem.find('horizontal_fov').text)
    
    # Calculate focal length (pinhole camera model)
    # fx = (width / 2) / tan(hfov / 2)
    fx = (width / 2.0) / np.tan(hfov / 2.0)
    fy = fx  # Assume square pixels
    
    # Principal point (image center)
    cx = width / 2.0
    cy = height / 2.0
    
    # Camera pose in world frame
    pose_elem = camera_model.find('pose')
    pose_str = pose_elem.text.strip().split()
    pose = [float(x) for x in pose_str]
    
    # Parse pose: [x, y, z, roll, pitch, yaw]
    position = np.array(pose[:3])
    rotation = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    
    return {
        'width': width,
        'height': height,
        'fx': fx,
        'fy': fy,
        'cx': cx,
        'cy': cy,
        'position': position,
        'rotation': rotation,
        'pose': pose
    }


if __name__ == "__main__":
    # Test with default path
    import os
    sdf_path = os.path.expanduser('~/robo_ws/src/pick_place_arm/worlds/my_world.sdf')
    
    try:
        params = parse_camera_sdf(sdf_path)
        print("Camera Parameters:")
        print(f"  Resolution: {params['width']}x{params['height']}")
        print(f"  Focal Length: fx={params['fx']:.2f}, fy={params['fy']:.2f}")
        print(f"  Principal Point: cx={params['cx']:.2f}, cy={params['cy']:.2f}")
        print(f"  Position: {params['position']}")
        print(f"  Rotation:\n{params['rotation']}")
    except Exception as e:
        print(f"Error: {e}")
