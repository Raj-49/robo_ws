from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Wrapper to keep backward compatibility. Use unified_gz_moveit from arm_moveit_config.
    unified = os.path.join(
        get_package_share_directory('arm_moveit_config'),
        'launch',
        'unified_gz_moveit.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unified),
            launch_arguments={
                'start_rviz': 'false',  # Gazebo + robot only by default
                'use_sim_time': 'true',
            }.items(),
        )
    ])
