import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch Gazebo + MoveIt
    unified_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arm_moveit_config"),
                "launch",
                "unified_gz_moveit.launch.py"
            )
        )
    )

    # Vision node for detecting boxes and plates
    vision_node = Node(
        package="robo_vision",
        executable="color_detector",
        name="detector",
        output="screen",
        parameters=[
            {"camera_topic": "/camera/image_raw"},
            {"detection_type": "box"}
        ]
    )

    return LaunchDescription([
        unified_launch,
        vision_node,
    ])
