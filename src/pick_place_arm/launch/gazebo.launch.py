import os
import tempfile
import xacro  # Import xacro for processing Xacro files
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Define package paths
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    pkg_robot_description = get_package_share_directory('pick_place_arm')

    # File paths
    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'arm.urdf.xacro')
    world_file = os.path.join(pkg_robot_description, 'worlds', 'my_world.sdf')
    config_file = os.path.join(pkg_robot_description, 'config', 'ros2_control.yaml')

    # Ensure files exist
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found at {xacro_file}")
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Controller config not found at {config_file}")

    # Convert Xacro to URDF and enable ros2_control section
    robot_description_config = xacro.process_file(xacro_file, mappings={
        'gazebo_control': 'true'
    })
    robot_description = robot_description_config.toxml()

    # Write processed URDF to a temporary file for Gazebo to consume
    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    tmp_urdf.write(robot_description)
    tmp_urdf_path = tmp_urdf.name
    tmp_urdf.close()

    # Declare launch arguments
    model_arg = DeclareLaunchArgument('model', default_value=tmp_urdf_path, description='Path to processed URDF file')
    world_arg = DeclareLaunchArgument('world', default_value=world_file, description='Path to Gazebo world file')

    # Start Gazebo Sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items()
    )

    # Robot State Publisher (now using converted URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Spawn the robot in Gazebo using the processed URDF file
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'pick_place_arm', '-file', LaunchConfiguration('model'), '-x', '-3', '-y', '0', '-z', '0', '-R', '0', '-P', '0', '-Y', '1.57'],
        output='screen'
    )

    # Function to load controllers
    def load_controller(name, activate=True):
        cmd = ['ros2', 'control', 'load_controller', '--controller-manager', '/controller_manager', name]
        if activate:
            cmd.extend(['--set-state', 'active'])
        return ExecuteProcess(cmd=cmd, output='screen')

    # Controller loading sequence
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                        load_controller('joint_state_broadcaster'),
                        TimerAction(
                            period=5.0,
                            actions=[
                                load_controller('arm_controller'),
                                TimerAction(
                                    period=2.0,
                                    actions=[load_controller('gripper_controller')]
                                )
                            ]
                        )
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        load_controllers
    ])
