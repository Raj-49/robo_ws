import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get package paths
    pick_place_hardware_share = get_package_share_directory('pick_place_hardware')
    arm_moveit_share = get_package_share_directory('arm_moveit_config')
    
    # Build MoveIt config with xacro mappings to enable fake hardware
    moveit_config = (
        MoveItConfigsBuilder("pick_place_arm", package_name="arm_moveit_config")
        .robot_description(
            file_path="config/pick_place_arm.urdf.xacro",
            mappings={
                "use_fake_hardware": "true",
                "gazebo_control": "false"  # Disable Gazebo ros2_control block
            }
        )
        .robot_description_semantic(file_path="config/pick_place_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='Use simulated time (false for fake hardware)'
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz', 
        default_value='true', 
        description='Start RViz'
    )

    # Robot State Publisher - CRITICAL for RViz visualization
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
        output="screen",
    )

    # Controller Manager with robot_description and ros2_control config
    ros2_control_config = os.path.join(pick_place_hardware_share, 'config', 'ros2_control.yaml')
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_control_config,
            {"use_sim_time": LaunchConfiguration('use_sim_time')}
        ],
        output="screen",
    )

    # Spawn Controllers
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # MoveIt Move Group
    cfg = moveit_config.to_dict()
    cfg.update({"use_sim_time": LaunchConfiguration('use_sim_time')})
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[cfg],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )

    # RViz
    rviz_config_path = os.path.join(arm_moveit_share, 'config', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        robot_state_publisher,
        controller_manager,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,
        move_group,
        rviz,
    ])
