import os
import shutil
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build MoveIt config (URDF/SRDF from arm_moveit_config)-----------------------------------------------------------------------------------------
   
    moveit_config = (
        MoveItConfigsBuilder("pick_place_arm", package_name="arm_moveit_config")
        .robot_description(file_path="config/pick_place_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/pick_place_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    # Paths------------------------------------------------------------------------------------------------------------------------------------------
    
    pick_place_share = get_package_share_directory('pick_place_arm')
    gz_share = get_package_share_directory('ros_gz_sim')
    world_default = PathJoinSubstitution([pick_place_share, 'worlds', 'my_world.sdf'])
    gazebo_launch_file = os.path.join(gz_share, 'launch', 'gz_sim.launch.py')

    # Launch args------------------------------------------------------------------------------------------------------------------------------------
    
    world_arg = DeclareLaunchArgument('world', default_value=world_default, description='Gazebo world SDF path/URI')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z')
    start_rviz_arg = DeclareLaunchArgument('start_rviz', default_value='true', description='Start RViz')
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Headless RViz via xvfb (SSH/no DISPLAY)')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use ROS simulated time (/clock) for nodes')

    # Use source xacro file in workspace src to generate an unscaled URDF file for Gazebo spawn------------------------------------------------------
    
    # Get workspace root dynamically (works on any system)
    pick_place_pkg_path = get_package_share_directory('pick_place_arm')
    # Navigate up from install/pick_place_arm/share/pick_place_arm to workspace root
    workspace_root = os.path.abspath(os.path.join(pick_place_pkg_path, '..', '..', '..', '..'))
    arm_xacro_src = os.path.join(workspace_root, 'src', 'pick_place_arm', 'urdf', 'arm.urdf.xacro')
    unscaled_urdf_path = '/tmp/pick_place_arm.urdf'
    try:
        unscaled_urdf = subprocess.check_output(['xacro', arm_xacro_src, 'SCALE:=1', 'BASE_YAW:=0']).decode('utf-8')
        with open(unscaled_urdf_path, 'w') as f:
            f.write(unscaled_urdf)
    except Exception as e:
        print('Warning: failed to generate unscaled URDF from src xacro:', e)
    
    # Set up Gazebo resource path to include our models directory------------------------------------------------------------------------------------
    
    models_path = os.path.join(pick_place_share, 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if gz_resource_path:
        gz_resource_path = f"{models_path}:{gz_resource_path}"
    else:
        gz_resource_path = models_path
    
    # Gazebo Sim (GZ/Ignition) Launch ---------------------------------------------------------------------------------------------------------------
    
    gazebo_env = {'GZ_SIM_RESOURCE_PATH': gz_resource_path}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items(),
    )

    # Bridge Gazebo clock to ROS /clock (world in my_world.sdf is named 'empty')
    # Use a single parameter_bridge instance and provide both ignition and gz message type variants--------------------------------------------------

    gz_world_clock = '/world/empty/clock'
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge_clock',
        arguments=[
            '-l', f'{gz_world_clock}@rosgraph_msgs/msg/Clock@ignition.msgs.Clock', 
            '-l', f'{gz_world_clock}@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        remappings=[(gz_world_clock, '/clock')], output='screen')
    
    # Bridge box contact topics from Gazebo to ROS2 (only existing models: red_box, green_box, blue_box)
    # Provide bidirectional mappings for both ignition.msgs and gz.msgs variants-----------------------------------------------------------------

    contact_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge_contacts',
        arguments=[
            # Red box contacts
            '-l', '/red_box/contacts@ros_gz_interfaces/msg/Contacts@ignition.msgs.Contacts',
            '-l', '/red_box/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
            # Green box contacts
            '-l', '/green_box/contacts@ros_gz_interfaces/msg/Contacts@ignition.msgs.Contacts',
            '-l', '/green_box/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
            # Blue box contacts
            '-l', '/blue_box/contacts@ros_gz_interfaces/msg/Contacts@ignition.msgs.Contacts',
            '-l', '/blue_box/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
        ],
        output='screen')
    
    # Bridge camera and attachment topics from Gazebo to ROS2 (perception + high-level manipulation control)-----------------------------------------
    
    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Colored box attachment topics
            '/red_box/attach@std_msgs/msg/Empty@gz.msgs.Empty',
            '/red_box/detach@std_msgs/msg/Empty@gz.msgs.Empty',
            '/green_box/attach@std_msgs/msg/Empty@gz.msgs.Empty',
            '/green_box/detach@std_msgs/msg/Empty@gz.msgs.Empty',
            '/blue_box/attach@std_msgs/msg/Empty@gz.msgs.Empty',
            '/blue_box/detach@std_msgs/msg/Empty@gz.msgs.Empty',
            # Bridge all model poses to TF message type (but publish to separate topic to avoid spam)
            '/world/empty/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/empty/pose/info', '/model_poses'),
        ],
        output='screen')
    
    # Robot state publisher with selectable sim time-------------------------------------------------------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )

    # Spawn robot into Gazebo from a pre-generated URDF file (guarantees create sees the full model)-------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', unscaled_urdf_path,
            '-name', 'pick_place_arm',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen',
    )

    # Start the controller spawner script from the installed package after a short delay-------------------------------------------------------------

    spawn_controllers_script = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(cmd=['python3', os.path.join(get_package_share_directory('pick_place_arm'), 'scripts', 'wait_and_spawn_controllers.py')], output='screen')
        ]
    )

    # Initial detach (clear pre-attachment silently) ------------------------------------------------------------------------------------------------

    initial_detach_runner = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(cmd=['python3', os.path.join(get_package_share_directory('pick_place_arm'), 'scripts', 'initial_detach.py')], output='screen')
        ]
    )

    # MoveIt move_group load all the moveit_config with selectable sim time  ------------------------------------------------------------------------

    cfg = moveit_config.to_dict()
    cfg.update({"use_sim_time": LaunchConfiguration('use_sim_time')})
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[cfg],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )

    # RViz setup: robust GUI/headless ---------------------------------------------------------------------------------------------------------------

    rviz_config_path = os.path.join(get_package_share_directory('arm_moveit_config'), 'config', 'moveit.rviz')
    safe_ld = ":".join(filter(None, [
        os.environ.get('LD_LIBRARY_PATH', ''),
        '/opt/ros/humble/lib',
        '/usr/lib/x86_64-linux-gnu',
        '/usr/lib',
    ]))
    base_env = {
        'LD_PRELOAD': '',
        'SNAP': '',
        'LD_LIBRARY_PATH': safe_ld,
        'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH', ''),
        'COLCON_PREFIX_PATH': os.environ.get('COLCON_PREFIX_PATH', ''),
    }
    has_xvfb = shutil.which('xvfb-run') is not None


    rviz_gui = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": LaunchConfiguration('use_sim_time')},
        ],
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    rviz_headless = None
    if has_xvfb:
        headless_env = dict(base_env)
        headless_env['ROS_LOG_DIR'] = '/tmp/roslogs'
        headless_env['RCUTILS_LOGGING_DIRECTORY'] = '/tmp/roslogs'
        headless_env['RCUTILS_LOGGING_USE_STDOUT'] = '1'
        headless_env['HOME'] = os.environ.get('HOME', '/tmp')
        rviz_headless = Node(
            package='rviz2', executable='rviz2', name='rviz2_headless', output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": LaunchConfiguration('use_sim_time')},
            ],
            env=headless_env,
            prefix=['xvfb-run', '-a', '-s', '"-screen 0 1280x1024x24"'],
            condition=IfCondition(LaunchConfiguration('headless')),
        )
    rviz_actions = [rviz_gui]
    if rviz_headless:
        rviz_actions.append(rviz_headless)
    rviz_group = GroupAction(actions=rviz_actions, condition=IfCondition(LaunchConfiguration('start_rviz')))
    
    # Set environment variable for Gazebo to find models --------------------------------------------------------------------------------------------

    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)
    
    return LaunchDescription([
        # Set environment first
        set_gz_resource_path,

        # args
        world_arg, x_arg, y_arg, z_arg, start_rviz_arg, headless_arg, use_sim_time_arg,

        # sim + robot
        gazebo,
        clock_bridge,
        contact_bridge,
        camera_bridge,
        robot_state_publisher,
        spawn_robot,

        # Spawn controllers after delay
        spawn_controllers_script,

        # Initial detach to clear any pre-existing attachments
        initial_detach_runner,
        
        # RViz Interactive Control (Visualization & Attach/Detach)
        ExecuteProcess(
            cmd=['python3', os.path.join(get_package_share_directory('pick_place_arm'), 'scripts', 'rviz_interactive_control.py')],
            output='screen'
        ),
        # TF Relay (Fixes Gazebo->TF spam)
        ExecuteProcess(
            cmd=['python3', os.path.join(get_package_share_directory('pick_place_arm'), 'scripts', 'pose_to_tf_relay.py')],
            output='screen'
        ),
        # moveit
        move_group,
        # rviz (optional)
        rviz_group,
    ])
