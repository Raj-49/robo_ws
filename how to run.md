cd robo_ws/
colcon build --packages-select pick_place_arm arm_moveit_config --parallel-workers 1
source install/setup.bash
ros2 launch arm_moveit_config unified_gz_moveit.launch.py