cmake_minimum_required(VERSION 3.8)
project(pick_place_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Directories
install(
  DIRECTORY launch meshes urdf config worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts so launch files can find and execute them from the install space
# Install helper scripts for launch files (share/scripts)
install(
  PROGRAMS
    scripts/wait_and_spawn_controllers.py
    scripts/controller_starter.py
    scripts/contact_monitor.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install executables/nodes (lib/package_name)
install(
  PROGRAMS
    scripts/pick_and_place.py
    scripts/pick_and_place_simple.py
    scripts/pick_and_place_moveit.py
    scripts/pick_and_place_cartesian.py
    scripts/demo_pick_place.py
    scripts/gripper_attachment_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
