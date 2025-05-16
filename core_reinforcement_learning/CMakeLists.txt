cmake_minimum_required(VERSION 3.8)
project(core_reinforcement_learning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmw_cyclonedds_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(core_custom_messages REQUIRED)
find_package(people_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/train_rl.py
  scripts/distance_to_undisturbed_path_node.py
  scripts/world_interact_node.py
  scripts/construct_safe_corridor_node.py
  scripts/publish_agents_velocity.py
  scripts/rl_node_manager.py
  scripts/ros_gazebo_env.py
  scripts/imitation_learning_utils.py
  scripts/model_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install package folder
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
    # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_ros_gazebo_env test/test_ros_gazebo_env.py ENV ${TEST_ENV})
  ament_add_pytest_test(test_pep257 test/test_pep257.py)
endif()

ament_package()
