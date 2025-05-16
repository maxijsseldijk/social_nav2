cmake_minimum_required(VERSION 3.8)
project(core_nav2_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rmw_cyclonedds_cpp REQUIRED)
find_package(core_custom_messages REQUIRED)
find_package(nav2_common REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(dwb_core REQUIRED)
find_package(social_force_window_planner REQUIRED)
find_package(rl_pure_pursuit_controller REQUIRED)


# Include Cpp "include" directory
include_directories(include
${dwb_core_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclpy
  ros_gz_bridge
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  rclcpp_action
  rclcpp_lifecycle
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  builtin_interfaces
  nav2_costmap_2d
  nav2_core
  pluginlib
  rmw_cyclonedds_cpp
  core_custom_messages
  nav2_common
  nav_2d_msgs
  dwb_msgs
  dwb_core
  social_force_window_planner
  rl_pure_pursuit_controller
)

set(library_name nav2_rl_local_planner)


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


add_library(${library_name} SHARED
  src/rl_local_planner.cpp
  src/resample_path.cpp

)


target_link_libraries(${library_name} ${dwb_core_LIBRARIES})
target_include_directories(${library_name} PUBLIC ${dwb_core_INCLUDE_DIRS} )

ament_target_dependencies(${library_name}
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core local_planner_plugin.xml)
# Install Cpp executables
install(TARGETS
  ${library_name}
  DESTINATION lib/${PROJECT_NAME})
# Install Python modules
ament_python_install_package(${PROJECT_NAME})


install(DIRECTORY include/
  DESTINATION include/
)

install(FILES  local_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
# Install Python executables
install(PROGRAMS
  scripts/follow_waypoints.py
  scripts/follow_single_waypoint.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install package folder
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(include)
ament_export_libraries(${library_name} )
ament_package()
