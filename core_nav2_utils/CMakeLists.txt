cmake_minimum_required(VERSION 3.8)
project(core_nav2_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(core_custom_messages REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)


set(dependencies
  rclcpp
  nav2_util
  nav_msgs
  core_custom_messages

)
include_directories(
  include
)

set(library_name core_nav2_utils)

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
  src/resample_path.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)
install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)

endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} )
ament_package()