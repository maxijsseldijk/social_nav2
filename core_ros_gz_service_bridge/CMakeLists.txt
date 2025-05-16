cmake_minimum_required(VERSION 3.8)
project(core_ros_gz_service_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(std_msgs REQUIRED)



# find Gazebo Fortress dependencies
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

set(GZ_TARGET_PREFIX ignition)
set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

message(STATUS "Compiling against Ignition Fortress")


set(GZ_MSGS_VERSION_MAJOR ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_MAJOR})
set(GZ_MSGS_VERSION_MINOR ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_MINOR})
set(GZ_MSGS_VERSION_PATCH ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_PATCH})
set(GZ_MSGS_VERSION_FULL ${GZ_MSGS_VERSION_MAJOR}.${GZ_MSGS_VERSION_MINOR}.${GZ_MSGS_VERSION_PATCH})


#Setting message types
set(BRIDGE_MESSAGE_TYPES
  builtin_interfaces
  geometry_msgs
  std_msgs
  rcl_interfaces
  ros_gz_interfaces
)


set(generated_path "${CMAKE_BINARY_DIR}/generated")
set(generated_files "${generated_path}/get_factory.cpp")
list(APPEND generated_files "${generated_path}/get_mappings.cpp")
set(convert_files "src/convert/utils.cpp")

foreach(package_name ${BRIDGE_MESSAGE_TYPES})
  find_package(${package_name} QUIET REQUIRED)
  message(STATUS "Found ${package_name}: ${${package_name}_VERSION} (${${package_name}_DIR})")

  list(APPEND generated_files "${generated_path}/factories/${package_name}.cpp")
  list(APPEND generated_files "${generated_path}/factories/${package_name}.hpp")

  list(APPEND convert_files "src/convert/${package_name}.cpp")
endforeach()

set(target_dependencies
  "bin/core_ros_gz_service_bridge_generate_factories"
  "resource/get_factory.cpp.em"
  "resource/get_mappings.cpp.em"
  "resource/pkg_factories.cpp.em"
  "resource/pkg_factories.hpp.em"
  "core_ros_gz_service_bridge/__init__.py"
  "core_ros_gz_service_bridge/mappings.py")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND Python3::Interpreter
  ARGS bin/core_ros_gz_service_bridge_generate_factories
    --output-path "${generated_path}" --template-dir resource
    --gz-msgs-ver "${GZ_MSGS_VERSION_FULL}"
  DEPENDS ${target_dependencies}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating factories for service interface types")

set(service_bridge_lib
  core_ros_gz_service_bridge_lib
)

add_library(${service_bridge_lib}
  SHARED
  src/bridge_config.cpp
  src/bridge_handle.cpp
  src/factory_interface.cpp
  src/service_factories/core_ros_gz_interfaces.cpp
  src/core_ros_gz_service_bridge.cpp
  ${convert_files}
  ${generated_files}
)

target_link_libraries(${service_bridge_lib}
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
)

ament_target_dependencies(${service_bridge_lib}
  rclcpp
  rclcpp_components
  yaml_cpp_vendor
  ros_gz_interfaces
  std_msgs
  ${BRIDGE_MESSAGE_TYPES}
)

target_include_directories(${service_bridge_lib}
  PUBLIC include
  PRIVATE src ${generated_path}
)

target_link_libraries(${service_bridge_lib}
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
)

rclcpp_components_register_node(
  ${service_bridge_lib}
  PLUGIN core_ros_gz_service_bridge::RosGzBridge
  EXECUTABLE bridge_node)

install(TARGETS ${service_bridge_lib}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${service_bridge_lib})

set(service_bridge_executables
  service_bridge
)

foreach(bridge ${service_bridge_executables})
  add_executable(${bridge}
    src/${bridge}.cpp
  )
  target_link_libraries(${bridge}
    ${service_bridge_lib}
  )
  ament_target_dependencies(${bridge}
    "rclcpp"
    ${BRIDGE_MESSAGE_TYPES}
  )
  install(TARGETS ${bridge}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_export_dependencies(
  rclcpp
  ${BRIDGE_MESSAGE_TYPES}
)

# Install launch files
install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
