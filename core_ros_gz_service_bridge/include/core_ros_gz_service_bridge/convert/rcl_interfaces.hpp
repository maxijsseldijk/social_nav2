//DONE!!

#ifndef CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT__RCL_INTERFACES_HPP_
#define CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT__RCL_INTERFACES_HPP_

// Ignition messages
#include <gz/msgs/any.pb.h>

// ROS 2 messages
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

#include <core_ros_gz_service_bridge/convert_decl.hpp>  //DONE!!

namespace core_ros_gz_service_bridge
{

template<>
void
convert_ros_to_gz(
  const rcl_interfaces::msg::ParameterValue & ros_msg,
  gz::msgs::Any & ign_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Any & ign_msg,
  rcl_interfaces::msg::ParameterValue & ros_msg);

}  // namespace core_ros_gz_service_bridge
#endif  // CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT__RCL_INTERFACES_HPP_
