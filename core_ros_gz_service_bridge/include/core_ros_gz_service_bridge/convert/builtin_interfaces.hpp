//DONE!!

#ifndef CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT__BUILTIN_INTERFACES_HPP_
#define CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT__BUILTIN_INTERFACES_HPP_

#include <gz/msgs/time.pb.h>

#include <builtin_interfaces/msg/time.hpp>

#include "core_ros_gz_service_bridge/convert_decl.hpp"  //DONE!!

namespace core_ros_gz_service_bridge
{

template<>
void
convert_ros_to_gz(
  const builtin_interfaces::msg::Time & ros_msg,
  gz::msgs::Time & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Time & gz_msg,
  builtin_interfaces::msg::Time & ros_msg);

}  // namespace core_ros_gz_service_bridge

#endif  // CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT__BUILTIN_INTERFACES_HPP_
