//DONE!!

#ifndef CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT__STD_MSGS_HPP_
#define CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT__STD_MSGS_HPP_

// Gazebo Msgs
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/color.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/uint32.pb.h>
#include <gz/msgs/stringmsg.pb.h>

// ROS 2 messages
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <core_ros_gz_service_bridge/convert_decl.hpp>  //DONE!!

namespace core_ros_gz_service_bridge
{

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Bool & ros_msg,
  gz::msgs::Boolean & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_msg,
  std_msgs::msg::Bool & ros_msg);

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::ColorRGBA & ros_msg,
  gz::msgs::Color & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Color & gz_msg,
  std_msgs::msg::ColorRGBA & ros_msg);

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Empty & ros_msg,
  gz::msgs::Empty & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Empty & gz_msg,
  std_msgs::msg::Empty & ros_msg);

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::UInt32 & ros_msg,
  gz::msgs::UInt32 & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::UInt32 & gz_msg,
  std_msgs::msg::UInt32 & ros_msg);

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Float32 & ros_msg,
  gz::msgs::Float & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Float & gz_msg,
  std_msgs::msg::Float32 & ros_msg);

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Float64 & ros_msg,
  gz::msgs::Double & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Double & gz_msg,
  std_msgs::msg::Float64 & ros_msg);

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Header & ros_msg,
  gz::msgs::Header & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Header & gz_msg,
  std_msgs::msg::Header & ros_msg);

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::Int32 & ros_msg,
  gz::msgs::Int32 & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Int32 & gz_msg,
  std_msgs::msg::Int32 & ros_msg);

template<>
void
convert_ros_to_gz(
  const std_msgs::msg::String & ros_msg,
  gz::msgs::StringMsg & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::StringMsg & gz_msg,
  std_msgs::msg::String & ros_msg);

}  // namespace core_ros_gz_service_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__STD_MSGS_HPP_
