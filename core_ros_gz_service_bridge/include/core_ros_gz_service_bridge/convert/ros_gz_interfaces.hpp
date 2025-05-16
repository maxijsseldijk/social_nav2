// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CORE_ROS_SERVICE_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_
#define CORE_ROS_SERVICE_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_

// Gazebo Msgs
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>

// ROS 2 messages
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/msg/entity_factory.hpp>

// Required for HAVE_DATAFRAME definition
#include <core_ros_gz_service_bridge/core_ros_gz_service_bridge.hpp>

#if HAVE_DATAFRAME
#include <gz/msgs/dataframe.pb.h>
#include <ros_gz_interfaces/msg/dataframe.hpp>
#endif  // HAVE_DATAFRAME 

#include <core_ros_gz_service_bridge/convert_decl.hpp>

namespace core_ros_gz_service_bridge
{


template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  gz::msgs::Entity & gz_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Entity & gz_msg,
  ros_gz_interfaces::msg::Entity & ros_msg);


template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::EntityFactory & ros_msg,
  gz::msgs::EntityFactory & gz_msg);


template<>
void
convert_gz_to_ros(
  const gz::msgs::EntityFactory & gz_msg,
  ros_gz_interfaces::msg::EntityFactory & ros_msg);


#if HAVE_DATAFRAME
template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Dataframe & ros_msg,
  gz::msgs::Dataframe & ign_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Dataframe & ign_msg,
  ros_gz_interfaces::msg::Dataframe & ros_msg);
#endif  // HAVE_DATAFRAME

}  // namespace core_ros_gz_service_bridge

#endif  // CORE_ROS_SERVICE_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_
