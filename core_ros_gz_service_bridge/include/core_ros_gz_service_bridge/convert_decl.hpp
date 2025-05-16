

//DONE!!

#ifndef CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT_DECL_HPP_
#define CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT_DECL_HPP_

namespace core_ros_gz_service_bridge
{

template<typename ROS_T, typename GZ_T>
void
convert_ros_to_gz(
  const ROS_T & ros_msg,
  GZ_T & gz_msg);

template<typename ROS_T, typename GZ_T>
void
convert_gz_to_ros(
  const GZ_T & gz_msg,
  ROS_T & ros_msg);

}  // namespace core_ros_gz_service_bridge

#endif  // CORE_ROS_GZ_SERVICE_BRIDGE__CONVERT_DECL_HPP_
