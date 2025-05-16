
//DONE!!
#include "core_ros_gz_service_bridge/convert/builtin_interfaces.hpp" //DONE!!

namespace core_ros_gz_service_bridge
{
template<>
void
convert_ros_to_gz(
  const builtin_interfaces::msg::Time & ros_msg,
  gz::msgs::Time & gz_msg)
{
  gz_msg.set_sec(ros_msg.sec);
  gz_msg.set_nsec(ros_msg.nanosec);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Time & gz_msg,
  builtin_interfaces::msg::Time & ros_msg)
{
  ros_msg.sec = gz_msg.sec();
  ros_msg.nanosec = gz_msg.nsec();
}
}  // namespace core_ros_gz_service_bridge
