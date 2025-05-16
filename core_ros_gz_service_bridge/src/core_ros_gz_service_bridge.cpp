//DONE!!

#include <core_ros_gz_service_bridge/core_ros_gz_service_bridge.hpp>  //DONE!!

#include <memory>
#include <string>

// #include "bridge_handle_ros_to_gz.hpp"   //DONE!!
// #include "bridge_handle_gz_to_ros.hpp"  //DONE!!


#include "bridge_handle.hpp"

namespace core_ros_gz_service_bridge
{

RosGzBridge::RosGzBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("core_ros_gz_service_bridge", options)
{
  gz_node_ = std::make_shared<gz::transport::Node>();

  this->declare_parameter<int>("subscription_heartbeat", 1000);
  this->declare_parameter<std::string>("config_file", "");

  int heartbeat;
  this->get_parameter("subscription_heartbeat", heartbeat);
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(heartbeat),
    std::bind(&RosGzBridge::spin, this));
}

void RosGzBridge::spin()
{
  for (auto & bridge : handles_) {
    bridge->Spin();
  }
}



void RosGzBridge::add_service_bridge(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name,
  const std::string & service_name)
{

  auto factory = get_service_factory(ros_type_name, gz_req_type_name, gz_rep_type_name);
  services_.push_back(factory->create_ros_service(shared_from_this(), gz_node_, service_name));
}

}  // namespace core_ros_gz_service_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(core_ros_gz_service_bridge::RosGzBridge)
