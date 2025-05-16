
//
#include "bridge_handle.hpp"  //DONE!!

#include <memory>
#include <string>

#include "get_factory.hpp"  //DONE!!

namespace core_ros_gz_service_bridge
{
BridgeHandle::BridgeHandle(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<gz::transport::Node> gz_node,
  const BridgeConfig & config)
: ros_node_(ros_node),
  gz_node_(gz_node),
  config_(config),
  factory_(get_factory(config.ros_type_name, config.gz_type_name))
{
}

BridgeHandle::~BridgeHandle() = default;

bool BridgeHandle::IsLazy() const
{
  return config_.is_lazy;
}

void BridgeHandle::Start()
{
  if (!this->HasPublisher()) {
    this->StartPublisher();
  }

  if (!this->IsLazy() && !this->HasSubscriber()) {
    this->StartSubscriber();
  }
}

void BridgeHandle::Spin()
{
  if (!this->IsLazy()) {
    return;
  }

  if (this->HasSubscriber() && this->NumSubscriptions() == 0) {
    RCLCPP_DEBUG(
      this->ros_node_->get_logger(),
      "Bridge [%s] - No subscriptions found, stopping bridge",
      config_.ros_topic_name.c_str());
    this->StopSubscriber();
  } else if (!this->HasSubscriber() && this->NumSubscriptions() > 0) {
    RCLCPP_DEBUG(
      this->ros_node_->get_logger(),
      "Bridge [%s] - Subscriptions found, starting bridge",
      config_.ros_topic_name.c_str());
    this->StartSubscriber();
  }
}
}  // namespace core_ros_gz_service_bridge
