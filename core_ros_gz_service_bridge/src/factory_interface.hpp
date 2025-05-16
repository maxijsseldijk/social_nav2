//DONE!!
#ifndef  FACTORY_INTERFACE_HPP_
#define  FACTORY_INTERFACE_HPP_

#include <memory>
#include <string>

// include Gazebo Transport
#include <gz/transport/Node.hh>

// include ROS 2
#include <rclcpp/rclcpp.hpp>

namespace core_ros_gz_service_bridge
{

class FactoryInterface
{
public:
  virtual ~FactoryInterface() = 0;

  virtual
  rclcpp::PublisherBase::SharedPtr
  create_ros_publisher(
    rclcpp::Node::SharedPtr ros_node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  gz::transport::Node::Publisher
  create_gz_publisher(
    std::shared_ptr<gz::transport::Node> gz_node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  rclcpp::SubscriptionBase::SharedPtr
  create_ros_subscriber(
    rclcpp::Node::SharedPtr ros_node,
    const std::string & topic_name,
    size_t queue_size,
    gz::transport::Node::Publisher & gz_pub) = 0;

  virtual
  void
  create_gz_subscriber(
    std::shared_ptr<gz::transport::Node> node,
    const std::string & topic_name,
    size_t queue_size,
    rclcpp::PublisherBase::SharedPtr ros_pub) = 0;
};

}  // namespace core_ros_gz_service_bridge

#endif  // FACTORY_INTERFACE_HPP_
