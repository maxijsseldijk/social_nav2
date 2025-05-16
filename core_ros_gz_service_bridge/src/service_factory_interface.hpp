//DONE!!!
#ifndef  SERVICE_FACTORY_INTERFACE_HPP_
#define  SERVICE_FACTORY_INTERFACE_HPP_

#include <memory>
#include <string>

#include <gz/transport/Node.hh>

#include <rclcpp/service.hpp>
#include <rclcpp/node.hpp>

namespace core_ros_gz_service_bridge
{

class ServiceFactoryInterface
{
public:
  virtual
  rclcpp::ServiceBase::SharedPtr
  create_ros_service(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<gz::transport::Node> gz_node,
    const std::string & service_name) = 0;
};

}  // namespace core_ros_gz_service_bridge

#endif  // SERVICE_FACTORY_INTERFACE_HPP_
