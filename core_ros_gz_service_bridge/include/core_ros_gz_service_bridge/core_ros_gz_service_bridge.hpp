//DONE!!!
#ifndef CORE_ROS_GZ_SERVICE_BRIDGE__ROS_GZ_BRIDGE_HPP_
#define CORE_ROS_GZ_SERVICE_BRIDGE__ROS_GZ_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/config.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/node.hpp>
#include "core_ros_gz_service_bridge/bridge_config.hpp"  //DONE

// Dataframe is available from versions 8.4.0 (fortress) forward
// This can be removed when the minimum supported version passes 8.4.0
#if (IGNITION_MSGS_MAJOR_VERSION > 8) || \
  ((IGNITION_MSGS_MAJOR_VERSION == 8) && (IGNITION_MSGS_MINOR_VERSION >= 4))
#define HAVE_DATAFRAME true
#endif

#if (GZ_MSGS_MAJOR_VERSION > 8) || \
  ((GZ_MSGS_MAJOR_VERSION == 8) && (GZ_MSGS_MINOR_VERSION >= 4))
#define HAVE_DATAFRAME true
#endif

namespace core_ros_gz_service_bridge
{
/// Forward declarations
class BridgeHandle;

/// \brief Component container for the ROS-GZ Bridge
class RosGzBridge : public rclcpp::Node
{
public:
  /// \brief Constructor
  /// \param[in] options options control creation of the ROS 2 node
  explicit RosGzBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());


  /// \brief Create a new ROS-GZ bridge for a service
  /// \param[in] ros_type_name Name of the ROS service (eg core_ros_gz_service_interfaces/srv/DeleteEntity)
  /// \param[in] gz_req_type_name Gazebo service request type
  /// \param[in] gz_req_type_name Gazebo service response type
  /// \param[in] service_name Address of the service to be bridged
  void add_service_bridge(
    const std::string & ros_type_name,
    const std::string & gz_req_type_name,
    const std::string & gz_rep_type_name,
    const std::string & service_name);

protected:
  /// \brief Periodic callback to check connectivity and liveliness
  void spin();

protected:
  /// \brief Pointer to Gazebo node used to create publishers/subscribers
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief List of bridge handles
  std::vector<std::shared_ptr<core_ros_gz_service_bridge::BridgeHandle>> handles_;

  /// \brief List of bridged ROS services
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  /// \brief Timer to control periodic callback
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};
}  // namespace core_ros_gz_service_bridge

#endif  // CORE_ROS_GZ_SERVICE_BRIDGE__ROS_GZ_BRIDGE_HPP_
