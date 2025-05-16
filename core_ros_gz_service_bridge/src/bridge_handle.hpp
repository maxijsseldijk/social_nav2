//DONE!!

#ifndef BRIDGE_HANDLE_HPP_
#define BRIDGE_HANDLE_HPP_

#include <memory>
#include <string>

#include <gz/transport/Node.hh>
#include <rclcpp/node.hpp>

#include "get_factory.hpp"  //DONE!!
#include <core_ros_gz_service_bridge/bridge_config.hpp>  //DONE!!


namespace core_ros_gz_service_bridge
{

/// \brief Core functionality and data for both bridge directions
class BridgeHandle
{
public:
  /// \brief Constructor
  /// Note, does not actually start the bridge, must be done with Start()
  ///
  /// \param[in] ros_node ROS node to create publishers/subscribers on
  /// \param[in] gz_node GZ node to create publishers/subscribers on
  /// \param[in] config Configuration parameters for this handle
  BridgeHandle(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<gz::transport::Node> gz_node,
    const BridgeConfig & config);

  /// \brief Destructor
  virtual ~BridgeHandle() = 0;

  /// \brief Initiate the bridge
  ///
  /// If the bridge is "lazy", then only the publisher (output) side is created.
  /// The not, both the publisher (output) and subscriber (input) side are
  /// created.
  void Start();

  /// \brief Spin the bridge, checking for new subscriptions on the output.
  ///
  /// Not necessary if the Bridge isn't lazy
  void Spin();

  /// \brief Inidicate if this is a "lazy" bridge
  ///
  /// A lazy bridge will always create the output side of the bridge, but
  /// will only create the input side of the bridge when downstream consumers
  /// are detected on the output side.
  /// This reduces the amount of overhead in the bridge processing messages
  /// for non-existent consumers.
  /// It will also allow for lazy publishers to work on the input side of the
  /// bridge.
  /// \return true if lazy
  bool IsLazy() const;

protected:
  /// \brief Get the number of subscriptions on the output side of the bridge
  virtual size_t NumSubscriptions() const = 0;

  /// \brief Indicate if the publisher (output) side has been created
  virtual bool HasPublisher() const = 0;

  /// \brief Start the publisher (output) side of the bridge
  virtual void StartPublisher() = 0;

  /// \brief Indicate if the subscriber (input) side has been created
  virtual bool HasSubscriber() const = 0;

  /// \brief Start the subscriber (input) side of the bridge
  virtual void StartSubscriber() = 0;

  /// \brief Stop the subscriber (input) side of the bridge
  virtual void StopSubscriber() = 0;

protected:
  /// \brief The ROS node used to create publishers/subscriptions
  rclcpp::Node::SharedPtr ros_node_;

  /// \brief The Gazebo node used to create publishers/subscriptions
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief The configuration parameters of this bridge
  BridgeConfig config_;

  /// \brief Typed factory used to create publishers/subscribers
  std::shared_ptr<FactoryInterface> factory_;
};

}  // namespace core_ros_gz_service_bridge
#endif  // BRIDGE_HANDLE_HPP_
