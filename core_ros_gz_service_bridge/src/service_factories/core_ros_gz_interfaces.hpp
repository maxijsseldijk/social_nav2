//DONE!!!
#ifndef SERVICE_FACTORIES__CORE_ROS_GZ_SERVICE_INTERFACES_HPP_
#define SERVICE_FACTORIES__CORE_ROS_GZ_SERVICE_INTERFACES_HPP_

#include <memory>
#include <string>

#include "service_factory_interface.hpp"  //DONE!!

namespace core_ros_gz_service_bridge
{

std::shared_ptr<ServiceFactoryInterface>
get_service_factory__ros_gz_interfaces(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name);

}  // namespace core_ros_gz_service_bridge

#endif  // SERVICE_FACTORIES__CORE_ROS_GZ_SERVICE_INTERFACES_HPP_
