
//DONE!!

#ifndef GET_FACTORY_HPP_
#define GET_FACTORY_HPP_

#include <memory>
#include <string>

#include "factory_interface.hpp"   //DONE!!
#include "service_factory_interface.hpp" //DONE!!

namespace core_ros_gz_service_bridge
{

std::shared_ptr<FactoryInterface>
get_factory(
  const std::string & ros_type_name,
  const std::string & gz_type_name);

std::shared_ptr<ServiceFactoryInterface>
get_service_factory(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name);

}  // namespace core_ros_gz_service_bridge

#endif  // GET_FACTORY_HPP_
