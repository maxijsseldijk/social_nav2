

#ifndef GET_MAPPINGS_HPP_
#define GET_MAPPINGS_HPP_

#include <map>
#include <string>

namespace core_ros_gz_service_bridge
{

bool
get_gz_to_ros_mapping(const std::string & gz_type_name, std::string & ros_type_name);

bool
get_ros_to_gz_mapping(const std::string & ros_type_name, std::string & gz_type_name);

std::multimap<std::string, std::string>
get_all_message_mappings_ros_to_gz();

}  // namespace core_ros_gz_service_bridge

#endif  // GET_MAPPINGS_HPP_
