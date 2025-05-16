
//DONE!!
#ifndef CONVERT__UTILS_HPP_
#define CONVERT__UTILS_HPP_

#include <string>

namespace core_ros_gz_service_bridge
{

// This can be used to replace `::` with `/` to make frame_id compatible with TF
std::string replace_delimiter(
  const std::string & input,
  const std::string & old_delim,
  const std::string new_delim);

std::string frame_id_gz_to_ros(const std::string & frame_id);

}  // namespace core_ros_gz_service_bridge

#endif  // CONVERT__UTILS_HPP_
