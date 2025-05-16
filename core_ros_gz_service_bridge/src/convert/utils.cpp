//DONE!!

#include "convert/utils.hpp"  //DONE!!

#include <string>

namespace core_ros_gz_service_bridge
{

// This can be used to replace `::` with `/` to make frame_id compatible with TF
std::string replace_delimiter(
  const std::string & input,
  const std::string & old_delim,
  const std::string new_delim)
{
  std::string output;
  output.reserve(input.size());

  std::size_t last_pos = 0;

  while (last_pos < input.size()) {
    std::size_t pos = input.find(old_delim, last_pos);
    output += input.substr(last_pos, pos - last_pos);
    if (pos != std::string::npos) {
      output += new_delim;
      pos += old_delim.size();
    }
    last_pos = pos;
  }

  return output;
}


std::string frame_id_gz_to_ros(const std::string & frame_id)
{
  return replace_delimiter(frame_id, "::", "/");
}

}  // namespace core_ros_gz_service_bridge
