#ifndef CORE_NAV2_NAVIGATION__RESAMPLE_PATH_HPP_
#define CORE_NAV2_NAVIGATION__RESAMPLE_PATH_HPP_
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "core_custom_messages/msg/path_with_length.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_rl
{
class ResamplePath
{
public:
  explicit ResamplePath(rclcpp::Logger logger) : logger_(logger) {}
  std::vector<std::array<double, 2>> convertPathToList(const nav_msgs::msg::Path & msg);
  std::pair<int, double> getNumberOfPosesWithinPathLength(
    const std::vector<std::array<double, 2>> & path, double preferredLength);
  std::array<double, 2> calculate_new_end_point(
    const std::array<double, 2> & pose_second_last, const std::array<double, 2> & pose_last,
    double total_length_rl_path, double preferred_length);
  std::vector<std::array<double, 2>> resample_uniform_distance(
    const std::vector<std::array<double, 2>> & original_path, int num_points,
    double preferred_length);
  core_custom_messages::msg::PathWithLength convertListToMsg(
    const std::vector<std::array<double, 2>> & path, const std::string & frame_id,
    double path_length, const rclcpp::Time & stamp);
  std::pair<core_custom_messages::msg::PathWithLength, int> processPath(
    const nav_msgs::msg::Path & smac_path, double rl_path_length_, int rl_path_samples_,
    const std::string & global_frame_);
  std::array<double, 2> calculate_new_pose(
    const std::array<double, 2> & pose1, const std::array<double, 2> & pose2, double ratio);
  std::vector<std::array<double, 2>> resampleUniformPath(
    const std::vector<std::array<double, 2>> & resampled_plan,
    const std::vector<std::array<double, 2>> & path_trimmed,
    double trimmed_preferred_distance_difference, int rl_path_samples_, double rl_path_length_,
    double totalLength);
  std::vector<std::array<double, 2>> trimPath(
    const std::vector<std::array<double, 2>> & resampled_plan, int numberOfPoses,
    const std::array<double, 2> & trimmed_pose_last);
  double calculate_distance(
    const std::array<double, 2> & pose1, const std::array<double, 2> & pose2);
  double calculatePathLength(const std::vector<std::array<double, 2>> & path);

private:
  rclcpp::Logger logger_;
};

}  // namespace nav2_rl

#endif  // CORE_NAV2_NAVIGATION__RESAMPLE_PATH_HPP_
