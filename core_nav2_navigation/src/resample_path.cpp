#include "core_nav2_navigation/resample_path.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_rl
{
const double EPSILON = 1e-4;
const double NEARLY_ONE = 0.9;

//Conversion Methods

/**
     * @brief Converts a path message to a list of points.
     *
     * This function takes a nav_msgs::msg::Path message and converts it into a list of points.
     * Each point is represented as a std::array<double, 2>, where the first element is the 
     * x-coordinate and the second element is the y-coordinate.
     *
     * @param msg The path message to convert.
     * @return A list of points representing the path.
     */
std::vector<std::array<double, 2>> ResamplePath::convertPathToList(const nav_msgs::msg::Path & msg)
{
  std::vector<std::array<double, 2>> path;
  for (const auto & pose : msg.poses) {
    std::array<double, 2> point = {pose.pose.position.x, pose.pose.position.y};
    path.push_back(point);
  }
  return path;
}

/**
     * @brief Converts a list of points to a path message.
     *
     * This function takes a list of points and converts it into a nav_msgs::msg::Path message.
     * Each point is represented as a std::array<double, 2>, where the first element is the
     * x-coordinate and the second element is the y-coordinate. The path is normalized with 
     * respect to the start point for the RL purpose it is used for, and the frame_id 
     * is set to the given frame_id. The function also sets the timestamp and frame_id for
     * each pose in the path message.
     *
     * @param path The list of points to convert.
     * @param frame_id The frame id to set in the path message.
     * @return A path message representing the list of points.
     */
core_custom_messages::msg::PathWithLength ResamplePath::convertListToMsg(
  const std::vector<std::array<double, 2>> & path, const std::string & frame_id,
  double total_path_length, const rclcpp::Time & stamp)
{
  core_custom_messages::msg::PathWithLength pathMsg;
  pathMsg.header.stamp = stamp;
  pathMsg.header.frame_id = frame_id;

  for (const auto & pose : path) {
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header.stamp = stamp;
    poseStamped.header.frame_id = frame_id;

    poseStamped.pose.position.x = pose[0];
    poseStamped.pose.position.y = pose[1];
    poseStamped.pose.position.z = 0.0;  //2D Path

    // Orientation is not used in the planner
    poseStamped.pose.orientation.x = 0.0;
    poseStamped.pose.orientation.y = 0.0;
    poseStamped.pose.orientation.z = 0.0;
    poseStamped.pose.orientation.w = 1.0;

    pathMsg.poses.push_back(poseStamped);
  }

  pathMsg.path_length = total_path_length;
  return pathMsg;
}

//Calculation Methods

/**
     * @brief Calculates the Euclidean distance between two points.
     *
     * This function takes two points, each represented as a std::array<double, 2>,
     * and calculates the Euclidean distance between them.
     * If the size of either array is not 2, an error message is logged and the function returns 0.
     *
     * @param pose1 The first point.
     * @param pose2 The second point.
     * @return The Euclidean distance between the two points.
     */
double ResamplePath::calculate_distance(
  const std::array<double, 2> & pose1, const std::array<double, 2> & pose2)
{
  if (pose1.size() != 2 || pose2.size() != 2) {
    RCLCPP_ERROR(logger_, "Poses must have 2 elements as the planner works in 2D. Returning 0.");
    return 0;
  }

  return std::sqrt(std::pow(pose1[0] - pose2[0], 2) + std::pow(pose1[1] - pose2[1], 2));
}

/**
     * @brief Calculates a new end point for a path.
     *
     * This function takes the last two points of a path and the total and preferred lengths 
     * of the path, and calculates a new end point for the path. If the total length of the path is
     * greater than or equal to the preferred length, the new end point is calculated such that 
     * the length of the path is equal to the preferred length. 
     * Otherwise, the last point of the path is returned as the new end point.
     *
     * @param pose_second_last The second last point of the path.
     * @param pose_last The last point of the path.
     * @param total_length_rl_path The total length of the path.
     * @param preferred_length The preferred length of the path.
     * @return The new end point for the path.
     */
std::array<double, 2> ResamplePath::calculate_new_end_point(
  const std::array<double, 2> & pose_second_last, const std::array<double, 2> & pose_last,
  double total_length_rl_path, double preferred_length)
{
  std::array<double, 2> new_last_pose;

  if (total_length_rl_path >= preferred_length) {
    double distance = calculate_distance(pose_second_last, pose_last);
    double ratio = 1 - (total_length_rl_path - preferred_length) / distance;
    new_last_pose = calculate_new_pose(pose_second_last, pose_last, ratio);
  } else {
    new_last_pose = pose_last;
  }

  return new_last_pose;
}

/**
     * @brief Calculates a new pose between two given poses.
     *
     * This function takes two poses and a ratio, and calculates a new pose that is located 
     * at the given ratio along the line segment between the two poses.
     * The new pose is calculated as a linear interpolation between the two poses.
     *
     * @param pose1 The first pose.
     * @param pose2 The second pose.
     * @param ratio The ratio at which to calculate the new pose.
     * @return The new pose.
     */
std::array<double, 2> ResamplePath::calculate_new_pose(
  const std::array<double, 2> & pose1, const std::array<double, 2> & pose2, double ratio)
{
  return {pose1[0] + (pose2[0] - pose1[0]) * ratio, pose1[1] + (pose2[1] - pose1[1]) * ratio};
}

//Path Processing Methods

/**
     * @brief Calculates the number of poses within a given path length.
     *
     * This function takes a path and a preferred length, and calculates the number 
     * of poses within the preferred length. The function also calculates the total length of the 
     * path up to the last pose within the preferred length. If the preferred length is greater 
     * than the total length of the path, the function returns the number of poses in the path
     * and the total length of the path.
     *
     * @param path The path.
     * @param preferredLength The preferred length.
     * @return A pair containing the number of poses and the total length.
     */
std::pair<int, double> ResamplePath::getNumberOfPosesWithinPathLength(
  const std::vector<std::array<double, 2>> & path, double preferredLength)
{
  double totalLength = 0;
  size_t currentPose = 0;
  while (totalLength < preferredLength && currentPose < path.size() - 1) {
    auto poseA = path[currentPose];
    auto poseB = path[currentPose + 1];
    double distance = calculate_distance(poseA, poseB);
    totalLength += distance;
    currentPose++;
  }
  int numberOfPoses = currentPose;
  return {numberOfPoses, totalLength};
}

/**
     * @brief Trims a path to a given number of poses.
     *
     * This function takes a path, a number of poses, and a last pose,
     * and trims the path to the given number of poses.
     * The last pose of the trimmed path is set to the given last pose.
     *
     * @param resampled_plan The path.
     * @param numberOfPoses The number of poses.
     * @param trimmed_pose_last The last pose.
     * @return The trimmed path.
     */
std::vector<std::array<double, 2>> ResamplePath::trimPath(
  const std::vector<std::array<double, 2>> & resampled_plan, int numberOfPoses,
  const std::array<double, 2> & trimmed_pose_last)
{
  std::vector<std::array<double, 2>> path_trimmed;
  path_trimmed.reserve(numberOfPoses);
  path_trimmed.insert(
    path_trimmed.end(), resampled_plan.begin(), resampled_plan.begin() + numberOfPoses - 1);
  path_trimmed.push_back(trimmed_pose_last);

  return path_trimmed;
}
/**
     * @brief Resamples a path with uniform distance between poses.
     *
     * This function takes a path, a number of points, and a preferred length, and resamples 
     * the path such that the distance between poses is uniform.
     * If the number of points is less than 2, an error message is logged
     * and the original path is returned.
     *
     * @param original_path The original path.
     * @param num_points The number of points.
     * @param preferred_length The preferred length.
     * @return The resampled path.
     */
std::vector<std::array<double, 2>> ResamplePath::resample_uniform_distance(
  const std::vector<std::array<double, 2>> & original_path, int num_points, double preferred_length)
{
  double segment_length = 0;

  if (num_points == 1) {
    std::vector<std::array<double, 2>> resampled_path;
    auto last_pose = original_path.back();
    resampled_path.push_back(last_pose);

    return resampled_path;
  } else if (num_points > 1) {
    segment_length = preferred_length / (num_points);
  } else {
    RCLCPP_ERROR(logger_, "Number of points must be greater than 0. Returning original path.");
    return original_path;
  }

  std::vector<std::array<double, 2>> resampled_path;
  double remaining_distance = 0;

  for (size_t pose = 0; pose < original_path.size() - 1; ++pose) {
    std::array<double, 2> pose_a = original_path[pose];
    std::array<double, 2> pose_b = original_path[pose + 1];

    double distance_a_b = calculate_distance(pose_a, pose_b);
    double total_distance = remaining_distance + distance_a_b;
    double num_points_in_segment = total_distance / segment_length;

    //RCLCPP_ERROR(logger_, "num_points_in_segment: %f", num_points_in_segment);
    // If a resampled point fits in the segment, add it to the resampled path
    for (int segment = 0; segment < static_cast<int>(num_points_in_segment); ++segment) {
      double ratio = 1 - (total_distance - segment_length * (segment + 1)) / distance_a_b;
      resampled_path.push_back(calculate_new_pose(pose_a, pose_b, ratio));
    }

    // The leftover distance is the distance that was not covered by the resampled points
    // This also includes the distance between the last resampled point 
    // and the next point in the original path.
    remaining_distance = total_distance - static_cast<int>(num_points_in_segment) * segment_length;
  }

  // This is necessary as small calculation errors can lead to the last point not being added
  // To prevent this an check is made that checks if we are close to the final point.
  if (
    calculate_distance(resampled_path.back(), original_path.back()) > segment_length * NEARLY_ONE) {
    resampled_path.push_back(original_path.back());
  } else {
    resampled_path.back() = original_path.back();
  }

  // Log an error if the number of points in the resampled path is not as expected
  if (static_cast<int>(resampled_path.size()) != num_points) {
    RCLCPP_ERROR(
      logger_,
      "Resampled path has %lu points instead of %d. Adjusting the number of points manually to "
      "prevent error in RL algorithm.",
      resampled_path.size(), num_points);
    RCLCPP_ERROR(
      logger_, "Last point: (%f, %f)", resampled_path.back()[0], resampled_path.back()[1]);
    RCLCPP_ERROR(
      logger_, "Original last point: (%f, %f)", original_path.back()[0], original_path.back()[1]);

    RCLCPP_ERROR(logger_, "First point: (%f, %f)", resampled_path[0][0], resampled_path[0][1]);
    resampled_path.resize(num_points);
  }

  return resampled_path;
}

/**
     * @brief Resamples a path.
     *
     * This function takes a path, a trimmed path, a trimmed preferred distance difference,
     *  a number of samples, a path length, and a total length, and resamples the path.
     * If the trimmed preferred distance difference is less than a small epsilon, the trimmed path 
     * is resampled with uniform distance between poses.
     * Otherwise, the original path is resampled with uniform distance between poses.
     *
     * @param resampled_plan The path.
     * @param path_trimmed The trimmed path.
     * @param trimmed_preferred_distance_difference The trimmed preferred distance difference.
     * @param rl_path_samples_ The number of samples.
     * @param rl_path_length_ The requested path length.
     * @param totalLength The total length of the path still to go.
     * @return The resampled path.
     */
std::vector<std::array<double, 2>> ResamplePath::resampleUniformPath(
  const std::vector<std::array<double, 2>> & resampled_plan,
  const std::vector<std::array<double, 2>> & path_trimmed,
  double trimmed_preferred_distance_difference, int rl_path_samples_, double rl_path_length_,
  double totalLength)
{
  std::vector<std::array<double, 2>> uniform_path;
  uniform_path.reserve(rl_path_samples_);

  if (trimmed_preferred_distance_difference < EPSILON) {
    uniform_path = resample_uniform_distance(path_trimmed, rl_path_samples_, rl_path_length_);
  } else {
    uniform_path = resample_uniform_distance(resampled_plan, rl_path_samples_, totalLength);
  }

  return uniform_path;
}

/**
     * @brief Processes a given path by resampling it based on specified parameters.
     *
     * This function takes a path represented by `smac_path` and resamples it to generate a new path
     * with a desired length and number of samples. The resampling process involves trimming the path
     * to the desired length, calculating the new end point, and resampling the path uniformly.
     *
     * @param smac_path The original path to be resampled.
     * @param rl_path_length_ The desired length of the resampled path.
     * @param rl_path_samples_ The desired number of samples in the resampled path.
     * @param global_frame_ The global frame of reference for the path.
     * @return
     */
std::pair<core_custom_messages::msg::PathWithLength, int> ResamplePath::processPath(
  const nav_msgs::msg::Path & smac_path, double rl_path_length_, int rl_path_samples_,
  const std::string & global_frame_)
{
  auto total_path_length = nav2_util::geometry_utils::calculate_path_length(smac_path);

  auto path_list = convertPathToList(smac_path);

  auto [pose_in_path, total_length_rl_path] =
    getNumberOfPosesWithinPathLength(path_list, rl_path_length_);
  auto new_end_point = calculate_new_end_point(
    path_list[pose_in_path - 1], path_list[pose_in_path], total_length_rl_path, rl_path_length_);

  auto trimmed_distance = calculate_distance(path_list[pose_in_path - 1], new_end_point);
  auto untrimmed_distance =
    calculate_distance(path_list[pose_in_path - 1], path_list[pose_in_path]);

  // Calculate the difference between the preferred and actual path length
  auto path_length_difference =
    std::abs(total_length_rl_path - untrimmed_distance + trimmed_distance - rl_path_length_);

  auto trimmed_path = trimPath(path_list, pose_in_path, new_end_point);

  auto resampled_uniform_path = resampleUniformPath(
    path_list, trimmed_path, path_length_difference, rl_path_samples_, rl_path_length_,
    total_path_length);

  // Log debug information
  RCLCPP_DEBUG(logger_, "Resampled path has %lu points", resampled_uniform_path.size());
  RCLCPP_DEBUG(logger_, "Original path has %lu points", path_list.size());
  RCLCPP_DEBUG(logger_, "Trimmed path has %lu points", trimmed_path.size());
  RCLCPP_DEBUG(logger_, "Preferred path length: %f", rl_path_length_);
  RCLCPP_DEBUG(logger_, "Total path length: %f", total_length_rl_path);
  RCLCPP_DEBUG(logger_, "Untrimmed path length: %f", untrimmed_distance);
  RCLCPP_DEBUG(logger_, "Trimmed path length: %f", trimmed_distance);
  RCLCPP_DEBUG(
    logger_, "Difference between trimmed and untrimmed path length: %f", path_length_difference);

  // Convert the list back to a message
  core_custom_messages::msg::PathWithLength resampled_path_msg = convertListToMsg(
    resampled_uniform_path, global_frame_, total_path_length, smac_path.header.stamp);

  return {resampled_path_msg, pose_in_path};
}

}  // namespace nav2_rl
