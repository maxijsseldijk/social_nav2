#include "rl_pure_pursuit_controller/rl_pure_pursuit_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace rl_pure_pursuit_controller
{
void RlPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  namespace_ = node->get_namespace();

  global_path_length_ = 0.0;

  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_pose_rviz", 1);
  carrot_plan_pub_ =
    node->create_publisher<core_custom_messages::msg::PathWithLength>("lookahead_pose", 1);

  rl_plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    namespace_ + "/rl_local_plan", 1, [this](const nav_msgs::msg::Path::SharedPtr msg) {
      rl_waypoint_ = msg;
      new_waypoint_received_ = true;
    });

  timestamp_rl_path_request_ = clock_->now();
  timestamp_rl_path_received_ = clock_->now();
  double transform_tolerance = 0.1;

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.3));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(0.5));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_vel_);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  new_waypoint_received_ = false;
}
void RlPurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " rl_pure_pursuit_controller::RlPurePursuitController",
    plugin_name_.c_str());
  carrot_pub_.reset();
  carrot_plan_pub_.reset();
}

void RlPurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type"
    " rl_pure_pursuit_controller::RlPurePursuitController",
    plugin_name_.c_str());
  carrot_pub_->on_activate();
  carrot_plan_pub_->on_activate();
  auto node = node_.lock();
}

void RlPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type"
    " rl_pure_pursuit_controller::RlPurePursuitController",
    plugin_name_.c_str());
  carrot_pub_->on_deactivate();
  carrot_plan_pub_->on_deactivate();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RlPurePursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

geometry_msgs::msg::PoseStamped RlPurePursuitController::getRlLookAheadPoint(
  const geometry_msgs::msg::PoseStamped carrot_pose)
{
  geometry_msgs::msg::PoseStamped current_carrot_pose;
  current_carrot_pose.header = carrot_pose.header;

  if (rl_waypoint_ == nullptr) {
    current_carrot_pose.pose = carrot_pose.pose;

  } else if (!new_waypoint_received_) {
    current_carrot_pose = last_carrot_pose_;

  } else {
    new_waypoint_received_ = false;
    current_carrot_pose.pose.position.x = rl_waypoint_->poses.begin()->pose.position.x;
    current_carrot_pose.pose.position.y = rl_waypoint_->poses.begin()->pose.position.y;
    last_carrot_pose_ = current_carrot_pose;
  }
  return current_carrot_pose;
}
geometry_msgs::msg::TwistStamped RlPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*speed*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto transformed_plan = transformGlobalPlan(pose);

  auto carrot_pose = getLookAheadPoint(lookahead_dist_, transformed_plan);

  // We convert the carrot pose to PathWithLength to have the proper information needed for RL
  core_custom_messages::msg::PathWithLength carrot_pose_with_length;
  carrot_pose_with_length.header = carrot_pose.header;
  carrot_pose_with_length.path_length = global_path_length_;
  carrot_pose_with_length.poses.push_back(carrot_pose);

  carrot_pub_->publish(createCarrotMsg(carrot_pose));
  carrot_plan_pub_->publish(carrot_pose_with_length);

  auto rl_carrot_pose = getRlLookAheadPoint(carrot_pose);

  double linear_vel, angular_vel;

  // If the goal pose is in front of the robot then compute the velocity using the pure pursuit algorithm
  // else rotate with the max angular velocity until the goal pose is in front of the robot
  if (rl_carrot_pose.pose.position.x > 0) {
    auto curvature = 2.0 * rl_carrot_pose.pose.position.y /
                     (rl_carrot_pose.pose.position.x * rl_carrot_pose.pose.position.x +
                      rl_carrot_pose.pose.position.y * rl_carrot_pose.pose.position.y);
    linear_vel = desired_linear_vel_;
    angular_vel = desired_linear_vel_ * curvature;
  } else {
    linear_vel = 0.0;
    angular_vel = max_angular_vel_;
  }

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z =
    max(-1.0 * abs(max_angular_vel_), min(angular_vel, abs(max_angular_vel_)));

  return cmd_vel;
}
void RlPurePursuitController::setPlan(const nav_msgs::msg::Path & path) { global_plan_ = path; }

nav_msgs::msg::Path RlPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }
  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }
  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();
  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_costmap_extent);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // TODO create function that trims the plan to a few select points that can
  // be used in the RL algorithm.

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

  // Calculate the total path length
  double total_path_length = nav2_util::geometry_utils::calculate_path_length(global_plan_);

  // Add the distance from robot to the first pose in the transformed plan
  if (!transformed_plan.poses.empty()) {
    // Robot is at origin in the transformed plan's frame
    double distance_to_first_pose = euclidean_distance(
      geometry_msgs::msg::PoseStamped(),  // Empty pose (0,0,0)
      transformed_plan.poses.front());

    total_path_length += distance_to_first_pose;

    RCLCPP_DEBUG(
      logger_, "Added distance to first pose: %f, Total path length: %f", distance_to_first_pose,
      total_path_length);
  }

  global_path_length_ = total_path_length;  // Make it available to publish to the rl script

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

geometry_msgs::msg::Point RlPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped RlPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;

    return pose;
  }

  return *goal_pose_it;
}

bool RlPurePursuitController::transformPose(
  const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double RlPurePursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

void RlPurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_vel_ = base_desired_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
}

}  // namespace rl_pure_pursuit_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rl_pure_pursuit_controller::RlPurePursuitController, nav2_core::Controller)
