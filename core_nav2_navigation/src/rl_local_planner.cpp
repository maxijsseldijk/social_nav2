#include "core_nav2_navigation/rl_local_planner.hpp"

#include <stdint.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"

// #define BENCHMARK_TESTING

namespace nav2_rl_local_planner
{

void RLLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros->getGlobalFrameID();

  tf_buffer_ = tf;
  name_ = name;
  auto node = parent_.lock();
  namespace_ = node->get_namespace();

  logger_ = node->get_logger();

  last_timestamp_ = 0.0;

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".controller", rclcpp::ParameterValue("dwb_core::DWBLocalPlanner"));
  node->get_parameter(name_ + ".controller", controller_name_);

  resample_path_ = std::make_shared<nav2_rl::ResamplePath>(logger_);
  first_call_ = true;
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rl_path_length", rclcpp::ParameterValue(5.0));
  node->get_parameter(name_ + ".rl_path_length", rl_path_length_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rl_path_samples", rclcpp::ParameterValue(100));
  node->get_parameter(name_ + ".rl_path_samples", rl_path_samples_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rl_action_output", rclcpp::ParameterValue(""));
  node->get_parameter(name_ + ".rl_action_output", rl_action_output_);

  if (controller_name_ == "dwb_core::DWBLocalPlanner") {
    // First check if the requested output is compatible with the controller
    // DWBLocalPlanner currently supports 'plan' and 'diff_drive' outputs
    if (rl_action_output_ != "plan" && rl_action_output_ != "diff_drive") {
      RCLCPP_ERROR(
        logger_, "Requested output '%s' is not supported by DWBLocalPlanner",
        rl_action_output_.c_str());
      throw nav2_core::PlannerException("Invalid output type for DWBLocalPlanner");
    }
    try {
      controller_ = std::make_shared<dwb_core::DWBLocalPlanner>();
      RCLCPP_INFO(logger_, "Controller %s instance created", controller_name_.c_str());

      controller_->configure(parent, name, tf, costmap_ros);
      RCLCPP_INFO(logger_, "Controller %s is configured", controller_name_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to create or configure controller %s: %s", controller_name_.c_str(),
        e.what());
    }

  } else if (controller_name_ == "social_force_window_planner::SFWPlannerNode") {
    // First check if the requested output is compatible with the controller
    // SFWPlannerNode currently supports 'plan' and 'diff_drive' outputs
    if (rl_action_output_ != "plan" && rl_action_output_ != "diff_drive") {
      RCLCPP_ERROR(
        logger_, "Requested output '%s' is not supported by SFWPlannerNode",
        rl_action_output_.c_str());
      throw nav2_core::PlannerException("Invalid output type for SFWPlannerNode");
    }
    try {
      controller_ = std::make_shared<social_force_window_planner::SFWPlannerNode>();
      controller_->configure(parent, name, tf, costmap_ros);
      RCLCPP_ERROR(logger_, "Controller %s is  configured", controller_name_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to create or configure controller %s: %s", controller_name_.c_str(),
        e.what());
    }

  } else if (controller_name_ == "rl_pure_pursuit_controller::RlPurePursuitController") {
    // First check if the requested output is compatible with the controller
    // RlPurePursuitController currently supports 'waypoint' and 'diff_drive' outputs
    if (
      rl_action_output_ != "waypoint" && rl_action_output_ != "diff_drive" &&
      rl_action_output_ != "plan") {
      RCLCPP_ERROR(
        logger_, "Requested output '%s' is not supported by RlPurePursuitController",
        rl_action_output_.c_str());
      throw nav2_core::PlannerException("Invalid output type for RlPurePursuitController");
    }
    try {
      controller_ = std::make_shared<rl_pure_pursuit_controller::RlPurePursuitController>();
      controller_->configure(parent, name, tf, costmap_ros);
      RCLCPP_ERROR(logger_, "Controller %s is  configured", controller_name_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to create or configure controller %s: %s", controller_name_.c_str(),
        e.what());
    }

  }

  else {
    RCLCPP_ERROR(logger_, "Controller %s is not supported", controller_name_.c_str());
  }

  path_subscription_ = node->create_subscription<nav_msgs::msg::Path>(
    namespace_ + "/rl_local_plan", 1, [this](const nav_msgs::msg::Path::SharedPtr msg) {
      RCLCPP_INFO(logger_, "Received reinforcement learning path");
      reinforcement_learning_path_ = msg;
    });

  resample_path_pub_ = node->create_publisher<core_custom_messages::msg::PathWithLength>(
    namespace_ + "/resampled_plan", 1);

  resample_path_local_pub_ = node->create_publisher<core_custom_messages::msg::PathWithLength>(
    namespace_ + "/resampled_plan_local", 1);

  imitation_learning_training_msg_ = nullptr;

  imitation_learning_training_ = node->create_subscription<std_msgs::msg::Bool>(
    "/imitation_learning_running", 1, [this](const std_msgs::msg::Bool::SharedPtr msg) {
      RCLCPP_INFO(logger_, "Imitation learning training message received");

      imitation_learning_training_msg_ = msg;
    });

  inside_interaction_range_msg_ = nullptr;

  inside_interaction_range_ = node->create_subscription<std_msgs::msg::Bool>(
    "/in_interaction_range", 1,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { inside_interaction_range_msg_ = msg; });

  test_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(namespace_ + "/test_path", 1);

  RCLCPP_INFO(logger_, "Configured RL Controller: %s", name_.c_str());
}

void RLLocalPlanner::cleanup()
{
  controller_->cleanup();

  RCLCPP_INFO(logger_, "Cleaned up RL Controller: %s", name_.c_str());
}

void RLLocalPlanner::activate()
{
  controller_->activate();

  RCLCPP_INFO(logger_, "Activated RL Controller: %s", name_.c_str());
}

void RLLocalPlanner::deactivate()
{
  controller_->deactivate();

  RCLCPP_INFO(logger_, "Deactivated RL Controller: %s", name_.c_str());
}

geometry_msgs::msg::TwistStamped RLLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  return controller_->computeVelocityCommands(robot_pose, robot_speed, goal_checker);
}

core_custom_messages::msg::PathWithLength RLLocalPlanner::transformPathToLocal(
  const core_custom_messages::msg::PathWithLength & path, const std::string & target_frame)
{
  core_custom_messages::msg::PathWithLength local_path;
  local_path.header.stamp = path.header.stamp;
  local_path.header.frame_id = target_frame;
  local_path.path_length = path.path_length;
  for (unsigned int i = 0; i < path.poses.size(); i++) {
    geometry_msgs::msg::PoseStamped pose = path.poses[i];
    pose.header.stamp = path.header.stamp;
    pose.header.frame_id = path.header.frame_id;

    try {
      tf2::Duration tf_tolerance = tf2::durationFromSec(0.2);
      geometry_msgs::msg::PoseStamped local_pose =
        tf_buffer_->transform(pose, target_frame, tf_tolerance);
      local_path.poses.push_back(local_pose);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        rclcpp::get_logger("RLLocalPlanner"), "Could NOT transform pose to %s: %s",
        target_frame.c_str(), ex.what());
    }
  }
  return local_path;
}

void RLLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  // There are two options either we use the RL plan for the SetPlan or we update the plan within the plugin itself.
  // For existing NAV2 plugins we will update the SetPlan as there is not a direct way to update the plan within the plugin.
  // For the Social Pure Pursuit we generate the plan within the plugin itself.

  if (
    controller_name_ == "rl_pure_pursuit_controller::RlPurePursuitController" ||
    rl_action_output_ == "diff_drive") {
    controller_->setPlan(path);
    return;
  } else {
    auto [uniform_path_msg, numberOfPoses] =
      resample_path_->processPath(path, rl_path_length_, rl_path_samples_, path.header.frame_id);
    resample_path_pub_->publish(uniform_path_msg);

    auto local_path = transformPathToLocal(uniform_path_msg, costmap_ros_->getBaseFrameID());
    resample_path_local_pub_->publish(local_path);

    nav_msgs::msg::Path path_test;
    path_test.header.stamp = rclcpp::Clock().now();
    path_test.header.frame_id = path.header.frame_id;
    path_test.poses.insert(
      path_test.poses.end(), uniform_path_msg.poses.begin(), uniform_path_msg.poses.end());
    test_path_pub_->publish(path_test);

    if (reinforcement_learning_path_ && rl_action_output_ == "plan") {
      if (imitation_learning_training_msg_ && imitation_learning_training_msg_->data == true) {
        reinforcement_learning_path_ = std::make_shared<nav_msgs::msg::Path>(path);
      } else if (inside_interaction_range_msg_ && inside_interaction_range_msg_->data == false) {
        RCLCPP_INFO(
          logger_,
          "RL path is created with outside interaction range for this reason the global plan "
          "is used as input.");
        controller_->setPlan(path);
        return;
      }

      double timestamp = reinforcement_learning_path_->header.stamp.sec +
                         reinforcement_learning_path_->header.stamp.nanosec / 1e9;

      if (last_timestamp_ == 0.0) {
        last_timestamp_ = timestamp;
      } else if (timestamp == last_timestamp_) {
        RCLCPP_INFO(
          logger_,
          "RL path not updated while the global planner updates. Consider using a smaller timestep "
          "for the RL algorithm. For now using previous plan");
        return;
      } else if (timestamp > last_timestamp_) {
        last_timestamp_ = timestamp;

        nav_msgs::msg::Path rl_path;
        rl_path.header.stamp = rclcpp::Clock().now();
        rl_path.header.frame_id = reinforcement_learning_path_->header.frame_id;
        rl_path.poses.insert(
          rl_path.poses.end(), reinforcement_learning_path_->poses.begin(),
          reinforcement_learning_path_->poses.end());

        controller_->setPlan(rl_path);

      } else {
        RCLCPP_ERROR(
          logger_, "RL path timestamp is less than the previous timestamp. Not updating the path");
        return;
      }

    } else if (reinforcement_learning_path_ == nullptr && rl_action_output_ == "plan") {
      RCLCPP_INFO(
        logger_,
        "rl_endpoint_ is not initialized. Check if the path is publishing on /RL_local_path. For "
        "now using global plan");

      controller_->setPlan(path);
    } else {
      RCLCPP_INFO(
        logger_,
        "Invalid rl_action_output_ parameter. Please set it to either 'plan','diff_drive' "
        " or 'waypoint'. Using global plan instead");

      controller_->setPlan(path);
    }
  }
}
void RLLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  controller_->setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_rl_local_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rl_local_planner::RLLocalPlanner, nav2_core::Controller)
