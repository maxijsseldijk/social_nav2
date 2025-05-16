#ifndef CORE_NAV2_NAVIGATION__RL_LOCAL_PLANNER_HPP_
#define CORE_NAV2_NAVIGATION__RL_LOCAL_PLANNER_HPP_
#include <memory>
#include <string>

#include "core_nav2_navigation/resample_path.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "social_force_window_planner/sfw_planner_node.hpp"
#include "rl_pure_pursuit_controller/rl_pure_pursuit_controller.hpp"
#include "std_msgs/msg/bool.hpp"

namespace nav2_rl_local_planner
{
class RLLocalPlanner : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for RLLocalPlanner
   */
  RLLocalPlanner() = default;

  /**
   * @brief Destructor for RLLocalPlanner
   */
  ~RLLocalPlanner() = default;

  /**
   * @brief Configure controller on bringup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer to use
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup resources
   */
  void cleanup() override;

  /**
   * @brief Activate controller
   */
  void activate() override;

  /**
   * @brief Deactivate controller
   */
  void deactivate() override;

  /**
   * @brief Main method to compute velocities using the optimizer
   * @param robot_pose Robot pose
   * @param robot_speed Robot speed
   * @param goal_checker Pointer to the goal checker for awareness if completed task
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, nav2_core::GoalChecker * goal_checker) override;

  core_custom_messages::msg::PathWithLength transformPathToLocal(
    const core_custom_messages::msg::PathWithLength & path, const std::string & target_frame);

  /**
   * @brief Set new reference path to track
   * @param path Path to track
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Set new speed limit from callback
   * @param speed_limit Speed limit to use
   * @param percentage Bool if the speed limit is absolute or relative
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Visualize trajectories
   * @param transformed_plan Transformed input plan
   */
  std::shared_ptr<nav2_core::Controller> controller_;
  std::string name_, namespace_, controller_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("RLLocalPlanner")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_rl::ResamplePath> resample_path_;

  double rl_path_length_;
  int rl_path_samples_;
  bool first_call_, merge_rl_path_with_smac_path_;
  double last_timestamp_;
  std::string rl_action_output_;
  std::string global_frame_;
  rclcpp::Publisher<core_custom_messages::msg::PathWithLength>::SharedPtr resample_path_pub_;
  rclcpp::Publisher<core_custom_messages::msg::PathWithLength>::SharedPtr resample_path_local_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr test_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imitation_learning_training_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr inside_interaction_range_;
  std::shared_ptr<std_msgs::msg::Bool> imitation_learning_training_msg_;
  std::shared_ptr<std_msgs::msg::Bool> inside_interaction_range_msg_;
  std::shared_ptr<nav_msgs::msg::Path> reinforcement_learning_path_;
};

}  // namespace nav2_rl_local_planner

#endif  // CORE_NAV2_NAVIGATION__RL_LOCAL_PLANNER_HPP_
