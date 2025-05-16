#ifndef RL_PURE_PURSUIT_CONTROLLER__RL_PURE_PURSUIT_CONTROLLER_HPP_
#define RL_PURE_PURSUIT_CONTROLLER__RL_PURE_PURSUIT_CONTROLLER_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "core_custom_messages/msg/path_with_length.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rl_pure_pursuit_controller
{

/**
 * @class rl_pure_pursuit_controller::RlPurePursuitController
 * @brief Rl pure pursuit controller plugin
 */
class RlPurePursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for rl_pure_pursuit_controller::RlPurePursuitController
   */
  RlPurePursuitController() = default;

  /**
   * @brief Destructor for rl_pure_pursuit_controller::RlPurePursuitController
   */
  ~RlPurePursuitController() = default;

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
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @return Lookahead point
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  /**
   * @brief Based on Rl waypoint received return updated lookaheadpoint
   * @param carrot_pose Model based lookahead distance
   * @return RL Lookahead point
   */
  geometry_msgs::msg::PoseStamped getRlLookAheadPoint(
    const geometry_msgs::msg::PoseStamped carrot_pose);
  /**
   * @brief Find the intersection a circle and a line segment.
   * This assumes the circle is centered at the origin.
   * If no intersection is found, a floating point error will occur.
   * @param p1 first endpoint of line segment
   * @param p2 second endpoint of line segment
   * @param r radius of circle
   * @return point of intersection
   */
  static geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_, namespace_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("RlPurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  tf2::Duration transform_tolerance_;
  double lookahead_dist_;
  double max_angular_vel_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
    carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<core_custom_messages::msg::PathWithLength>>
    carrot_plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rl_plan_sub_;

  rclcpp::Time timestamp_rl_path_request_;
  rclcpp::Time timestamp_rl_path_received_;
  double global_path_length_;
  nav_msgs::msg::Path global_plan_;
  bool new_waypoint_received_;
  geometry_msgs::msg::PoseStamped last_carrot_pose_;
  std::mutex mutex_;
  std::shared_ptr<nav_msgs::msg::Path> rl_waypoint_;
  double desired_linear_vel_, base_desired_linear_vel_;
};

}  // namespace rl_pure_pursuit_controller

#endif  // RL_PURE_PURSUIT_CONTROLLER__RL_PURE_PURSUIT_CONTROLLER_HPP_