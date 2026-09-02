#ifndef AGV_LYAPUNOV_CONTROLLER__LYAPUNOV_CONTROLLER_HPP_
#define AGV_LYAPUNOV_CONTROLLER__LYAPUNOV_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace agv_lyapunov_controller
{

class LyapunovController : public nav2_core::Controller
{
public:
  LyapunovController() = default;
  ~LyapunovController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;

  // Global Path & Visualizer Publisher
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

  // Parameter Kontrol Lyapunov
  double k_x_{1.5};
  double k_y_{2.0};
  double k_theta_{1.2};
  double lookahead_dist_{0.3};
  double max_vel_x_{0.5};
  double max_vel_theta_{1.0};
  double desired_linear_vel_{0.3};

  // Helper math
  double getYaw(const geometry_msgs::msg::Quaternion & q);
};

}  // namespace agv_lyapunov_controller

#endif  // AGV_LYAPUNOV_CONTROLLER__LYAPUNOV_CONTROLLER_HPP_