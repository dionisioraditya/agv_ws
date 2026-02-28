#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace my_nav2_planners
{

class MyPlanner : public nav2_core::GlobalPlanner
{
public:
  MyPlanner() = default;
  ~MyPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Example params you can tune for your algorithm
  double timeout_sec_{1.5};
  bool allow_unknown_{false};

  // === RRT params (ADD) ===
  int max_iterations_{8000};
  double step_size_{0.25};        // meter
  double goal_tolerance_{0.30};   // meter
  double goal_bias_{0.05};        // 0..1
  bool smooth_path_{true};
  int smooth_tries_{200};

  // Helpers
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;
};

}  // namespace my_nav2_planners
