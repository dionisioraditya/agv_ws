#include "agv_lyapunov_controller/lyapunov_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "angles/angles.h"
#include <cmath>
#include <algorithm>

namespace agv_lyapunov_controller
{

void LyapunovController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Node pointer is null during configure");
  }

  // Publisher untuk visualisasi local plan di RViz (/local_plan)
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

  // Deklarasi & Load Parameters dari YAML
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k_x", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k_y", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k_theta", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.3));

  node->get_parameter(plugin_name_ + ".k_x", k_x_);
  node->get_parameter(plugin_name_ + ".k_y", k_y_);
  node->get_parameter(plugin_name_ + ".k_theta", k_theta_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
}

void LyapunovController::activate()
{
  if (local_plan_pub_) {
    local_plan_pub_->on_activate();
  }
}

void LyapunovController::deactivate()
{
  if (local_plan_pub_) {
    local_plan_pub_->on_deactivate();
  }
}

void LyapunovController::cleanup()
{
  local_plan_pub_.reset();
}

void LyapunovController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

double LyapunovController::getYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::TwistStamped LyapunovController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = pose.header.stamp;
  cmd_vel.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    return cmd_vel;
  }

  // Publish local plan untuk RViz
  if (local_plan_pub_ && local_plan_pub_->is_activated()) {
    auto plan_to_publish = global_plan_;
    plan_to_publish.header.stamp = pose.header.stamp;
    local_plan_pub_->publish(plan_to_publish);
  }

  // Pastikan pose robot dan koordinat path berada di frame yang sama
  geometry_msgs::msg::PoseStamped current_pose = pose;
  if (!global_plan_.header.frame_id.empty() && pose.header.frame_id != global_plan_.header.frame_id) {
    try {
      current_pose = tf_buffer_->transform(pose, global_plan_.header.frame_id, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      if (auto node = node_.lock()) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 1000,
          "Transform pose failed: %s", ex.what());
      }
      return cmd_vel;
    }
  }

  // 1. Current Robot Pose (x, y, theta)
  double rx = current_pose.pose.position.x;
  double ry = current_pose.pose.position.y;
  double r_theta = getYaw(current_pose.pose.orientation);

  // 2. Cari titik terdekat pada path terhadap posisi robot
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - rx;
    double dy = global_plan_.poses[i].pose.position.y - ry;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // 3. Dari closest_idx, cari waypoint ke depan sejauh lookahead_dist_
  size_t target_idx = closest_idx;
  for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - rx;
    double dy = global_plan_.poses[i].pose.position.y - ry;
    double dist = std::hypot(dx, dy);
    if (dist >= lookahead_dist_) {
      target_idx = i;
      break;
    }
    target_idx = i;
  }

  const auto & target_pose = global_plan_.poses[target_idx];
  double ref_x = target_pose.pose.position.x;
  double ref_y = target_pose.pose.position.y;
  double ref_theta = getYaw(target_pose.pose.orientation);

  // Jika waypoint tidak memiliki orientasi valid, orientasikan sesuai arah path
  if (target_idx + 1 < global_plan_.poses.size()) {
    double dx = global_plan_.poses[target_idx + 1].pose.position.x - ref_x;
    double dy = global_plan_.poses[target_idx + 1].pose.position.y - ref_y;
    ref_theta = std::atan2(dy, dx);
  }

  // 4. Hitung Error dalam Robot Frame (Kanayama Transformation)
  double dx_global = ref_x - rx;
  double dy_global = ref_y - ry;

  double e_x =  std::cos(r_theta) * dx_global + std::sin(r_theta) * dy_global;
  double e_y = -std::sin(r_theta) * dx_global + std::cos(r_theta) * dy_global;
  double e_theta = angles::normalize_angle(ref_theta - r_theta);

  // 5. Hitung jarak ke titik akhir untuk perlambatan halus (Ramp down)
  const auto & goal_pose = global_plan_.poses.back();
  double dist_to_goal = std::hypot(goal_pose.pose.position.x - rx, goal_pose.pose.position.y - ry);

  double v_ref = desired_linear_vel_;
  if (dist_to_goal < 0.5) {
    v_ref = std::max(0.05, desired_linear_vel_ * (dist_to_goal / 0.5));
  }
  double w_ref = 0.0;

  // 6. Lyapunov Control Law:
  // v = v_ref * cos(e_theta) + k_x * e_x
  // w = w_ref + v_ref * (k_y * e_y + k_theta * sin(e_theta))
  double v = v_ref * std::cos(e_theta) + k_x_ * e_x;
  double w = w_ref + v_ref * (k_y_ * e_y + k_theta_ * std::sin(e_theta));

  // 7. Saturasi Kecepatan Maksimum
  v = std::clamp(v, 0.0, max_vel_x_);
  w = std::clamp(w, -max_vel_theta_, max_vel_theta_);

  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.angular.z = w;

  return cmd_vel;
}

void LyapunovController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_vel_x_ = max_vel_x_ * (speed_limit / 100.0);
  } else {
    max_vel_x_ = speed_limit;
  }
}

}  // namespace agv_lyapunov_controller

// Export Plugin agar terbaca oleh pluginlib Nav2
PLUGINLIB_EXPORT_CLASS(agv_lyapunov_controller::LyapunovController, nav2_core::Controller)