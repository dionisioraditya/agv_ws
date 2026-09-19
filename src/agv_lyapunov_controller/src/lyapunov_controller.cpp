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
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k_rotate", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_rot_vel", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_vel_theta", rclcpp::ParameterValue(0.18));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lim_theta", rclcpp::ParameterValue(1.5));

  node->get_parameter(plugin_name_ + ".k_x", k_x_);
  node->get_parameter(plugin_name_ + ".k_y", k_y_);
  node->get_parameter(plugin_name_ + ".k_theta", k_theta_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".k_rotate", k_rotate_);
  node->get_parameter(plugin_name_ + ".max_rot_vel", max_rot_vel_);
  node->get_parameter(plugin_name_ + ".min_vel_theta", min_vel_theta_);
  node->get_parameter(plugin_name_ + ".acc_lim_theta", acc_lim_theta_);
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
  is_rotating_to_goal_ = false;  // Reset latching saat rute baru diterima
  last_w_ = 0.0;
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
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
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

  // Periksa jarak dan selisih orientasi terhadap goal akhir
  const auto & goal_pose = global_plan_.poses.back();
  double dist_to_goal = std::hypot(goal_pose.pose.position.x - rx, goal_pose.pose.position.y - ry);
  double final_goal_yaw = getYaw(goal_pose.pose.orientation);
  double yaw_error = angles::normalize_angle(final_goal_yaw - r_theta);

  // Cek jika GoalChecker Nav2 sudah menyatakan goal tercapai
  if (goal_checker && goal_checker->isGoalReached(current_pose.pose, goal_pose.pose, velocity)) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    is_rotating_to_goal_ = false;
    last_w_ = 0.0;
    return cmd_vel;
  }

  // =========================================================================
  // FASE 2: ROTATE-TO-GOAL (In-place rotation ketika sudah di dalam radius goal)
  // =========================================================================
  if (dist_to_goal <= xy_goal_tolerance_ || is_rotating_to_goal_) {
    is_rotating_to_goal_ = true;
    cmd_vel.twist.linear.x = 0.0;  // KUNCI kecepatan maju agar robot tidak melingkar

    // Hitung dt untuk ramp rate limiter
    rclcpp::Time current_time = pose.header.stamp;
    double dt = 0.033;
    if (last_time_.nanoseconds() != 0) {
      dt = (current_time - last_time_).seconds();
      if (dt <= 0.0 || dt > 0.5) {
        dt = 0.033;
      }
    }
    last_time_ = current_time;

    // 1. Cek apakah sudut sudah sesuai toleransi
    if (std::abs(yaw_error) <= yaw_goal_tolerance_) {
      cmd_vel.twist.angular.z = 0.0;
      last_w_ = 0.0;
      return cmd_vel;
    }

    // 2. Hitung target kecepatan putar dengan batas maksimum halus (max_rot_vel_)
    double target_w = k_rotate_ * yaw_error;
    target_w = std::clamp(target_w, -max_rot_vel_, max_rot_vel_);

    // Terapkan min_vel_theta_ agar motor tidak macet karena gesekan lantai
    if (std::abs(target_w) < min_vel_theta_) {
      target_w = std::copysign(min_vel_theta_, target_w);
    }

    // 3. Acceleration / Deceleration Limiter (seperti acc_lim_theta & decel_lim_theta di DWB)
    // Mencegah hentakan mendadak yang membuat inersia 20kg bablas/overshoot
    double max_dw = acc_lim_theta_ * dt;
    double dw = target_w - last_w_;
    dw = std::clamp(dw, -max_dw, max_dw);

    double final_w = last_w_ + dw;
    last_w_ = final_w;

    cmd_vel.twist.angular.z = final_w;
    return cmd_vel;
  }

  // =========================================================================
  // FASE 1: PATH TRACKING (Lyapunov Control Law mengikuti arah lintasan)
  // =========================================================================

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
  double ref_theta;

  // Arah referensi saat tracking adalah arah kurva path (tangent of path)
  if (target_idx + 1 < global_plan_.poses.size()) {
    double dx = global_plan_.poses[target_idx + 1].pose.position.x - ref_x;
    double dy = global_plan_.poses[target_idx + 1].pose.position.y - ref_y;
    ref_theta = std::atan2(dy, dx);
  } else if (target_idx > 0) {
    double dx = ref_x - global_plan_.poses[target_idx - 1].pose.position.x;
    double dy = ref_y - global_plan_.poses[target_idx - 1].pose.position.y;
    ref_theta = std::atan2(dy, dx);
  } else {
    ref_theta = getYaw(target_pose.pose.orientation);
  }

  // 4. Hitung Error dalam Robot Frame (Kanayama Transformation)
  double dx_global = ref_x - rx;
  double dy_global = ref_y - ry;

  double e_x =  std::cos(r_theta) * dx_global + std::sin(r_theta) * dy_global;
  double e_y = -std::sin(r_theta) * dx_global + std::cos(r_theta) * dy_global;
  double e_theta = angles::normalize_angle(ref_theta - r_theta);

  // 5. Perlambatan bertahap saat mendekati goal
  double v_ref = desired_linear_vel_;
  if (dist_to_goal < 0.6) {
    v_ref = std::max(0.1, desired_linear_vel_ * (dist_to_goal / 0.6));
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