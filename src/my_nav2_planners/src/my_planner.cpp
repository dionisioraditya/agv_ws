#include "my_nav2_planners/my_planner.hpp"

#include <cmath>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace my_nav2_planners
{

void MyPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
        
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // Declare/get params
    node_->declare_parameter(name_ + ".timeout_sec", timeout_sec_);
    node_->declare_parameter(name_ + ".allow_unknown", allow_unknown_);

    node_->get_parameter(name_ + ".timeout_sec", timeout_sec_);
    node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);

    RCLCPP_INFO(node_->get_logger(), "[%s] configured. timeout=%.2f allow_unknown=%s",
                name_.c_str(), timeout_sec_, allow_unknown_ ? "true" : "false");
}

void MyPlanner::cleanup() { RCLCPP_INFO(node_->get_logger(), "[%s] cleanup", name_.c_str()); }
void MyPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "[%s] activate", name_.c_str()); }
void MyPlanner::deactivate() { RCLCPP_INFO(node_->get_logger(), "[%s] deactivate", name_.c_str()); }

bool MyPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  auto * cm = costmap_ros_->getCostmap();
  return cm->worldToMap(wx, wy, mx, my);
}

void MyPlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  auto * cm = costmap_ros_->getCostmap();
  cm->mapToWorld(mx, my, wx, wy);
}

nav_msgs::msg::Path MyPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = costmap_ros_->getGlobalFrameID();
        path.header.stamp = node_->now();

        // Basic frame sanity
        if (start.header.frame_id != path.header.frame_id || goal.header.frame_id != path.header.frame_id) {
            throw std::runtime_error("Start/Goal must be in the costmap global frame (usually 'map').");
        }

        auto * cm = costmap_ros_->getCostmap();

        unsigned int sx, sy, gx, gy;
        if (!worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
            throw std::runtime_error("Start is outside the costmap.");
        }
        if (!worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
            throw std::runtime_error("Goal is outside the costmap.");
        }

        // --- R&D AREA START ---
        // Sanity planner: sample points along line and reject if obstacle hit.
        const double dx = goal.pose.position.x - start.pose.position.x;
        const double dy = goal.pose.position.y - start.pose.position.y;
        const double dist = std::hypot(dx, dy);
        const int steps = std::max(2, static_cast<int>(dist / cm->getResolution()));

        for (int i = 0; i <= steps; i++) {
            double t = static_cast<double>(i) / static_cast<double>(steps);
            double wx = start.pose.position.x + t * dx;
            double wy = start.pose.position.y + t * dy;

            unsigned int mx, my;
            if (!worldToMap(wx, wy, mx, my)) continue;

            const unsigned char c = cm->getCost(mx, my);

            const bool is_lethal = (c >= nav2_costmap_2d::LETHAL_OBSTACLE);
            const bool is_unknown = (c == nav2_costmap_2d::NO_INFORMATION);

            if (is_lethal || (!allow_unknown_ && is_unknown)) {
            throw std::runtime_error("Straight-line path hit an obstacle/unknown. Replace with your planner.");
            }

            geometry_msgs::msg::PoseStamped p;
            p.header = path.header;
            p.pose.position.x = wx;
            p.pose.position.y = wy;
            p.pose.position.z = 0.0;
            p.pose.orientation = goal.pose.orientation; // simple: point same as goal
            path.poses.push_back(p);
        }
        // --- R&D AREA END ---

        return path;
    }

}  // namespace my_nav2_planners

PLUGINLIB_EXPORT_CLASS(my_nav2_planners::MyPlanner, nav2_core::GlobalPlanner)
