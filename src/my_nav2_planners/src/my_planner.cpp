#include "my_nav2_planners/my_planner.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <random>
#include <utility>

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
    node_->declare_parameter(name_ + ".max_iterations", max_iterations_);
    node_->declare_parameter(name_ + ".step_size", step_size_);
    node_->declare_parameter(name_ + ".goal_tolerance", goal_tolerance_);
    node_->declare_parameter(name_ + ".goal_bias", goal_bias_);
    node_->declare_parameter(name_ + ".smooth_path", smooth_path_);
    node_->declare_parameter(name_ + ".smooth_tries", smooth_tries_);

    node_->get_parameter(name_ + ".max_iterations", max_iterations_);
    node_->get_parameter(name_ + ".step_size", step_size_);
    node_->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
    node_->get_parameter(name_ + ".goal_bias", goal_bias_);
    node_->get_parameter(name_ + ".smooth_path", smooth_path_);
    node_->get_parameter(name_ + ".smooth_tries", smooth_tries_);

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

  // Frame sanity
  if (start.header.frame_id != path.header.frame_id || goal.header.frame_id != path.header.frame_id) {
    throw std::runtime_error("Start/Goal must be in the costmap global frame (usually 'map').");
  }

  auto * cm = costmap_ros_->getCostmap();
  const double resolution = cm->getResolution();
  const unsigned int size_x = cm->getSizeInCellsX();
  const unsigned int size_y = cm->getSizeInCellsY();

  unsigned int sx, sy, gx, gy;
  if (!worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
    throw std::runtime_error("Start is outside the costmap.");
  }
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
    throw std::runtime_error("Goal is outside the costmap.");
  }

  // ---------------- RRT PARAMS (declare/get in configure, lihat bagian 2) ----------------
  const int    max_iterations   = max_iterations_;
  const double step_size_m      = step_size_;        // meters
  const double goal_tolerance_m = goal_tolerance_;    // meters
  const double goal_bias        = goal_bias_;         // [0..1]
  const bool   do_smooth        = smooth_path_;
  const int    smooth_tries     = smooth_tries_;
  // -------------------------------------------------------------------------------------

  auto isCellValid = [&](unsigned int mx, unsigned int my) -> bool {
    const unsigned char c = cm->getCost(mx, my);
    const bool is_lethal  = (c >= nav2_costmap_2d::LETHAL_OBSTACLE);
    const bool is_unknown = (c == nav2_costmap_2d::NO_INFORMATION);
    if (is_lethal) return false;
    if (!allow_unknown_ && is_unknown) return false;
    return true;
  };

  auto isWorldValid = [&](double wx, double wy) -> bool {
    unsigned int mx, my;
    if (!worldToMap(wx, wy, mx, my)) return false;
    return isCellValid(mx, my);
  };

  // Collision check along segment (sampling)
  auto isSegmentValid = [&](double x0, double y0, double x1, double y1) -> bool {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dist = std::hypot(dx, dy);
    const double ds = std::max(0.5 * resolution, 0.02); // sample step
    const int steps = std::max(1, (int)std::ceil(dist / ds));
    for (int i = 0; i <= steps; ++i) {
      const double t = (double)i / (double)steps;
      const double x = x0 + t * dx;
      const double y = y0 + t * dy;
      if (!isWorldValid(x, y)) return false;
    }
    return true;
  };

  struct Node2D {
    double x{0}, y{0};
    int parent{-1};
  };

  struct Tree {
    std::vector<Node2D> nodes;
    int add(double x, double y, int parent) {
      nodes.push_back(Node2D{x, y, parent});
      return (int)nodes.size() - 1;
    }
    int nearest(double x, double y) const {
      int best = 0;
      double best_d2 = std::numeric_limits<double>::infinity();
      for (int i = 0; i < (int)nodes.size(); ++i) {
        const double dx = nodes[i].x - x;
        const double dy = nodes[i].y - y;
        const double d2 = dx*dx + dy*dy;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
      }
      return best;
    }
  };

  enum class ExtendStatus { Trapped, Advanced, Reached };

  auto steer = [&](double from_x, double from_y, double to_x, double to_y, double step) {
    const double dx = to_x - from_x;
    const double dy = to_y - from_y;
    const double d  = std::hypot(dx, dy);
    if (d <= step) return std::pair<double,double>(to_x, to_y);
    const double ux = dx / d;
    const double uy = dy / d;
    return std::pair<double,double>(from_x + ux*step, from_y + uy*step);
  };

  auto extend = [&](Tree & T, double qx, double qy, int & new_idx) -> ExtendStatus {
    const int near_idx = T.nearest(qx, qy);
    const auto & near = T.nodes[near_idx];
    auto [nx, ny] = steer(near.x, near.y, qx, qy, step_size_m);

    // reject if invalid or segment collides
    if (!isWorldValid(nx, ny)) return ExtendStatus::Trapped;
    if (!isSegmentValid(near.x, near.y, nx, ny)) return ExtendStatus::Trapped;

    new_idx = T.add(nx, ny, near_idx);

    const double rem = std::hypot(qx - nx, qy - ny);
    if (rem <= 1e-6) return ExtendStatus::Reached;
    if (std::hypot(qx - nx, qy - ny) <= step_size_m * 0.5) return ExtendStatus::Reached; // close enough
    return ExtendStatus::Advanced;
  };

  auto connect = [&](Tree & T, double qx, double qy, int & last_idx) -> ExtendStatus {
    ExtendStatus s = ExtendStatus::Trapped;
    int idx = -1;
    do {
      s = extend(T, qx, qy, idx);
      if (s == ExtendStatus::Trapped) { last_idx = -1; return s; }
      last_idx = idx;
    } while (s == ExtendStatus::Advanced);
    return s; // Reached or Trapped
  };

  // Random sampler over costmap bounds (uniform), with goal-bias
  std::mt19937 rng((unsigned)std::chrono::high_resolution_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> uni01(0.0, 1.0);
  std::uniform_int_distribution<unsigned int> rx(0, size_x - 1);
  std::uniform_int_distribution<unsigned int> ry(0, size_y - 1);

  auto sampleFree = [&]() -> std::pair<double,double> {
    // goal bias
    if (uni01(rng) < goal_bias) {
      return {goal.pose.position.x, goal.pose.position.y};
    }

    // try random cells
    for (int k = 0; k < 2000; ++k) {
      const unsigned int mx = rx(rng);
      const unsigned int my = ry(rng);
      if (!isCellValid(mx, my)) continue;
      double wx, wy;
      mapToWorld(mx, my, wx, wy);
      return {wx, wy};
    }
    // fallback: return goal (at least deterministic)
    return {goal.pose.position.x, goal.pose.position.y};
  };

  auto distToGoal = [&](double x, double y) {
    return std::hypot(goal.pose.position.x - x, goal.pose.position.y - y);
  };

  // Init trees (bidirectional)
  Tree Ta, Tb;
  Ta.nodes.reserve(5000);
  Tb.nodes.reserve(5000);

  Ta.add(start.pose.position.x, start.pose.position.y, -1);
  Tb.add(goal.pose.position.x,  goal.pose.position.y,  -1);

  const rclcpp::Time t0 = node_->now();

  int connect_a = -1, connect_b = -1;
  bool solved = false;

  for (int it = 0; it < max_iterations; ++it) {
    if ((node_->now() - t0).seconds() > timeout_sec_) break;

    auto [qx, qy] = sampleFree();

    int new_idx_a = -1;
    auto s1 = extend(Ta, qx, qy, new_idx_a);
    if (s1 != ExtendStatus::Trapped) {
      const auto & a_new = Ta.nodes[new_idx_a];

      int last_idx_b = -1;
      auto s2 = connect(Tb, a_new.x, a_new.y, last_idx_b);
      if (s2 == ExtendStatus::Reached && last_idx_b >= 0) {
        connect_a = new_idx_a;
        connect_b = last_idx_b;
        solved = true;
        break;
      }
    }

    // swap (RRT-Connect style)
    std::swap(Ta, Tb);
  }

  if (!solved) {
    throw std::runtime_error("RRT failed: timeout/iterations exceeded (no connection found).");
  }

  // Reconstruct path:
  // NOTE: because we swap trees, we must ensure which tree contains start vs goal at the end.
  // Easiest: detect which root is closer to start.
  auto rootDist = [&](const Tree & T) {
    const auto & r = T.nodes.front();
    return std::hypot(r.x - start.pose.position.x, r.y - start.pose.position.y);
  };

  // After potential swaps, Ta/Tb may not be (start-tree, goal-tree).
  // We'll rebuild two chains based on which root is start.
  Tree * Tstart = nullptr;
  Tree * Tgoal  = nullptr;
  int idx_start = -1;
  int idx_goal  = -1;

  if (rootDist(Ta) < rootDist(Tb)) {
    Tstart = &Ta; Tgoal = &Tb;
    idx_start = connect_a; idx_goal = connect_b;
  } else {
    Tstart = &Tb; Tgoal = &Ta;
    idx_start = connect_b; idx_goal = connect_a;
  }

  std::vector<std::pair<double,double>> pts;

  // backtrack start tree to root
  {
    std::vector<std::pair<double,double>> chain;
    int i = idx_start;
    while (i >= 0) {
      chain.push_back({Tstart->nodes[i].x, Tstart->nodes[i].y});
      i = Tstart->nodes[i].parent;
    }
    std::reverse(chain.begin(), chain.end());
    pts.insert(pts.end(), chain.begin(), chain.end());
  }

  // backtrack goal tree to root (from connection toward goal root)
  {
    std::vector<std::pair<double,double>> chain;
    int i = idx_goal;
    while (i >= 0) {
      chain.push_back({Tgoal->nodes[i].x, Tgoal->nodes[i].y});
      i = Tgoal->nodes[i].parent;
    }
    // chain currently ends at goal-root (goal). We want from connection -> goal, so keep as-is
    // But first element duplicates connection; drop it
    if (!chain.empty()) chain.erase(chain.begin());
    pts.insert(pts.end(), chain.begin(), chain.end());
  }

  // If end not within tolerance, append exact goal (optional)
  if (!pts.empty() && distToGoal(pts.back().first, pts.back().second) > goal_tolerance_m) {
    if (isSegmentValid(pts.back().first, pts.back().second, goal.pose.position.x, goal.pose.position.y)) {
      pts.push_back({goal.pose.position.x, goal.pose.position.y});
    }
  }

  // Optional smoothing: random shortcutting
  if (do_smooth && pts.size() >= 3) {
    std::uniform_int_distribution<int> ridx(0, (int)pts.size()-1);
    for (int k = 0; k < smooth_tries; ++k) {
      int i = ridx(rng), j = ridx(rng);
      if (i == j) continue;
      if (i > j) std::swap(i, j);
      if (j - i < 2) continue;

      const auto & A = pts[i];
      const auto & B = pts[j];
      if (isSegmentValid(A.first, A.second, B.first, B.second)) {
        // remove middle
        pts.erase(pts.begin() + i + 1, pts.begin() + j);
      }
      if ((int)pts.size() < 3) break;
      ridx = std::uniform_int_distribution<int>(0, (int)pts.size()-1);
    }
  }

  // Fill nav_msgs/Path
  path.poses.clear();
  path.poses.reserve(pts.size());

  auto yawToQuat = [&](double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion out;
    out.x = q.x(); out.y = q.y(); out.z = q.z(); out.w = q.w();
    return out;
  };

  for (size_t i = 0; i < pts.size(); ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = pts[i].first;
    p.pose.position.y = pts[i].second;
    p.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i + 1 < pts.size()) {
      yaw = std::atan2(pts[i+1].second - pts[i].second, pts[i+1].first - pts[i].first);
    } else {
      yaw = tf2::getYaw(goal.pose.orientation);
    }
    p.pose.orientation = yawToQuat(yaw);
    path.poses.push_back(p);
  }

  // Final sanity: ensure start->goal connectivity is obstacle free (optional strict check)
  if (path.poses.empty()) {
    throw std::runtime_error("RRT produced empty path (unexpected).");
  }

  return path;
}  // namespace my_nav2_planners

PLUGINLIB_EXPORT_CLASS(my_nav2_planners::MyPlanner, nav2_core::GlobalPlanner)
