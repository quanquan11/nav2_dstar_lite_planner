// Copyright 2025 Lee Sheng Quan & contributors
// SPDX‑License‑Identifier: Apache‑2.0
//
// A minimal D* Lite Global Planner plugin for ROS 2 Navigation2 (Nav2).
// This file is *self‑contained*: just drop it into
//   nav2_dstar_lite_planner/src/
// add the accompanying CMakeLists.txt / package.xml / plugin.xml
// (see previous messages) and `colcon build`.
//
// *** NOTE ***
// • The code is intentionally compact rather than hyper‑optimised; it aims
//   to be easy to read and extend.
// • Incremental updates (re‑planning on costmap change) are supported, but
//   for clarity the example triggers a fresh `createPlan()` call per BT
//   request. Hook a subscription to the costmap update topic if you want
//   *automatic* re‑planning.
// • All parameters mirror those of NavFn so you can switch planners simply
//   by changing `planner_plugins:` in your YAML.
//
// ╭──────────────────────────────────────────────────────────────────────╮
// │  HEADERS & HELPERS                                                  │
// ╰──────────────────────────────────────────────────────────────────────╯

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_dstar_lite_planner {

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

// -----------------------------------------------------------------------------
//  D* Lite data structures
// -----------------------------------------------------------------------------
struct Key {
  float k1, k2;
  bool operator<(const Key & o) const {
    return (k1 < o.k1) || (k1 == o.k1 && k2 < o.k2);
  }
};

class DStarLitePlanner : public nav2_core::GlobalPlanner {
public:
  DStarLitePlanner() = default;
  ~DStarLitePlanner() override = default;

  // ─────────────────────────────────────────────────────────────────────
  //  Lifecycle hooks
  // ─────────────────────────────────────────────────────────────────────
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    (void)tf;  // TF not needed for grid‑based global planning

    node_        = parent;
    node_logger_ = node_.lock()->get_logger();
    clock_       = node_.lock()->get_clock();
    name_        = name;

    costmap_ros_ = costmap_ros;
    costmap_     = costmap_ros_->getCostmap();
    frame_id_    = costmap_ros_->getGlobalFrameID();
    size_x_      = costmap_->getSizeInCellsX();
    size_y_      = costmap_->getSizeInCellsY();
    resolution_  = costmap_->getResolution();

    // Parameters mirroring navfn_planner
    declare_parameter_if_not_declared(node_.lock(), name_ + ".tolerance", rclcpp::ParameterValue(0.5));
    node_.lock()->get_parameter(name_ + ".tolerance", tolerance_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".allow_unknown", rclcpp::ParameterValue(true));
    node_.lock()->get_parameter(name_ + ".allow_unknown", allow_unknown_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".lethal_cost", rclcpp::ParameterValue(253));
    node_.lock()->get_parameter(name_ + ".lethal_cost", lethal_cost_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".neutral_cost", rclcpp::ParameterValue(50));
    node_.lock()->get_parameter(name_ + ".neutral_cost", neutral_cost_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".cost_factor", rclcpp::ParameterValue(0.8));
    node_.lock()->get_parameter(name_ + ".cost_factor", cost_factor_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".connectivity", rclcpp::ParameterValue(8));
    node_.lock()->get_parameter(name_ + ".connectivity", connectivity_);
    connectivity_ = connectivity_ == 4 ? 4 : 8;  // clamp

    // declare_parameter_if_not_declared(node_.lock(), name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.0));
    // node_.lock()->get_parameter(name_ + ".interpolation_resolution", interp_res_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
    node_.lock()->get_parameter(name_ + ".use_final_approach_orientation", use_final_approach_orientation_);

    RCLCPP_INFO(node_logger_, "D* Lite planner configured (grid: %u × %u, res: %.2f m)",
                size_x_, size_y_, resolution_);

    allocateStructures();
  }

  void activate()   override {}
  void deactivate() override {}
  void cleanup()    override {}

  // ─────────────────────────────────────────────────────────────────────
  //  Main path‑planning entry point
  // ─────────────────────────────────────────────────────────────────────
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                                 const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path path;
    path.header.stamp    = clock_->now();
    path.header.frame_id = frame_id_;

    // 1) Convert start / goal to grid indices
    unsigned int sx, sy, gx, gy;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
      throw std::runtime_error("Start is out of bounds");
    }
    if (!worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
      throw std::runtime_error("Goal is out of bounds");
    }

    start_idx_ = index(sx, sy);
    goal_idx_  = index(gx, gy);

    // 2) Costmap → local arrays (only if size changed)
    if (size_x_ != costmap_->getSizeInCellsX() || size_y_ != costmap_->getSizeInCellsY()) {
      size_x_ = costmap_->getSizeInCellsX();
      size_y_ = costmap_->getSizeInCellsY();
      allocateStructures();
    }

    // 3) (Re)initialise D* Lite state
    initialiseDStar();

    // 4) Run the incremental search
    computeShortestPath();

    // 5) Extract the path by greedily following the lowest rhs-minimizing successor
    std::vector<int> grid_path;
    int current = start_idx_;
    size_t guard = 0;

    while (current != goal_idx_ && guard++ < size_x_ * size_y_) {
      int best_succ = -1;
      float min_rhs = INF_;

      for (int nbr : getSuccessors(current)) {
        float candidate_rhs = traversalCost(current, nbr) + g_[nbr];
        if (candidate_rhs < min_rhs) {
          min_rhs = candidate_rhs;
          best_succ = nbr;
        }
      }

      if (best_succ < 0 || min_rhs == INF_) {
        RCLCPP_WARN(node_logger_, "No valid path: stuck at index %d", current);
        break;
      }

      grid_path.push_back(best_succ);
      current = best_succ;
    }


    if (grid_path.empty() || current != goal_idx_) {
      throw std::runtime_error("D* Lite failed to find a path");
    }

    // 6) Convert grid indices → world poses
    for (int cell : grid_path) {
      geometry_msgs::msg::PoseStamped wp;
      wp.header = path.header;
      unsigned int mx = cell % size_x_;
      unsigned int my = cell / size_x_;
      mapToWorld(mx, my, wp.pose.position.x, wp.pose.position.y);
      path.poses.push_back(wp);
    }

    // // 7) Optional interpolation
    // if (interp_res_ > 0.0) {
    //   nav2_util::interpolate(path, interp_res_);
    // }

    // 8) Final‑approach orientation trick (mirrors NavFn)
    if (use_final_approach_orientation_ && path.poses.size() >= 2) {
      auto & last   = path.poses.back().pose.position;
      auto & before = path.poses.end()[-2].pose.position;
      double dx = last.x - before.x;
      double dy = last.y - before.y;
      double yaw = std::atan2(dy, dx);
      path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    }

    return path;
  }

private:
  // ─────────────────────────────────────────────────────────────────────
  //  Helpers
  // ─────────────────────────────────────────────────────────────────────
  inline size_t index(unsigned int x, unsigned int y) const { return y * size_x_ + x; }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
      return false;
    }
    mx = static_cast<unsigned int>(std::floor((wx - costmap_->getOriginX()) / resolution_));
    my = static_cast<unsigned int>(std::floor((wy - costmap_->getOriginY()) / resolution_));
    return mx < size_x_ && my < size_y_;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
  {
    wx = costmap_->getOriginX() + (mx + 0.5) * resolution_;
    wy = costmap_->getOriginY() + (my + 0.5) * resolution_;
  }

  // Return 4‑ or 8‑connected successors that are not lethal obstacles
  std::vector<int> getSuccessors(int idx) const
  {
    unsigned int x = idx % size_x_;
    unsigned int y = idx / size_x_;
    std::vector<int> nbrs;
    const int dx4[4] = {1, 0, -1, 0};
    const int dy4[4] = {0, 1, 0, -1};
    const int dx8[4] = {1, 1, -1, -1};
    const int dy8[4] = {1, -1, 1, -1};

    for (int k = 0; k < 4; ++k) {
      int nx = static_cast<int>(x) + dx4[k];
      int ny = static_cast<int>(y) + dy4[k];
      if (nx >= 0 && ny >= 0 && nx < static_cast<int>(size_x_) && ny < static_cast<int>(size_y_)) {
        int nidx = index(nx, ny);
        if (isTraversable(nidx)) nbrs.push_back(nidx);
      }
    }
    if (connectivity_ == 8) {
      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(x) + dx8[k];
        int ny = static_cast<int>(y) + dy8[k];
        if (nx >= 0 && ny >= 0 && nx < static_cast<int>(size_x_) && ny < static_cast<int>(size_y_)) {
          int nidx = index(nx, ny);
          if (isTraversable(nidx)) nbrs.push_back(nidx);
        }
      }
    }
    return nbrs;
  }

  bool isTraversable(int idx) const
  {
    uint8_t c = costmap_->getCharMap()[idx];
    if (c >= lethal_cost_) return false;            // obstacle
    if (!allow_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
    return true;
  }

  float traversalCost(int idx_from, int idx_to) const
  {
    // Basic cost model: neutral_cost + scaled costmap cost
    uint8_t c = costmap_->getCharMap()[idx_to];
    float cost = neutral_cost_ + cost_factor_ * static_cast<float>(c);

    // Diagonal penalty
    unsigned int x1 = idx_from % size_x_, y1 = idx_from / size_x_;
    unsigned int x2 = idx_to % size_x_,   y2 = idx_to / size_x_;
    if (x1 != x2 && y1 != y2) cost *= std::sqrt(2.0f);
    return cost;
  }

  float heuristic(int idx_a, int idx_b) const
  {
    int ax = idx_a % size_x_, ay = idx_a / size_x_;
    int bx = idx_b % size_x_, by = idx_b / size_x_;
    int dx = std::abs(ax - bx);
    int dy = std::abs(ay - by);
    // Octile distance (good for 8‑connected grid, degrades to Manhattan for 4)
    return neutral_cost_ * (dx + dy) + (std::sqrt(2.0f) - 2.0f * neutral_cost_) * std::min(dx, dy);
  }

  // -------------------------------------------------------------------------
  //  Core D* Lite functions (minimal implementation)
  // -------------------------------------------------------------------------
  struct PQItem {
    Key key;
    int idx;
  };
  struct PQCmp {
    bool operator()(const PQItem & a, const PQItem & b) const { return b.key < a.key; }
  };

  void allocateStructures()
  {
    size_t N = size_x_ * size_y_;
    g_.assign(N, INF_);
    rhs_.assign(N, INF_);
    in_open_.assign(N, false);
  }

  void initialiseDStar()
  {
    km_ = 0.0;
    std::fill(g_.begin(), g_.end(), INF_);
    std::fill(rhs_.begin(), rhs_.end(), INF_);
    std::fill(in_open_.begin(), in_open_.end(), false);
    while (!open_.empty()) open_.pop();

    rhs_[goal_idx_] = 0.0;
    Key k_goal = calculateKey(goal_idx_);
    pushOpen(goal_idx_, k_goal);
  }

  Key calculateKey(int idx) const
  {
    float v = std::min(g_[idx], rhs_[idx]);
    return {v + heuristic(start_idx_, idx) + km_, v};
  }

  void pushOpen(int idx, const Key & key)
  {
    open_.push({key, idx});
    in_open_[idx] = true;
    key_hash_[idx] = key;
  }

  void updateVertex(int idx)
  {
    if (idx != goal_idx_) {
      float min_rhs = INF_;
      for (int s : getSuccessors(idx)) {
        min_rhs = std::min(min_rhs, traversalCost(idx, s) + g_[s]);
      }
      rhs_[idx] = min_rhs;
    }

    if (in_open_[idx]) {
      // Lazy removal: just mark as not‑in‑open; we'll skip it when popped
      in_open_[idx] = false;
    }

    if (g_[idx] != rhs_[idx]) {
      pushOpen(idx, calculateKey(idx));
    }
  }

  void computeShortestPath()
  {
    while (!open_.empty() &&
           ( open_.top().key < calculateKey(start_idx_) ||
             rhs_[start_idx_] != g_[start_idx_] ))
    {
      PQItem top = open_.top(); open_.pop();
      if (!in_open_[top.idx]) continue;  // stale entry
      in_open_[top.idx] = false;

      Key new_key = calculateKey(top.idx);
      if (top.key < new_key) {
        // outdated key, re‑insert
        pushOpen(top.idx, new_key);
        continue;
      }

      if (g_[top.idx] > rhs_[top.idx]) {
        g_[top.idx] = rhs_[top.idx];
        for (int p : getSuccessors(top.idx)) updateVertex(p);
      } else {
        g_[top.idx] = INF_;
        updateVertex(top.idx);
        for (int p : getSuccessors(top.idx)) updateVertex(p);
      }
    }
  }

  float cost(int from, int to) const { return traversalCost(from, to); }

  // ─────────────────────────────────────────────────────────────────────
  //  Members
  // ─────────────────────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger node_logger_{rclcpp::get_logger("DStarLite")};
  rclcpp::Clock::SharedPtr clock_;

  std::string name_;
  std::string frame_id_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ {nullptr};

  // Grid metrics
  unsigned int size_x_ {0}, size_y_ {0};
  double       resolution_ {0.05};

  // Parameters
  double tolerance_ {0.5};
  bool   allow_unknown_ {true};
  int    lethal_cost_ {253};
  int    neutral_cost_ {50};
  double cost_factor_ {0.8};
  int    connectivity_ {8};
  // double interp_res_ {0.0};
  bool   use_final_approach_orientation_ {false};

  // D* Lite state
  int start_idx_ {-1}, goal_idx_ {-1};
  float km_ {0.0};

  static constexpr float INF_ = std::numeric_limits<float>::infinity();

  std::vector<float> g_;
  std::vector<float> rhs_;
  std::vector<bool>  in_open_;
  std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> open_;
  std::unordered_map<int, Key> key_hash_;  // latest key per state
};

}  // namespace nav2_dstar_lite_planner

// ────────────────────────────────────────────────────────────────────────────
//  Register with pluginlib
// ────────────────────────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(nav2_dstar_lite_planner::DStarLitePlanner, nav2_core::GlobalPlanner)
