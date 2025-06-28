// Copyright 2025 Lee Sheng Quan & contributors
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_dstar_lite_planner {

using nav2_util::declare_parameter_if_not_declared;

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

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer>,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
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

    declare_parameter_if_not_declared(node_.lock(), name_ + ".tolerance", rclcpp::ParameterValue(0.5));
    node_.lock()->get_parameter(name_ + ".tolerance", tolerance_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".allow_unknown", rclcpp::ParameterValue(true));
    node_.lock()->get_parameter(name_ + ".allow_unknown", allow_unknown_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".lethal_cost", rclcpp::ParameterValue(253));
    node_.lock()->get_parameter(name_ + ".lethal_cost", lethal_cost_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".neutral_cost", rclcpp::ParameterValue(50));
    node_.lock()->get_parameter(name_ + ".neutral_cost", neutral_cost_);
    RCLCPP_INFO(node_logger_, "D* Lite: neutral_cost_ parameter value: %d", neutral_cost_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".cost_factor", rclcpp::ParameterValue(0.8));
    node_.lock()->get_parameter(name_ + ".cost_factor", cost_factor_);

    declare_parameter_if_not_declared(node_.lock(), name_ + ".connectivity", rclcpp::ParameterValue(8));
    node_.lock()->get_parameter(name_ + ".connectivity", connectivity_);
    connectivity_ = connectivity_ == 4 ? 4 : 8;

    declare_parameter_if_not_declared(node_.lock(), name_ + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
    node_.lock()->get_parameter(name_ + ".use_final_approach_orientation", use_final_approach_orientation_);

    RCLCPP_INFO(node_logger_, "D* Lite planner configured (grid: %u × %u, res: %.2f m)", size_x_, size_y_, resolution_);

    allocateStructures();
    costmap_changed_ = true;
    dstar_initialized_ = false;
    last_goal_idx_ = -1;
  }

  void activate() override {}
  void deactivate() override {}
  void cleanup() override {}

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                               const geometry_msgs::msg::PoseStamped & goal) override
  {
    // Log robot and goal world positions and grid indices
    unsigned int sx, sy, gx, gy;
    bool start_in_map = worldToMap(start.pose.position.x, start.pose.position.y, sx, sy);
    bool goal_in_map = worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy);
    RCLCPP_INFO(node_logger_, "D* Lite: Robot world (%.3f, %.3f) -> grid (%s, %s) | Goal world (%.3f, %.3f) -> grid (%s, %s)",
                start.pose.position.x, start.pose.position.y,
                start_in_map ? std::to_string(sx).c_str() : "OUT",
                start_in_map ? std::to_string(sy).c_str() : "OUT",
                goal.pose.position.x, goal.pose.position.y,
                goal_in_map ? std::to_string(gx).c_str() : "OUT",
                goal_in_map ? std::to_string(gy).c_str() : "OUT");

    // Check if we can reuse the last plan
    if (!costmap_changed_ && start == last_start_ && goal == last_goal_) {
      RCLCPP_INFO(node_logger_, "D* Lite: REUSING CACHED PATH (no changes detected)");
      return last_path_;
    }

    nav_msgs::msg::Path path;
    path.header.stamp    = clock_->now();
    path.header.frame_id = frame_id_;

    if (!start_in_map || !goal_in_map) {
      bool found = false;
      for (double dx = -tolerance_; dx <= tolerance_; dx += resolution_) {
        for (double dy = -tolerance_; dy <= tolerance_; dy += resolution_) {
          double x = goal.pose.position.x + dx;
          double y = goal.pose.position.y + dy;
          unsigned int mx, my;
          if (worldToMap(x, y, mx, my)) {
            if (isTraversable(index(mx, my))) {
              gx = mx;
              gy = my;
              found = true;
              RCLCPP_WARN(node_logger_, "Goal fallback: using nearby traversable cell at (%.2f, %.2f)", x, y);
              break;
            }
          }
        }
        if (found) break;
      }
      if (!found) {
        throw std::runtime_error("Goal is out of bounds and no fallback found within tolerance.");
      }
    }

    start_idx_ = index(sx, sy);
    goal_idx_  = index(gx, gy);

    // Special case: start == goal
    if (start_idx_ == goal_idx_) {
      path.poses.push_back(start);
      if (!use_final_approach_orientation_) {
        path.poses.back().pose.orientation = goal.pose.orientation;
      }
      // Cache the results
      last_start_ = start;
      last_goal_ = goal;
      last_path_ = path;
      costmap_changed_ = false;
      RCLCPP_INFO(node_logger_, "D* Lite: Start equals goal - returning single point path");
      return path;
    }

    if (size_x_ != costmap_->getSizeInCellsX() || size_y_ != costmap_->getSizeInCellsY()) {
      size_x_ = costmap_->getSizeInCellsX();
      size_y_ = costmap_->getSizeInCellsY();
      allocateStructures();
      dstar_initialized_ = false;  // Force re-initialization
      RCLCPP_INFO(node_logger_, "D* Lite: Costmap size changed - forcing re-initialization");
    }

    // Only initialize D* Lite if the goal or map size changes
    bool need_initialization = !dstar_initialized_ || goal_idx_ != last_goal_idx_ ||
                              size_x_ != costmap_->getSizeInCellsX() ||
                              size_y_ != costmap_->getSizeInCellsY();

    if (need_initialization) {
      std::string reason = !dstar_initialized_ ? "not initialized" :
                           goal_idx_ != last_goal_idx_ ? "new goal" :
                           (size_x_ != costmap_->getSizeInCellsX() || size_y_ != costmap_->getSizeInCellsY()) ? "costmap size changed" :
                           "unknown";
      RCLCPP_WARN(node_logger_, "D* Lite: FULL INITIALIZATION (reason: %s)", reason.c_str());
      initialiseDStar();
      dstar_initialized_ = true;
      last_goal_idx_ = goal_idx_;
      last_start_idx_ = start_idx_;
      prev_start_idx_ = start_idx_;
    } else {
      // Use incremental updates - update robot position if it changed
      if (start_idx_ != last_start_idx_) {
        RCLCPP_INFO(node_logger_, "D* Lite: INCREMENTAL ROBOT POSITION UPDATE (robot moved)");
        updateRobotPosition(start);
        last_start_idx_ = start_idx_;
      } else {
        RCLCPP_INFO(node_logger_, "D* Lite: USING EXISTING STATE (no changes needed)");
      }
    }
    
    RCLCPP_DEBUG(node_logger_, "D* Lite: Computing shortest path (start: %d, goal: %d, km: %.2f)",
                 start_idx_, goal_idx_, km_);
    computeShortestPath();

    std::vector<int> grid_path;
    int current = start_idx_;
    size_t guard = 0;

    while (current != goal_idx_ && guard++ < size_x_ * size_y_) {
      grid_path.push_back(current);
      current = came_from_[current];
      if (current < 0) break;
    }
    grid_path.push_back(goal_idx_);

    if (grid_path.empty() || current != goal_idx_) {
      throw std::runtime_error("D* Lite failed to find a path");
    }

    for (size_t i = 0; i < grid_path.size(); ++i) {
      int cell = grid_path[i];
      geometry_msgs::msg::PoseStamped wp;
      wp.header = path.header;
      unsigned int mx = cell % size_x_;
      unsigned int my = cell / size_x_;
      mapToWorld(mx, my, wp.pose.position.x, wp.pose.position.y);

      // Set orientation for first pose
      if (i == 0) {
        wp.pose.orientation = start.pose.orientation;
      }

      path.poses.push_back(wp);
    }

    // Handle final orientation
    if (use_final_approach_orientation_) {
      if (path.poses.size() == 1) {
        path.poses.back().pose.orientation = start.pose.orientation;
      } else if (path.poses.size() >= 2) {
        auto & last = path.poses.back().pose.position;
        auto & before = path.poses.end()[-2].pose.position;
        double dx = last.x - before.x;
        double dy = last.y - before.y;
        double yaw = std::atan2(dy, dx);
        path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      }
    } else {
      path.poses.back().pose.orientation = goal.pose.orientation;
    }

    RCLCPP_INFO(node_logger_, "D* Lite: path with %zu points generated", path.poses.size());
    // Cache the results
    last_start_ = start;
    last_goal_ = goal;
    last_path_ = path;
    costmap_changed_ = false;
    RCLCPP_DEBUG(node_logger_, "D* Lite: Path cached for future reuse");
    return path;
  }

  // Public method to mark costmap as changed
  void notifyCostmapChanged() {
    RCLCPP_INFO(node_logger_, "D* Lite: COSTMAP CHANGED notification received");
    costmap_changed_ = true;
  }

  // Dynamic update methods for D* Lite
  void updateCostmapCell(unsigned int mx, unsigned int my, uint8_t new_cost) {
    if (mx >= size_x_ || my >= size_y_) {
      RCLCPP_WARN(node_logger_, "Costmap cell update out of bounds: (%u, %u)", mx, my);
      return;
    }
    
    int idx = index(mx, my);
    uint8_t old_cost = costmap_->getCharMap()[idx];
    
    // Only update if cost actually changed
    if (old_cost != new_cost) {
      costmap_->getCharMap()[idx] = new_cost;
      
      RCLCPP_INFO(node_logger_, "D* Lite: INCREMENTAL CELL UPDATE (%u, %u): %d -> %d", mx, my, old_cost, new_cost);
      
      // Update the affected vertex and its predecessors
      updateVertex(idx);
      
      // Update predecessors (cells that can reach this cell)
      std::vector<int> preds = getPredecessors(idx);
      RCLCPP_DEBUG(node_logger_, "D* Lite: Updating %zu predecessor vertices", preds.size());
      for (int pred : preds) {
        updateVertex(pred);
      }
      
      // Recompute shortest path incrementally
      RCLCPP_DEBUG(node_logger_, "D* Lite: Recomputing path after cell update");
      computeShortestPath();
    } else {
      RCLCPP_DEBUG(node_logger_, "D* Lite: Cell (%u, %u) cost unchanged (%d), skipping update", mx, my, old_cost);
    }
  }

  void updateCostmapRegion(unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y) {
    std::vector<int> affected_vertices;
    
    // Collect all affected vertices
    for (unsigned int y = min_y; y <= max_y && y < size_y_; ++y) {
      for (unsigned int x = min_x; x <= max_x && x < size_x_; ++x) {
        int idx = index(x, y);
        affected_vertices.push_back(idx);
      }
    }
    
    RCLCPP_INFO(node_logger_, "D* Lite: INCREMENTAL REGION UPDATE (%u,%u) to (%u,%u) - %zu vertices", 
                min_x, min_y, max_x, max_y, affected_vertices.size());
    
    // Update all affected vertices
    for (int idx : affected_vertices) {
      updateVertex(idx);
    }
    
    // Recompute shortest path incrementally
    RCLCPP_DEBUG(node_logger_, "D* Lite: Recomputing path after region update");
    computeShortestPath();
  }

  void updateRobotPosition(const geometry_msgs::msg::PoseStamped& new_start) {
    unsigned int new_sx, new_sy;
    if (!worldToMap(new_start.pose.position.x, new_start.pose.position.y, new_sx, new_sy)) {
      RCLCPP_WARN(node_logger_, "New robot position out of bounds");
      return;
    }
    int new_start_idx = index(new_sx, new_sy);
    float hval = (prev_start_idx_ >= 0) ? heuristic(prev_start_idx_, new_start_idx) : 0.0f;
    RCLCPP_WARN(node_logger_, "D* Lite: heuristic(%d, %d) = %.2f", prev_start_idx_, new_start_idx, hval);
    RCLCPP_INFO(node_logger_, "D* Lite: ROBOT POSITION UPDATE: %d -> %d (km: %.2f -> %.2f), world (%.3f, %.3f) -> grid (%u, %u)",
                prev_start_idx_, new_start_idx, km_, km_ + hval,
                new_start.pose.position.x, new_start.pose.position.y, new_sx, new_sy);
    if (prev_start_idx_ >= 0) {
      km_ += hval;
    }
    prev_start_idx_ = new_start_idx;
    start_idx_ = new_start_idx;
    updateVertex(start_idx_);
    RCLCPP_DEBUG(node_logger_, "D* Lite: Robot position updated to (%u, %u)", new_sx, new_sy);
  }

  // Helper method to get predecessors (cells that can reach this cell)
  std::vector<int> getPredecessors(int idx) const {
    unsigned int x = idx % size_x_;
    unsigned int y = idx / size_x_;
    std::vector<int> preds;
    
    const int dx4[4] = {1, 0, -1, 0};
    const int dy4[4] = {0, 1, 0, -1};
    const int dx8[4] = {1, 1, -1, -1};
    const int dy8[4] = {1, -1, 1, -1};
    
    // Check 4-connected neighbors
    for (int k = 0; k < 4; ++k) {
      int nx = static_cast<int>(x) + dx4[k];
      int ny = static_cast<int>(y) + dy4[k];
      if (nx >= 0 && ny >= 0 && nx < static_cast<int>(size_x_) && ny < static_cast<int>(size_y_)) {
        int nidx = index(nx, ny);
        if (isTraversable(nidx)) preds.push_back(nidx);
      }
    }
    
    // Check 8-connected neighbors if using 8-connectivity
    if (connectivity_ == 8) {
      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(x) + dx8[k];
        int ny = static_cast<int>(y) + dy8[k];
        if (nx >= 0 && ny >= 0 && nx < static_cast<int>(size_x_) && ny < static_cast<int>(size_y_)) {
          int nidx = index(nx, ny);
          if (isTraversable(nidx)) preds.push_back(nidx);
        }
      }
    }
    
    return preds;
  }

private:
  inline size_t index(unsigned int x, unsigned int y) const { return y * size_x_ + x; }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) return false;
    mx = static_cast<unsigned int>((wx - costmap_->getOriginX()) / resolution_);
    my = static_cast<unsigned int>((wy - costmap_->getOriginY()) / resolution_);
    return mx < size_x_ && my < size_y_;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
  {
    wx = costmap_->getOriginX() + (mx + 0.5) * resolution_;
    wy = costmap_->getOriginY() + (my + 0.5) * resolution_;
  }

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
    if (c >= lethal_cost_) return false;
    if (!allow_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
    return true;
  }

  float traversalCost(int idx_from, int idx_to) const
  {
    uint8_t c = costmap_->getCharMap()[idx_to];
    float cost = neutral_cost_ + cost_factor_ * static_cast<float>(c);
    unsigned int x1 = idx_from % size_x_, y1 = idx_from / size_x_;
    unsigned int x2 = idx_to % size_x_, y2 = idx_to / size_x_;
    if (x1 != x2 && y1 != y2) cost *= std::sqrt(2.0f);
    return cost;
  }

  // Fix heuristic formula
  float heuristic(int a, int b) const {
    int ax = a % size_x_, ay = a / size_x_;
    int bx = b % size_x_, by = b / size_x_;
    int dx = std::abs(ax - bx), dy = std::abs(ay - by);
    float D = static_cast<float>(neutral_cost_);
    float D2 = std::sqrt(2.0f) * D;
    return D * (dx + dy) + (D2 - 2.0f * D) * std::min(dx, dy);
  }

  void allocateStructures()
  {
    size_t N = size_x_ * size_y_;
    g_.assign(N, INF_);
    rhs_.assign(N, INF_);
    in_open_.assign(N, false);
    came_from_.assign(N, -1); // NEW
  }

  void initialiseDStar()
  {
    RCLCPP_WARN(node_logger_, "D* Lite: initialiseDStar() called, resetting km_ to 0");
    km_ = 0.0;
    std::fill(g_.begin(), g_.end(), INF_);
    std::fill(rhs_.begin(), rhs_.end(), INF_);
    std::fill(in_open_.begin(), in_open_.end(), false);
    std::fill(came_from_.begin(), came_from_.end(), -1); // NEW
    while (!open_.empty()) open_.pop();

    rhs_[goal_idx_] = 0.0;
    pushOpen(goal_idx_, calculateKey(goal_idx_));
  }

  void updateVertex(int idx)
  {
    if (idx != goal_idx_) {
      float min_rhs = INF_;
      int best_pred = -1;
      for (int s : getSuccessors(idx)) {
        float val = traversalCost(idx, s) + g_[s];
        if (val < min_rhs) {
          min_rhs = val;
          best_pred = s;
        }
      }
      rhs_[idx] = min_rhs;
      came_from_[idx] = best_pred; // NEW
    }

    if (in_open_[idx]) in_open_[idx] = false;

    if (g_[idx] != rhs_[idx]) {
      pushOpen(idx, calculateKey(idx));
    }
  }

  void computeShortestPath()
  {
    int vertices_processed = 0;
    int max_iterations = size_x_ * size_y_; // Safety guard
    
    while (!open_.empty() &&
          (open_.top().key < calculateKey(start_idx_) || rhs_[start_idx_] != g_[start_idx_])) {
      
      if (++vertices_processed > max_iterations) {
        RCLCPP_WARN(node_logger_, "D* Lite: computeShortestPath exceeded max iterations (%d)", max_iterations);
        break;
      }
      
      PQItem top = open_.top(); open_.pop();
      if (!in_open_[top.idx]) continue;
      in_open_[top.idx] = false;

      Key new_key = calculateKey(top.idx);
      if (top.key < new_key) {
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
    
    RCLCPP_DEBUG(node_logger_, "D* Lite: computeShortestPath processed %d vertices, open list size: %zu", 
                 vertices_processed, open_.size());
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

  struct PQItem {
    Key key;
    int idx;
  };
  struct PQCmp {
    bool operator()(const PQItem & a, const PQItem & b) const { return b.key < a.key; }
  };

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger node_logger_{rclcpp::get_logger("DStarLite")};
  rclcpp::Clock::SharedPtr clock_;

  std::string name_, frame_id_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ {nullptr};

  unsigned int size_x_ {0}, size_y_ {0};
  double resolution_ {0.05};

  double tolerance_ {0.5};
  bool allow_unknown_ {true};
  int lethal_cost_ {253}, neutral_cost_ {50};
  double cost_factor_ {0.8};
  int connectivity_ {8};
  bool use_final_approach_orientation_ {false};

  int start_idx_ {-1}, goal_idx_ {-1};
  float km_ {0.0};
  static constexpr float INF_ = std::numeric_limits<float>::infinity();

  std::vector<float> g_, rhs_;
  std::vector<bool> in_open_;
  std::vector<int> came_from_; // NEW
  std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> open_;
  std::unordered_map<int, Key> key_hash_;

  bool costmap_changed_ {true};
  geometry_msgs::msg::PoseStamped last_start_;
  geometry_msgs::msg::PoseStamped last_goal_;
  nav_msgs::msg::Path last_path_;

  bool dstar_initialized_ {false};
  int last_goal_idx_ {-1};
  int last_start_idx_ {-1};
  int prev_start_idx_ {-1};
};

}  // namespace nav2_dstar_lite_planner

PLUGINLIB_EXPORT_CLASS(nav2_dstar_lite_planner::DStarLitePlanner, nav2_core::GlobalPlanner)