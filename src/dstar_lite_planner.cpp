// Copyright 2025 Lee Sheng Quan & contributors
// SPDX-License-Identifier: Apache-2.0

#include "nav2_dstar_lite_planner/dstar_lite_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include <chrono>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_dstar_lite_planner {

using nav2_util::declare_parameter_if_not_declared;

void DStarLitePlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer>,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Failed to lock parent node");
    }
    
    node_        = parent;
    node_logger_ = node->get_logger();
    clock_       = node->get_clock();
    name_        = name;

    costmap_ros_ = costmap_ros;
    costmap_     = costmap_ros_->getCostmap();
    frame_id_    = costmap_ros_->getGlobalFrameID();
    size_x_      = costmap_->getSizeInCellsX();
    size_y_      = costmap_->getSizeInCellsY();
    resolution_  = costmap_->getResolution();

    declare_parameter_if_not_declared(node, name_ + ".tolerance", rclcpp::ParameterValue(0.5));
    node->get_parameter(name_ + ".tolerance", tolerance_);

    declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

    declare_parameter_if_not_declared(node, name_ + ".lethal_cost", rclcpp::ParameterValue(253));
    node->get_parameter(name_ + ".lethal_cost", lethal_cost_);

    declare_parameter_if_not_declared(node, name_ + ".neutral_cost", rclcpp::ParameterValue(50));
    node->get_parameter(name_ + ".neutral_cost", neutral_cost_);

    declare_parameter_if_not_declared(node, name_ + ".cost_factor", rclcpp::ParameterValue(0.8));
    node->get_parameter(name_ + ".cost_factor", cost_factor_);

    declare_parameter_if_not_declared(node, name_ + ".connectivity", rclcpp::ParameterValue(8));
    node->get_parameter(name_ + ".connectivity", connectivity_);
    connectivity_ = connectivity_ == 4 ? 4 : 8;

    declare_parameter_if_not_declared(node, name_ + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
    node->get_parameter(name_ + ".use_final_approach_orientation", use_final_approach_orientation_);

    // Search tuning parameters
    declare_parameter_if_not_declared(node, name_ + ".max_search_ratio_initial", rclcpp::ParameterValue(0.5));
    node->get_parameter(name_ + ".max_search_ratio_initial", max_search_ratio_initial_);
    declare_parameter_if_not_declared(node, name_ + ".max_search_ratio_incremental", rclcpp::ParameterValue(0.2));
    node->get_parameter(name_ + ".max_search_ratio_incremental", max_search_ratio_incremental_);

    // Precompute constants for performance
    sqrt2_ = std::sqrt(2.0f);
    sqrt2_cost_ = sqrt2_ * neutral_cost_;

    // NavFn-inspired time-slicing parameters
    declare_parameter_if_not_declared(node, name_ + ".max_planning_time_ms", rclcpp::ParameterValue(100.0));
    node->get_parameter(name_ + ".max_planning_time_ms", max_planning_time_ms_);
    declare_parameter_if_not_declared(node, name_ + ".cycle_batch_size", rclcpp::ParameterValue(256));
    node->get_parameter(name_ + ".cycle_batch_size", cycle_batch_size_);
    declare_parameter_if_not_declared(node, name_ + ".terminal_check_interval", rclcpp::ParameterValue(10));
    node->get_parameter(name_ + ".terminal_check_interval", terminal_check_interval_param_);

    RCLCPP_INFO(node_logger_, "D* Lite planner configured (grid: %u × %u, res: %.2f m)", size_x_, size_y_, resolution_);

    allocateStructures();
    costmap_changed_ = true;
    dstar_initialized_ = false;
    last_goal_idx_ = -1;
  }

void DStarLitePlanner::activate() {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node_logger_, "D* Lite planner activated");
  }
}

void DStarLitePlanner::deactivate() {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node_logger_, "D* Lite planner deactivated");
  }
}

void DStarLitePlanner::cleanup() {
  if (auto node = node_.lock()) {
    RCLCPP_INFO(node_logger_, "D* Lite planner cleaned up");
  }
}

nav_msgs::msg::Path DStarLitePlanner::createPlan(const geometry_msgs::msg::PoseStamped & start,
                               const geometry_msgs::msg::PoseStamped & goal)
{
    RCLCPP_INFO(node_logger_,
      "createPlan called: start(%.3f, %.3f) goal(%.3f, %.3f) frame=%s map=%ux%u res=%.3f",
      start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y,
      frame_id_.c_str(), size_x_, size_y_, resolution_);
    auto planning_start = std::chrono::high_resolution_clock::now();

    nav_msgs::msg::Path path;
    if (checkCachedPath(start, goal, planning_start, path)) {
      return path;
    }

    path.header.stamp    = clock_->now();
    path.header.frame_id = frame_id_;

    // Ensure we are using the latest costmap dimensions before any world->map checks
    updateCostmapSizeIfChanged();
    // Cache the underlying charmap pointer for hot loops in this planning call
    cmap_cache_ = costmap_->getCharMap();

    unsigned int sx, sy, gx, gy;
    if (!validateAndConvertCoordinates(start, goal, sx, sy, gx, gy)) {
      throw std::runtime_error("Start/Goal is out of bounds and no fallback found within tolerance.");
    }

    start_idx_ = index(sx, sy);
    goal_idx_  = index(gx, gy);

    // Trace chosen grid cells and costs
    uint8_t sc = costmap_->getCharMap()[start_idx_];
    uint8_t gc = costmap_->getCharMap()[goal_idx_];
    RCLCPP_INFO(node_logger_, "Pinned cells: start_idx=%d(cost=%u) goal_idx=%d(cost=%u)",
                static_cast<int>(start_idx_), static_cast<unsigned>(sc),
                static_cast<int>(goal_idx_), static_cast<unsigned>(gc));

    if (start_idx_ == goal_idx_) {
      return handleStartEqualsGoal(start, goal, planning_start);
    }

    performDStarPlanning(start);

    std::vector<int> grid_path = extractPath();
    if (grid_path.empty()) {
      throw std::runtime_error("D* Lite failed to find a path");
    }

    path = convertGridPathToWorld(grid_path, start, goal);

    cacheResults(start, goal, path);
    logPlanningMetrics(planning_start, path);

    return path;
}

void DStarLitePlanner::notifyCostmapChanged() {
    costmap_changed_ = true;
  }

void DStarLitePlanner::updateCostmapCell(unsigned int mx, unsigned int my, uint8_t new_cost) {
    if (mx >= size_x_ || my >= size_y_) return;
    
    int idx = index(mx, my);
    uint8_t old_cost = costmap_->getCharMap()[idx];
    
    if (old_cost != new_cost) {
      costmap_->getCharMap()[idx] = new_cost;
      updateVertex(idx);
      
      // Update predecessors efficiently
      for (int pred : getPredecessors(idx)) {
        updateVertex(pred);
      }
      
      computeShortestPath();
    }
  }

void DStarLitePlanner::updateCostmapRegion(unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y) {
    std::vector<int> affected_vertices;
    affected_vertices.reserve((max_x - min_x + 1) * (max_y - min_y + 1));
    
    for (unsigned int y = min_y; y <= max_y && y < size_y_; ++y) {
      for (unsigned int x = min_x; x <= max_x && x < size_x_; ++x) {
        affected_vertices.push_back(index(x, y));
      }
    }
    
    for (int idx : affected_vertices) {
      updateVertex(idx);
    }
    
    computeShortestPath();
  }

void DStarLitePlanner::updateRobotPosition(const geometry_msgs::msg::PoseStamped& new_start) {
    unsigned int new_sx, new_sy;
    if (!worldToMap(new_start.pose.position.x, new_start.pose.position.y, new_sx, new_sy)) {
      return;
    }
    
    int new_start_idx = index(new_sx, new_sy);
    
    // FIXED: Correct D* Lite km update: km = km + h(s_last, s_start)
    // This should be heuristic FROM last start TO current start
    if (prev_start_idx_ >= 0 && prev_start_idx_ != new_start_idx) {
      float hval = heuristic(prev_start_idx_, new_start_idx);
      km_ += hval;
      RCLCPP_INFO(node_logger_, "D*LITE: Robot moved from %d to %d, km increased by %.3f (total km=%.3f)", 
                  prev_start_idx_, new_start_idx, hval, km_);
    }
    
    // Update positions
    prev_start_idx_ = start_idx_;  // Store current as previous
    start_idx_ = new_start_idx;    // Update to new position
    
    // No need to updateVertex(start_idx_) here - D* Lite maintains queue from before
}

inline int DStarLitePlanner::index(unsigned int x, unsigned int y) const {
  // Safe integer index computation
  return static_cast<int>(y) * static_cast<int>(size_x_) + static_cast<int>(x);
}

bool DStarLitePlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) return false;
    mx = static_cast<unsigned int>((wx - costmap_->getOriginX()) / resolution_);
    my = static_cast<unsigned int>((wy - costmap_->getOriginY()) / resolution_);
    return mx < size_x_ && my < size_y_;
  }

void DStarLitePlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
  {
    wx = costmap_->getOriginX() + (mx + 0.5) * resolution_;
    wy = costmap_->getOriginY() + (my + 0.5) * resolution_;
  }

std::vector<int> DStarLitePlanner::getSuccessors(int idx) const
  {
    int sx = static_cast<int>(size_x_);
    int x = idx % sx;
    int y = idx / sx;
    std::vector<int> nbrs;
    nbrs.reserve(8); // Pre-allocate for 8-connectivity
    
    const int dx4[4] = {1, 0, -1, 0};
    const int dy4[4] = {0, 1, 0, -1};
    const int dx8[4] = {1, 1, -1, -1};
    const int dy8[4] = {1, -1, 1, -1};

    // 4-connected neighbors
    for (int k = 0; k < 4; ++k) {
      int nx = static_cast<int>(x) + dx4[k];
      int ny = static_cast<int>(y) + dy4[k];
      if (nx >= 0 && ny >= 0 && nx < static_cast<int>(size_x_) && ny < static_cast<int>(size_y_)) {
        int nidx = index(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
        if (isTraversable(nidx)) nbrs.push_back(nidx);
      }
    }
    
    // 8-connected neighbors with NavFn-style no corner-cutting
    if (connectivity_ == 8) {
      for (int k = 0; k < 4; ++k) {
        int nx = static_cast<int>(x) + dx8[k];
        int ny = static_cast<int>(y) + dy8[k];
        if (nx >= 0 && ny >= 0 && nx < static_cast<int>(size_x_) && ny < static_cast<int>(size_y_)) {
          int nidx = index(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
          // require adjacent cardinals to be traversable to avoid cutting corners
          bool ok = false;
          if (nx > static_cast<int>(x) && ny > static_cast<int>(y)) {
            // down-right: need right and down
            ok = isTraversable(index(static_cast<unsigned int>(x + 1), static_cast<unsigned int>(y))) &&
                 isTraversable(index(static_cast<unsigned int>(x), static_cast<unsigned int>(y + 1)));
          } else if (nx < static_cast<int>(x) && ny > static_cast<int>(y)) {
            // down-left: need left and down
            ok = isTraversable(index(static_cast<unsigned int>(x - 1), static_cast<unsigned int>(y))) &&
                 isTraversable(index(static_cast<unsigned int>(x), static_cast<unsigned int>(y + 1)));
          } else if (nx > static_cast<int>(x) && ny < static_cast<int>(y)) {
            // up-right: need right and up
            ok = isTraversable(index(static_cast<unsigned int>(x + 1), static_cast<unsigned int>(y))) &&
                 isTraversable(index(static_cast<unsigned int>(x), static_cast<unsigned int>(y - 1)));
          } else if (nx < static_cast<int>(x) && ny < static_cast<int>(y)) {
            // up-left: need left and up
            ok = isTraversable(index(static_cast<unsigned int>(x - 1), static_cast<unsigned int>(y))) &&
                 isTraversable(index(static_cast<unsigned int>(x), static_cast<unsigned int>(y - 1)));
          }
          if (ok && isTraversable(nidx)) nbrs.push_back(nidx);
        }
      }
    }
    return nbrs;
  }

std::vector<int> DStarLitePlanner::getPredecessors(int idx) const {
    return getSuccessors(idx); // For grid-based navigation, predecessors = successors
  }

bool DStarLitePlanner::isTraversable(int idx) const
  {
    // Bounds guard
    int N = static_cast<int>(size_x_ * size_y_);
    if (idx < 0 || idx >= N) return false;

    uint8_t c = cmap_cache_ ? cmap_cache_[idx] : costmap_->getCharMap()[idx];
    if (c >= lethal_cost_) return false;
    if (!allow_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) return false;
    return true;
  }

float DStarLitePlanner::getCostMultiplier(int idx) const {
    uint8_t cost = cmap_cache_ ? cmap_cache_[idx] : costmap_->getCharMap()[idx];

    // NavFn-style cost policy
    if (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_) {
      return static_cast<float>(neutral_cost_);
    }
    if (cost >= lethal_cost_) {
      return 1e6f; // effectively blocked
    }

    // Keep free space at neutral cost; scale only above neutral
    if (cost <= neutral_cost_) {
      return static_cast<float>(neutral_cost_);
    }

    return static_cast<float>(neutral_cost_) + static_cast<float>(cost_factor_) *
           static_cast<float>(cost - neutral_cost_);
  }

float DStarLitePlanner::traversalCost(int idx_from, int idx_to) const
  {
    float cost = getCostMultiplier(idx_to);
    
    int sx = static_cast<int>(size_x_);
    int x1 = idx_from % sx, y1 = idx_from / sx;
    int x2 = idx_to % sx, y2 = idx_to / sx;
    
    if (x1 != x2 && y1 != y2) cost *= sqrt2_;
    return cost;
  }

float DStarLitePlanner::heuristic(int a, int b) const {
    int sx = static_cast<int>(size_x_);
    int ax = a % sx, ay = a / sx;
    int bx = b % sx, by = b / sx;
    int dx = std::abs(ax - bx), dy = std::abs(ay - by);
    
    // NavFn-inspired: slightly optimistic heuristic for faster initial search
    float h = neutral_cost_ * (dx + dy) + (sqrt2_cost_ - 2.0f * neutral_cost_) * std::min(dx, dy);
    
    // Scale down slightly for initial search to encourage exploration
    if (!dstar_initialized_) {
        h *= 0.95f; // Slightly optimistic for faster initial convergence
    }
    
    return h;
  }

float DStarLitePlanner::heuristicCached(int a, int b) const
  {
    // a is expected to be current start_idx_ in this planner
    if (heuristic_cache_start_idx_ != a || heuristic_cache_vec_.size() != size_x_ * size_y_) {
      heuristic_cache_vec_.assign(size_x_ * size_y_, -1.0f);
      heuristic_cache_start_idx_ = a;
    }
    float v = heuristic_cache_vec_[b];
    if (v >= 0.0f) return v;
    float h = heuristic(a, b);
    heuristic_cache_vec_[b] = h;
    return h;
  }

void DStarLitePlanner::allocateStructures()
  {
    size_t N = size_x_ * size_y_;
    
    // NavFn-style: check if reallocation is needed
    if (g_.size() != N) {
      g_.assign(N, INF_);
      rhs_.assign(N, INF_);
      in_open_.assign(N, false);
      came_from_.assign(N, -1);
      
      // Pre-allocate performance optimization structures
      successor_cache_.reserve(8);
      heuristic_cache_vec_.assign(N, -1.0f);
      heuristic_cache_start_idx_ = -1;
      
      // Clear priority queue efficiently
      std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> empty;
      open_.swap(empty);
      key_hash_.clear();
      key_hash_.reserve(std::min(N / 50, static_cast<size_t>(10000)));
    }
  }

bool DStarLitePlanner::checkCachedPath(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  std::chrono::high_resolution_clock::time_point planning_start,
  nav_msgs::msg::Path& path)
{
  if (!costmap_changed_ && start == last_start_ && goal == last_goal_) {
    path = last_path_;
    path.header.stamp = clock_->now();
    logPlanningMetrics(planning_start, path);
    return true;
  }
  return false;
}

bool DStarLitePlanner::validateAndConvertCoordinates(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  unsigned int& sx, unsigned int& sy,
  unsigned int& gx, unsigned int& gy)
{
  // Log costmap bounds and inputs once to aid debugging
  const double origin_x = costmap_->getOriginX();
  const double origin_y = costmap_->getOriginY();
  const double world_max_x = origin_x + size_x_ * resolution_;
  const double world_max_y = origin_y + size_y_ * resolution_;

  auto log_bounds_once = [&]() {
    RCLCPP_WARN_ONCE(
      node_logger_,
      "Costmap bounds: origin(%.3f, %.3f) size(%u x %u) res=%.3f -> world[%.3f..%.3f, %.3f..%.3f]; start(%.3f, %.3f) goal(%.3f, %.3f) tol=%.3f",
      origin_x, origin_y, size_x_, size_y_, resolution_,
      origin_x, world_max_x, origin_y, world_max_y,
      start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y,
      tolerance_);
  };

  bool start_in_map = worldToMap(start.pose.position.x, start.pose.position.y, sx, sy);
  bool goal_in_map  = worldToMap(goal.pose.position.x,  goal.pose.position.y,  gx, gy);

  if (start_in_map && goal_in_map) {
    // If either start or goal lands on an occupied cell, try to pin to nearest free
    auto pin_if_blocked = [&](unsigned int& mx, unsigned int& my) {
      int idx = index(mx, my);
      if (isTraversable(idx)) return true;
      const double step = std::max(0.5 * resolution_, 0.01);
      for (double r = step; r <= tolerance_; r += step) {
        for (double dx = -r; dx <= r; dx += step) {
          for (double dy = -r; dy <= r; dy += step) {
            unsigned int tx, ty;
            if (worldToMap(costmap_->getOriginX() + (mx + 0.5) * resolution_ + dx,
                           costmap_->getOriginY() + (my + 0.5) * resolution_ + dy, tx, ty)) {
              int tidx = index(tx, ty);
              if (isTraversable(tidx)) { mx = tx; my = ty; return true; }
            }
          }
        }
      }
      return false;
    };

    bool ok_start = pin_if_blocked(sx, sy);
    bool ok_goal  = pin_if_blocked(gx, gy);
    if (!ok_start || !ok_goal) {
      RCLCPP_ERROR(node_logger_, "Failed to pin %s within tolerance of %.2f m",
                   (!ok_start ? "start" : "goal"), tolerance_);
      return false;
    }
    return true;
  }

  // Helper to find a nearby valid cell around a world coordinate
  auto find_fallback = [&](double wx, double wy, unsigned int& mx, unsigned int& my) -> bool {
    const double step = std::max(0.5 * resolution_, 0.01);
    for (double dx = -tolerance_; dx <= tolerance_; dx += step) {
      for (double dy = -tolerance_; dy <= tolerance_; dy += step) {
        unsigned int tx, ty;
        if (worldToMap(wx + dx, wy + dy, tx, ty)) {
          int tidx = index(tx, ty);
          if (isTraversable(tidx)) { mx = tx; my = ty; return true; }
        }
      }
    }
    return false;
  };

  // Adjust start if needed
  if (!start_in_map) {
    log_bounds_once();
    if (!find_fallback(start.pose.position.x, start.pose.position.y, sx, sy)) {
      RCLCPP_ERROR(node_logger_, "Start is out of costmap bounds and no fallback found within tolerance");
      return false;
    }
    RCLCPP_WARN(node_logger_, "Adjusted start to nearby valid cell inside costmap");
  }

  // Adjust goal if needed
  if (!goal_in_map) {
    log_bounds_once();
    if (!find_fallback(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
      RCLCPP_ERROR(node_logger_, "Goal is out of costmap bounds and no fallback found within tolerance");
      return false;
    }
    RCLCPP_WARN(node_logger_, "Adjusted goal to nearby valid cell inside costmap");
  }

  // Final pinning for in-bounds case where cells might be occupied
  auto pin_if_blocked2 = [&](unsigned int& mx, unsigned int& my, const char* label) {
    int idx = index(mx, my);
    if (isTraversable(idx)) return true;
    const double step = std::max(0.5 * resolution_, 0.01);
    for (double r = step; r <= tolerance_; r += step) {
      for (double dx = -r; dx <= r; dx += step) {
        for (double dy = -r; dy <= r; dy += step) {
          unsigned int tx, ty;
          if (worldToMap(costmap_->getOriginX() + (mx + 0.5) * resolution_ + dx,
                         costmap_->getOriginY() + (my + 0.5) * resolution_ + dy, tx, ty)) {
            int tidx = index(tx, ty);
            if (isTraversable(tidx)) { mx = tx; my = ty; return true; }
          }
        }
      }
    }
    RCLCPP_ERROR(node_logger_, "Could not pin %s to a free cell within tolerance", label);
    return false;
  };
  if (!pin_if_blocked2(sx, sy, "start")) return false;
  if (!pin_if_blocked2(gx, gy, "goal")) return false;
  return true;
}

nav_msgs::msg::Path DStarLitePlanner::handleStartEqualsGoal(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  std::chrono::high_resolution_clock::time_point planning_start)
{
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = frame_id_;
  path.poses.push_back(start);
  if (!use_final_approach_orientation_) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }
  last_plan_type_ = PlanType::REPLAN;
  cacheResults(start, goal, path);
  logPlanningMetrics(planning_start, path);
  return path;
}

void DStarLitePlanner::updateCostmapSizeIfChanged()
{
  if (size_x_ != costmap_->getSizeInCellsX() || size_y_ != costmap_->getSizeInCellsY()) {
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();
    // Also refresh resolution and cached frame id in case map changed
    resolution_ = costmap_->getResolution();
    frame_id_ = costmap_ros_->getGlobalFrameID();
    allocateStructures();
    dstar_initialized_ = false;
  }
}

void DStarLitePlanner::performDStarPlanning(const geometry_msgs::msg::PoseStamped& start)
{
  bool first_call = !dstar_initialized_;
  bool new_goal = goal_idx_ != last_goal_idx_;
  bool map_changed = size_x_ != costmap_->getSizeInCellsX() ||
                     size_y_ != costmap_->getSizeInCellsY();

  // FIXED: Only reinitialize for new goals or map changes, NOT empty queues or rhs values
  bool need_initialization = first_call || new_goal || map_changed;

  if (need_initialization) {
    RCLCPP_INFO(node_logger_,
      "D*LITE REINIT: first=%s new_goal=%s map_changed=%s",
      first_call ? "YES" : "NO",
      new_goal ? "YES" : "NO",
      map_changed ? "YES" : "NO");
    initialiseDStar();
    dstar_initialized_ = true;
    last_goal_idx_ = goal_idx_;
    last_start_idx_ = start_idx_;
    prev_start_idx_ = start_idx_;
    last_plan_type_ = PlanType::INITIAL;
  } else if (start_idx_ != last_start_idx_) {
    updateRobotPosition(start);
    last_start_idx_ = start_idx_;
    last_plan_type_ = PlanType::INCREMENTAL;
  } else {
    last_plan_type_ = PlanType::REPLAN;
  }

  computeShortestPath();
}

void DStarLitePlanner::cacheResults(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  const nav_msgs::msg::Path& path)
{
  last_start_ = start;
  last_goal_ = goal;
  last_path_ = path;
  costmap_changed_ = false;
}

void DStarLitePlanner::logPlanningMetrics(
  std::chrono::high_resolution_clock::time_point planning_start,
  const nav_msgs::msg::Path& path)
{
  auto planning_end = std::chrono::high_resolution_clock::now();
  auto planning_duration =
    std::chrono::duration_cast<std::chrono::microseconds>(planning_end - planning_start);
  double planning_time_ms = planning_duration.count() / 1000.0;

  double path_length = calculatePathLength(path);

  double dist_to_goal = std::sqrt(
    std::pow(last_goal_.pose.position.x - last_start_.pose.position.x, 2) +
    std::pow(last_goal_.pose.position.y - last_start_.pose.position.y, 2));

  const char* plan_type =
    (last_plan_type_ == PlanType::INITIAL) ? "INITIAL" :
    (last_plan_type_ == PlanType::INCREMENTAL) ? "INCREMENTAL" : "REPLAN";

  RCLCPP_INFO(node_logger_,
    "METRICS [%s]: dist_to_goal=%.3fm | path_length=%.3fm | planning_time=%.2fms | waypoints=%zu | km=%.2f | iterations=%d",
    plan_type, dist_to_goal, path_length, planning_time_ms, path.poses.size(), km_, vertices_processed_);
}

void DStarLitePlanner::initialiseDStar()
  {
    km_ = 0.0;
    std::fill(g_.begin(), g_.end(), INF_);
    std::fill(rhs_.begin(), rhs_.end(), INF_);
    std::fill(in_open_.begin(), in_open_.end(), false);
    std::fill(came_from_.begin(), came_from_.end(), -1);
    
    while (!open_.empty()) open_.pop();
    key_hash_.clear();

    // D* Lite starts from goal - CRITICAL PROOF of backwards search
    rhs_[goal_idx_] = 0.0;
    pushOpen(goal_idx_, calculateKey(goal_idx_));
    
    RCLCPP_INFO(node_logger_, "D*LITE VALIDATION: Initialized with goal_idx=%d, rhs[goal]=%.1f, g[goal]=%.1f - PROVES backwards search", 
                goal_idx_, rhs_[goal_idx_], g_[goal_idx_]);
  }

void DStarLitePlanner::updateVertex(int idx)
  {
    float old_rhs = rhs_[idx];
    
    if (idx != goal_idx_) {
      float min_rhs = INF_;
      int best_pred = -1;
      
      // Use predecessors for backwards D* Lite search from goal
      std::vector<int> preds = getPredecessors(idx);
      for (int s : preds) {
        float val = traversalCost(s, idx) + g_[s];
        if (val < min_rhs) {
          min_rhs = val;
          best_pred = s;
        }
      }
      rhs_[idx] = min_rhs;
      came_from_[idx] = best_pred;
    }

    // D* Lite validation: log inconsistent vertices (g != rhs)
    if (idx == start_idx_ && old_rhs != rhs_[idx]) {
      RCLCPP_DEBUG(node_logger_, "D*LITE UPDATE START: g[%d]=%.2f, rhs[%d]=%.2f->%.2f - PROVES two-value system", 
                   idx, g_[idx], idx, old_rhs, rhs_[idx]);
    }

    if (in_open_[idx]) in_open_[idx] = false;

    if (g_[idx] != rhs_[idx]) {
      pushOpen(idx, calculateKey(idx));
    }
  }

void DStarLitePlanner::computeShortestPath()
  {
    vertices_processed_ = 0;
    auto t_start = std::chrono::steady_clock::now();

    // FIXED: Remove artificial iteration limits - let D* Lite run to natural convergence
    // Only keep safety timeout to prevent infinite loops in case of bugs
    const int safety_max_iterations = static_cast<int>(size_x_ * size_y_ * 2); // 2x map size as safety
    const double safety_timeout_ms = 5000.0; // 5 second safety timeout

    // Standard D* Lite loop condition: run until start is locally consistent and expanded
    auto loop_cond = [&]() -> bool {
      if (open_.empty()) return false;
      Key top_key = open_.top().key;
      Key start_key = calculateKey(start_idx_);
      return (top_key < start_key) || (rhs_[start_idx_] != g_[start_idx_]);
    };

    while (loop_cond()) {
      ++vertices_processed_;
      
      // Safety checks only - not artificial limits
      if (vertices_processed_ > safety_max_iterations) {
        RCLCPP_ERROR(node_logger_, "D* Lite SAFETY: Exceeded %d iterations - possible infinite loop bug", 
                     safety_max_iterations);
        break;
      }
      
      // Safety timeout check
      if (vertices_processed_ % 1000 == 0) {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_now - t_start).count();
        if (elapsed_ms > safety_timeout_ms) {
          RCLCPP_ERROR(node_logger_, "D* Lite SAFETY: Exceeded %.0f ms timeout - possible infinite loop bug", 
                       safety_timeout_ms);
          break;
        }
      }
      
      PQItem top = open_.top();
      open_.pop();
      if (!in_open_[top.idx]) continue;
      in_open_[top.idx] = false;

      // Quick consistency check before expensive key recomputation
      if (g_[top.idx] == rhs_[top.idx]) continue;

      // Recompute key only if necessary
      Key current_key = calculateKey(top.idx);
      if (top.key < current_key) {
        pushOpen(top.idx, current_key);
        continue;
      }

      // Process vertex with D* Lite's overconsistent/underconsistent logic
      if (g_[top.idx] > rhs_[top.idx]) {
        // Overconsistent vertex: make consistent
        g_[top.idx] = rhs_[top.idx];
        
        if (top.idx == start_idx_) {
          RCLCPP_DEBUG(node_logger_, "D*LITE CONSISTENCY: Start vertex made consistent (g=rhs=%.2f) - PROVES two-value convergence", g_[top.idx]);
        }
        
        // Update predecessors of u
        getSuccessorsOptimized(top.idx, successor_cache_); // grid is symmetric; successors=predecessors
        for (int pred : successor_cache_) {
          updateVertex(pred);
        }
      } else {
        // Underconsistent vertex: set g=INF and recalculate
        g_[top.idx] = INF_;
        updateVertex(top.idx);
        
        getSuccessorsOptimized(top.idx, successor_cache_); // predecessors
        for (int pred : successor_cache_) {
          updateVertex(pred);
        }
      }
    }

    if (g_[start_idx_] == INF_) {
        RCLCPP_ERROR(node_logger_, "SEARCH FAILED: Start vertex unreachable after %d iterations", vertices_processed_);
        
        // Debug: check if start and goal are traversable
        bool start_traversable = isTraversable(start_idx_);
        bool goal_traversable = isTraversable(goal_idx_);
        uint8_t start_cost = costmap_->getCharMap()[start_idx_];
        uint8_t goal_cost = costmap_->getCharMap()[goal_idx_];
        
        RCLCPP_ERROR(node_logger_, "SEARCH DEBUG: start_idx=%d (traversable=%s, cost=%d), goal_idx=%d (traversable=%s, cost=%d)", 
                     start_idx_, start_traversable ? "YES" : "NO", start_cost,
                     goal_idx_, goal_traversable ? "YES" : "NO", goal_cost);
    }
  }

Key DStarLitePlanner::calculateKey(int idx) const
  {
    float v = std::min(g_[idx], rhs_[idx]);
    float h = heuristicCached(start_idx_, idx);
    Key key = {v + h + km_, v};
    
    // Log key calculation for goal vertex to validate D* Lite formula
    if (idx == goal_idx_ && vertices_processed_ % 100 == 0) {
      RCLCPP_DEBUG(node_logger_, "D*LITE KEY for GOAL: k1=%.2f (v=%.2f + h=%.2f + km=%.2f), k2=%.2f - PROVES backwards search from goal", 
                   key.k1, v, h, km_, key.k2);
    }
    
    return key;
  }

void DStarLitePlanner::pushOpen(int idx, const Key & key)
  {
    // Only add if not already in queue with same or better key
    if (!in_open_[idx] || key < key_hash_[idx]) {
      open_.push({key, idx});
      in_open_[idx] = true;
      key_hash_[idx] = key;
    }
  }

std::vector<int> DStarLitePlanner::extractPath() const {
    std::vector<int> grid_path;
    grid_path.reserve(std::max(size_x_, size_y_)); 
    
    // Debug: Check if start is reachable
    if (g_[start_idx_] == INF_) {
        RCLCPP_ERROR(node_logger_, "PATH EXTRACTION FAILED: Start vertex has infinite cost (g=%f, rhs=%f)", 
                     g_[start_idx_], rhs_[start_idx_]);
        return grid_path; // Empty path
    }
    
    RCLCPP_DEBUG(node_logger_, "PATH EXTRACTION: start=%d, goal=%d, g[start]=%.2f, came_from[start]=%d", 
                 start_idx_, goal_idx_, g_[start_idx_], came_from_[start_idx_]);
    
    int current = start_idx_;
    size_t guard = 0;
    const size_t max_path_length = size_x_ * size_y_;

    // Build path from start to goal following came_from pointers, with greedy fallback
    while (current != goal_idx_ && guard++ < max_path_length) {
      grid_path.push_back(current);

      int next = came_from_[current];
      if (next < 0 || next >= static_cast<int>(size_x_ * size_y_)) {
        // Greedy fallback: choose neighbor minimizing g[n] + traversalCost(current,n)
        getSuccessorsOptimized(current, successor_cache_);
        float best_val = INF_;
        int best_n = -1;
        for (int n : successor_cache_) {
          float val = g_[n] + traversalCost(current, n);
          if (val < best_val) { best_val = val; best_n = n; }
        }
        if (best_n < 0) {
          RCLCPP_ERROR(node_logger_, "PATH EXTRACTION: No valid neighbor from %d at step %zu", current, guard);
          break;
        }
        next = best_n;
      }

      // Check for loops
      if (next == current) {
        RCLCPP_ERROR(node_logger_, "PATH EXTRACTION: Loop detected at vertex %d", current);
        break;
      }

      current = next;
    }
    
    // Add goal if reached
    if (current == goal_idx_) {
      grid_path.push_back(goal_idx_);
      RCLCPP_DEBUG(node_logger_, "PATH EXTRACTION SUCCESS: Found path with %zu waypoints", grid_path.size());
    } else {
      RCLCPP_ERROR(node_logger_, "PATH EXTRACTION FAILED: Did not reach goal (current=%d, goal=%d, steps=%zu)", 
                   current, goal_idx_, guard);
    }
    
    return grid_path;
  }

nav_msgs::msg::Path DStarLitePlanner::convertGridPathToWorld(const std::vector<int>& grid_path, 
                                           const geometry_msgs::msg::PoseStamped& start,
                                           const geometry_msgs::msg::PoseStamped& goal) const {
    nav_msgs::msg::Path path;
    path.header.stamp = clock_->now();
    path.header.frame_id = frame_id_;
    path.poses.reserve(grid_path.size());

    for (size_t i = 0; i < grid_path.size(); ++i) {
      int cell = grid_path[i];
      geometry_msgs::msg::PoseStamped wp;
      wp.header = path.header;
      
      int sx = static_cast<int>(size_x_);
      unsigned int mx = static_cast<unsigned int>(cell % sx);
      unsigned int my = static_cast<unsigned int>(cell / sx);
      mapToWorld(mx, my, wp.pose.position.x, wp.pose.position.y);

      if (i == 0) {
        wp.pose.orientation = start.pose.orientation;
      }
      path.poses.push_back(wp);
    }

    // Handle final orientation
    if (use_final_approach_orientation_ && path.poses.size() >= 2) {
      auto & last = path.poses.back().pose.position;
      auto & before = path.poses.end()[-2].pose.position;
      double dx = last.x - before.x;
      double dy = last.y - before.y;
      double yaw = std::atan2(dy, dx);
      path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    } else {
      path.poses.back().pose.orientation = goal.pose.orientation;
    }

    return path;
  }

double DStarLitePlanner::calculatePathLength(const nav_msgs::msg::Path& path) const {
    if (path.poses.size() < 2) return 0.0;
    
    double path_length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
      double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
      path_length += std::sqrt(dx*dx + dy*dy);
    }
    return path_length;
  }

void DStarLitePlanner::getSuccessorsOptimized(int idx, std::vector<int>& nbrs) const {
    nbrs.clear();
    
    // NavFn-style optimization: batch boundary checks
    int sx = static_cast<int>(size_x_);
    int sy = static_cast<int>(size_y_);
    int x = idx % sx;
    int y = idx / sx;
    
    // Pre-compute boundary conditions
    bool can_go_left = x > 0;
    bool can_go_right = x < sx - 1;
    bool can_go_up = y > 0;
    bool can_go_down = y < sy - 1;
    
    // 4-connected neighbors with minimal computation
    if (can_go_right) {
        int nidx = idx + 1;
        if (isTraversable(nidx)) nbrs.push_back(nidx);
    }
    if (can_go_down) {
        int nidx = idx + sx;
        if (isTraversable(nidx)) nbrs.push_back(nidx);
    }
    if (can_go_left) {
        int nidx = idx - 1;
        if (isTraversable(nidx)) nbrs.push_back(nidx);
    }
    if (can_go_up) {
        int nidx = idx - sx;
        if (isTraversable(nidx)) nbrs.push_back(nidx);
    }
    
    // 8-connected diagonals (only if 8-connectivity enabled) with no corner-cutting
    if (connectivity_ == 8) {
        if (can_go_right && can_go_down) {
            int nidx = idx + sx + 1;
            // require both cardinals free: right and down
            if (isTraversable(idx + 1) && isTraversable(idx + sx) && isTraversable(nidx)) nbrs.push_back(nidx);
        }
        if (can_go_left && can_go_down) {
            int nidx = idx + sx - 1;
            // require both cardinals free: left and down
            if (isTraversable(idx - 1) && isTraversable(idx + sx) && isTraversable(nidx)) nbrs.push_back(nidx);
        }
        if (can_go_right && can_go_up) {
            int nidx = idx - sx + 1;
            // require both cardinals free: right and up
            if (isTraversable(idx + 1) && isTraversable(idx - sx) && isTraversable(nidx)) nbrs.push_back(nidx);
        }
        if (can_go_left && can_go_up) {
            int nidx = idx - sx - 1;
            // require both cardinals free: left and up
            if (isTraversable(idx - 1) && isTraversable(idx - sx) && isTraversable(nidx)) nbrs.push_back(nidx);
        }
    }
}

}  // namespace nav2_dstar_lite_planner

PLUGINLIB_EXPORT_CLASS(nav2_dstar_lite_planner::DStarLitePlanner, nav2_core::GlobalPlanner)
