// Copyright 2025 Lee Sheng Quan & contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <queue>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_dstar_lite_planner {

  struct Key {
    float k1, k2;
    bool operator<(const Key & o) const {
      return (k1 < o.k1) || (k1 == o.k1 && k2 < o.k2);
    }
  };

  struct PQItem {
    Key key;
    int idx;
  };

  struct PQCmp {
    bool operator()(const PQItem & a, const PQItem & b) const {
      return b.key < a.key;
    }
  };

class DStarLitePlanner : public nav2_core::GlobalPlanner {
public:
  DStarLitePlanner() = default;
  ~DStarLitePlanner() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer>,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                               const geometry_msgs::msg::PoseStamped & goal) override;

  // Public method to mark costmap as changed
  void notifyCostmapChanged();

  // Dynamic update methods for D* Lite
  void updateCostmapCell(unsigned int mx, unsigned int my, uint8_t new_cost);
  void updateCostmapRegion(unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y);
  void updateRobotPosition(const geometry_msgs::msg::PoseStamped& new_start);

private:
  // Core D* Lite methods
  void initialiseDStar();
  void updateVertex(int idx);
  void computeShortestPath();
  Key calculateKey(int idx) const;
  void pushOpen(int idx, const Key & key);

  // Helper methods
  inline size_t index(unsigned int x, unsigned int y) const;
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;
  
  std::vector<int> getSuccessors(int idx) const;
  std::vector<int> getPredecessors(int idx) const;
  
  // Optimized versions for performance (NavFn-inspired)
  void getSuccessorsOptimized(int idx, std::vector<int>& nbrs) const;
  float heuristicCached(int a, int b) const;
  bool isTraversable(int idx) const;
  float traversalCost(int idx_from, int idx_to) const;
  float heuristic(int a, int b) const;
  float getCostMultiplier(int idx) const;
  
  void allocateStructures();

  // Helper methods for createPlan
  bool checkCachedPath(const geometry_msgs::msg::PoseStamped& start,
                       const geometry_msgs::msg::PoseStamped& goal,
                       std::chrono::high_resolution_clock::time_point planning_start,
                       nav_msgs::msg::Path& path);
  bool validateAndConvertCoordinates(const geometry_msgs::msg::PoseStamped& start,
                                   const geometry_msgs::msg::PoseStamped& goal,
                                   unsigned int& sx, unsigned int& sy,
                                   unsigned int& gx, unsigned int& gy);
  nav_msgs::msg::Path handleStartEqualsGoal(const geometry_msgs::msg::PoseStamped& start,
                                          const geometry_msgs::msg::PoseStamped& goal,
                                          std::chrono::high_resolution_clock::time_point planning_start);
  void updateCostmapSizeIfChanged();
  void performDStarPlanning(const geometry_msgs::msg::PoseStamped& start);
  void cacheResults(const geometry_msgs::msg::PoseStamped& start,
                   const geometry_msgs::msg::PoseStamped& goal,
                   const nav_msgs::msg::Path& path);
  void logPlanningMetrics(std::chrono::high_resolution_clock::time_point planning_start,
                         const nav_msgs::msg::Path& path);

  // Optimized path methods
  std::vector<int> extractPath() const;
  nav_msgs::msg::Path convertGridPathToWorld(const std::vector<int>& grid_path, 
                                           const geometry_msgs::msg::PoseStamped& start,
                                           const geometry_msgs::msg::PoseStamped& goal) const;
  double calculatePathLength(const nav_msgs::msg::Path& path) const;

  // Member variables
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

  // Precomputed constants for performance
  float sqrt2_;
  float sqrt2_cost_;
  int vertices_processed_ = 0;
  
  // Performance optimization caches
  mutable std::vector<int> successor_cache_;
  mutable std::unordered_map<int, float> heuristic_cache_;
  
  // Connectivity lookup tables (precomputed for performance)
  static constexpr int dx8_[8] = {1, 0, -1, 0, 1, 1, -1, -1};
  static constexpr int dy8_[8] = {0, 1, 0, -1, 1, -1, 1, -1};

  std::vector<float> g_, rhs_;
  std::vector<bool> in_open_;
  std::vector<int> came_from_;
  std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> open_;
  std::unordered_map<int, Key> key_hash_;
  
  // NavFn-inspired priority management
  std::vector<std::vector<int>> priority_buffers_;
  int current_priority_level_;
  static constexpr int MAX_PRIORITY_LEVELS = 1000;

  bool costmap_changed_ {true};
  geometry_msgs::msg::PoseStamped last_start_;
  geometry_msgs::msg::PoseStamped last_goal_;
  nav_msgs::msg::Path last_path_;

  bool dstar_initialized_ {false};
  int last_goal_idx_ {-1};
  int last_start_idx_ {-1};
  int prev_start_idx_ {-1};

enum class PlanType { INITIAL, INCREMENTAL, REPLAN };
PlanType last_plan_type_{PlanType::INITIAL};
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
