// Copyright 2025 Lee Sheng Quan & contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_dstar_lite_planner
{

class DStarLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DStarLitePlanner();
  ~DStarLitePlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  void notifyCostmapChanged();

  // Dynamic update methods for D* Lite
  void updateCostmapCell(unsigned int mx, unsigned int my, uint8_t new_cost);
  void updateCostmapRegion(unsigned int min_x, unsigned int min_y, unsigned int max_x, unsigned int max_y);
  void updateRobotPosition(const geometry_msgs::msg::PoseStamped& new_start);

protected:
  // Grid indexing and transformation
  inline size_t index(unsigned int x, unsigned int y) const;
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

  // Traversability and cost
  bool isTraversable(int idx) const;
  float traversalCost(int from, int to) const;
  float heuristic(int a, int b) const;

  // D* Lite core
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

  void allocateStructures();
  void initialiseDStar();
  Key calculateKey(int idx) const;
  void pushOpen(int idx, const Key & key);
  void updateVertex(int idx);
  void computeShortestPath();
  std::vector<int> getSuccessors(int idx) const;
  std::vector<int> getPredecessors(int idx) const;

  // Members
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger node_logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::string name_;
  std::string frame_id_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;

  unsigned int size_x_ = 0, size_y_ = 0;
  double resolution_ = 0.05;

  double tolerance_ = 0.5;
  bool allow_unknown_ = true;
  int lethal_cost_ = 253;
  int neutral_cost_ = 50;
  double cost_factor_ = 0.8;
  int connectivity_ = 8;
  bool use_final_approach_orientation_ = false;

  int start_idx_ = -1, goal_idx_ = -1;
  float km_ = 0.0;
  static constexpr float INF_ = std::numeric_limits<float>::infinity();

  std::vector<float> g_;
  std::vector<float> rhs_;
  std::vector<bool> in_open_;
  std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> open_;
  std::unordered_map<int, Key> key_hash_;

  // Caching for efficient plan reuse
  geometry_msgs::msg::PoseStamped last_start_;
  geometry_msgs::msg::PoseStamped last_goal_;
  nav_msgs::msg::Path last_path_;
  bool costmap_changed_ = true;
  
  // D* Lite state tracking
  bool dstar_initialized_ = false;
  int last_goal_idx_ = -1;
  int last_start_idx_ = -1;
  int prev_start_idx_ = -1;
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_