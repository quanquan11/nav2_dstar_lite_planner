// Copyright 2025 Lee Sheng Quan & contributors
// SPDX‑License‑Identifier: Apache‑2.0
//
// D* Lite global‑planner header for ROS 2 Nav2.
// Split out from the self‑contained example so that users
// who prefer the traditional *.hpp + *.cpp* structure can include
// this file in *include/nav2_dstar_lite_planner/* and keep the
// implementation in *src/dstar_lite_planner.cpp*.
//
// See the companion implementation file for full algorithm details.

#pragma once

#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_dstar_lite_planner
{

//------------------------------------------------------------------------------
//  Basic grid‑node container used by D* Lite.
//------------------------------------------------------------------------------
struct Node
{
  int x{0}, y{0};
  float g{std::numeric_limits<float>::infinity()};
  float rhs{std::numeric_limits<float>::infinity()};

  Node() = default;
  Node(int _x, int _y) : x(_x), y(_y) {}
  bool operator==(const Node & other) const {return x == other.x && y == other.y;}
};

struct Key
{
  float k1{0.f}, k2{0.f};
  bool operator<(const Key & o) const noexcept
  {
    return (k1 < o.k1) || (k1 == o.k1 && k2 < o.k2);
  }
};

//------------------------------------------------------------------------------
//  Main planner class (interface identical to NavFnPlanner).
//------------------------------------------------------------------------------
class DStarLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DStarLitePlanner() = default;
  ~DStarLitePlanner() override = default;

  // ‑‑‑ Nav2 lifecycle hooks ‑‑‑ ------------------------------------------------
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

  // Optional: incremental updates when the global costmap publishes changes.
  void onMapUpdate(const nav2_msgs::msg::Costmap & msg);

private:
  // Helpers --------------------------------------------------------------------
  void initialiseSearch();
  void updateVertex(int idx);
  void computeShortestPath();
  Key  calculateKey(int idx) const;
  bool costIsLethal(uint8_t cost) const;
  int  getBestSuccessor(int idx) const;

  // Conversion helpers (thin wrappers around Costmap2D methods).
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

  // Members --------------------------------------------------------------------
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Logger logger_{rclcpp::get_logger("DStarLitePlanner")};
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  // Parameters (mirrored from NavFn for drop‑in replacement)
  double interpolation_resolution_{0.0};  // 0 → no resampling (default NavFn behaviour)
  double tolerance_{0.5};
  bool   allow_unknown_{true};
  bool   use_final_approach_orientation_{false};
  int    lethal_cost_{253};
  int    neutral_cost_{50};
  double cost_factor_{0.8};
  int    connectivity_{8};  // 4 or 8

  // D* Lite state ----------------------------------------------------------------
  float km_{0.f};
  int   start_idx_{-1};
  int   goal_idx_{-1};
  std::unordered_map<int, Node> nodes_;           // flattened‑index → state
  using PQElem = std::pair<Key,int>;              // (priority, index)
  struct PQCmp { bool operator()(const PQElem & a, const PQElem & b) const {return b.first < a.first;} };
  std::priority_queue<PQElem, std::vector<PQElem>, PQCmp> open_;  // min‑heap

  // Cached size params for speed.
  int size_x_{0};
  int size_y_{0};
  double resolution_{0.05};
  double origin_x_{0.0}, origin_y_{0.0};
};

} // namespace nav2_dstar_lite_planner
