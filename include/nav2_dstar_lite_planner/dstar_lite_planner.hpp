#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_dstar_lite_planner
{

struct Node {
  int x, y;
  double g, rhs;

  bool operator==(const Node &other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const Node &other) const {
    return !(*this == other);
  }
};

struct Key {
  double k1, k2;

  bool operator<(const Key &other) const {
    if (k1 == other.k1) return k2 < other.k2;
    return k1 < other.k1;
  }

  bool operator>(const Key &other) const {
    if (k1 == other.k1) return k2 > other.k2;
    return k1 > other.k1;
  }
};

// ✅ Custom comparator that compares only keys
struct CompareKeys {
  bool operator()(const std::pair<Key, Node> &a, const std::pair<Key, Node> &b) const {
    return a.first > b.first;
  }
};

class DStarLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DStarLitePlanner() = default;
  ~DStarLitePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) override;

protected:
  nav2_util::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::string global_frame_;
  nav2_costmap_2d::Costmap2D *costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double interpolation_resolution_;

  std::priority_queue<std::pair<Key, Node>,
                      std::vector<std::pair<Key, Node>>,
                      CompareKeys> open_list_;

  std::unordered_map<int, std::unordered_map<int, Node>> node_map_;

  Key calculateKey(const Node &node);
  void insertOrUpdate(const Node &node, const Key &key);
  std::vector<Node> getNeighbors(const Node &node);
  double heuristic(const Node &a, const Node &b);
  void updateVertex(const Node &node);
};

}  // namespace nav2_dstar_lite_planner

// Provide a hash specialization for Node to avoid instantiation errors
namespace std {
template <>
struct hash<nav2_dstar_lite_planner::Node> {
  std::size_t operator()(const nav2_dstar_lite_planner::Node& node) const {
    return hash<int>()(node.x) ^ (hash<int>()(node.y) << 1);
  }
};
}

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
