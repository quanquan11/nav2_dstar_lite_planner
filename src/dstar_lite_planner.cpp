#include "nav2_dstar_lite_planner/dstar_lite_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <limits>

namespace nav2_dstar_lite_planner {

void DStarLitePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  if (!costmap_ros) {
  RCLCPP_ERROR(node_->get_logger(), "[DStarLitePlanner] costmap_ros is null!");
  return;
}


  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void DStarLitePlanner::cleanup() {
  RCLCPP_INFO(node_->get_logger(), "Cleaning up D* Lite Planner");
} 

void DStarLitePlanner::activate() {
  RCLCPP_INFO(node_->get_logger(), "Activating D* Lite Planner");
}

void DStarLitePlanner::deactivate() {
  RCLCPP_INFO(node_->get_logger(), "Deactivating D* Lite Planner");
}

nav_msgs::msg::Path DStarLitePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;

  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  RCLCPP_INFO(node_->get_logger(), "D* Lite createPlan() called.");

  // Always return at least two poses to activate Nav2
  geometry_msgs::msg::PoseStamped pose_start, pose_goal;

  // Copy input start and goal
  pose_start.pose.position = start.pose.position;
  pose_start.pose.orientation = start.pose.orientation;

  pose_goal.pose.position = goal.pose.position;
  pose_goal.pose.orientation = goal.pose.orientation;

  path.poses.push_back(pose_start);
  path.poses.push_back(pose_goal);

  return path;
}

Key DStarLitePlanner::calculateKey(const Node &node) {
  double k1 = std::min(node.g, node.rhs);
  double k2 = k1;
  return {k1, k2};
}

void DStarLitePlanner::insertOrUpdate(const Node &node, const Key &key) {
  open_list_.push(std::make_pair(key, node));
}

std::vector<Node> DStarLitePlanner::getNeighbors(const Node &node) {
  std::vector<Node> nbrs;
  const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  for (int i = 0; i < 8; ++i) {
    int nx = node.x + dx[i];
    int ny = node.y + dy[i];
    if (nx >= 0 && ny >= 0 && nx < (int)costmap_->getSizeInCellsX() &&
        ny < (int)costmap_->getSizeInCellsY()) {
      if (costmap_->getCost(nx, ny) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) continue;
      if (node_map_[nx][ny].x == 0 && node_map_[nx][ny].y == 0)
        node_map_[nx][ny] = Node{nx, ny, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
      nbrs.push_back(node_map_[nx][ny]);
    }
  }
  return nbrs;
}

double DStarLitePlanner::heuristic(const Node &a, const Node &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

void DStarLitePlanner::updateVertex(const Node &u) {
  Node &node = node_map_[u.x][u.y];
  double min_rhs = std::numeric_limits<double>::infinity();
  for (const auto &nbr : getNeighbors(node)) {
    Node &s = node_map_[nbr.x][nbr.y];
    double cost = s.g + heuristic(node, s);
    if (cost < min_rhs) min_rhs = cost;
  }
  node.rhs = min_rhs;

  if (node.g != node.rhs) {
    insertOrUpdate(node, calculateKey(node));
  }
}

}  // namespace nav2_dstar_lite_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_dstar_lite_planner::DStarLitePlanner, nav2_core::GlobalPlanner)
