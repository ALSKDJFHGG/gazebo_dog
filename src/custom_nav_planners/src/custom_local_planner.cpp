#include "custom_nav_planners/custom_local_planner.hpp"
 
namespace custom_nav_planners
{
CustomLocalPlanner::CustomLocalPlanner() {}
 
void CustomLocalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  RCLCPP_INFO(node_->get_logger(), "Configuring CustomLocalPlanner: %s", name_.c_str());
}
 
void CustomLocalPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating CustomLocalPlanner");
}
 
void CustomLocalPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating CustomLocalPlanner");
}
 
void CustomLocalPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up CustomLocalPlanner");
}
 
void CustomLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_path_ = path;
  RCLCPP_INFO(node_->get_logger(), "Received global path with %zu points", path.poses.size());
}
 
geometry_msgs::msg::TwistStamped CustomLocalPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = node_->now();
 
  // Simple constant velocity towards the next waypoint
  if (!global_path_.poses.empty()) {
    cmd_vel.twist.linear.x = 0.2;  // Constant forward speed (m/s)
    cmd_vel.twist.angular.z = 0.0; // No rotation (simplified)
  }
 
  RCLCPP_INFO(node_->get_logger(), "Computed velocity: linear.x=%.2f", cmd_vel.twist.linear.x);
  return cmd_vel;
}
}  // namespace custom_nav_planners
 
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_nav_planners::CustomLocalPlanner, nav2_core::Controller)