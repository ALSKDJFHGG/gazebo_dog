#ifndef CUSTOM_LOCAL_PLANNER_HPP_
#define CUSTOM_LOCAL_PLANNER_HPP_
 
#include <nav2_core/controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
 
namespace custom_nav_planners
{
class CustomLocalPlanner : public nav2_core::Controller
{
public:
  CustomLocalPlanner();
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override {}
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
 
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  nav_msgs::msg::Path global_path_;
};
}  // namespace custom_nav_planners
 
#endif  // CUSTOM_LOCAL_PLANNER_HPP_