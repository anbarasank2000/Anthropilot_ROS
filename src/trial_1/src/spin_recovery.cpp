#include "trial_1/spin_recovery.hpp"

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace trial_1
{

void SpinRecovery::onConfigure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  // parameters
  nav2_util::declare_parameter_if_not_declared(node, name + ".spin_yaw_rate", rclcpp::ParameterValue(spin_yaw_rate_));
  nav2_util::declare_parameter_if_not_declared(node, name + ".spin_duration", rclcpp::ParameterValue(spin_duration_));
  nav2_util::declare_parameter_if_not_declared(node, name + ".call_clear_costmaps", rclcpp::ParameterValue(call_clear_costmaps_));

  node->get_parameter(name + ".spin_yaw_rate", spin_yaw_rate_);
  node->get_parameter(name + ".spin_duration", spin_duration_);
  node->get_parameter(name + ".call_clear_costmaps", call_clear_costmaps_);

  cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

  if (call_clear_costmaps_) {
    clear_costmap_client_ = node->create_client<std_srvs::srv::Empty>("/clear_costmaps");
  }
}

void SpinRecovery::onActivate()
{
  // nothing to do; publisher already created on configure
}

void SpinRecovery::onDeactivate()
{
  // nothing to do
}

void SpinRecovery::onCleanup()
{
  cmd_vel_pub_.reset();
  clear_costmap_client_.reset();
}

void SpinRecovery::onRun(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_WARN(logger_, "Executing SpinRecovery: spin %.2frad/s for %.2fs", spin_yaw_rate_, spin_duration_);

  // call clear costmaps if enabled and available
  if (call_clear_costmaps_ && clear_costmap_client_) {
    if (clear_costmap_client_->wait_for_service(1s)) {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      clear_costmap_client_->async_send_request(req);
      RCLCPP_INFO(logger_, "Requested clear_costmaps service");
    } else {
      RCLCPP_WARN(logger_, "clear_costmaps service not available");
    }
  }

  // publish cmd_vel to spin
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = spin_yaw_rate_;

  rclcpp::Rate rate(20);
  auto start_time = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start_time) < std::chrono::duration<double>(spin_duration_)) {
    cmd_vel_pub_->publish(cmd);
    rate.sleep();
  }

  // stop
  cmd.angular.z = 0.0;
  cmd.linear.x = 0.0;
  cmd_vel_pub_->publish(cmd);

  RCLCPP_INFO(logger_, "SpinRecovery finished");
}

}  // namespace trial_1

// plugin export
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(trial_1::SpinRecovery, nav2_core::Recovery)
