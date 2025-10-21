#ifndef TRIAL_1__SPIN_RECOVERY_HPP_
#define TRIAL_1__SPIN_RECOVERY_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/recovery.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"

namespace trial_1
{

class SpinRecovery : public nav2_core::Recovery
{
public:
  SpinRecovery() = default;
  void onConfigure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name) override;
  void onActivate() override;
  void onDeactivate() override;
  void onCleanup() override;
  void onRun(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("spin_recovery");
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmap_client_;
  double spin_yaw_rate_ = 1.0;  // rad/s
  double spin_duration_ = 3.0;  // seconds
  bool call_clear_costmaps_ = true;
};

}  // namespace trial_1

#endif  // TRIAL_1__SPIN_RECOVERY_HPP_
