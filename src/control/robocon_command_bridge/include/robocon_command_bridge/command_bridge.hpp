#ifndef ROBOCON_COMMAND_BRIDGE__COMMAND_BRIDGE_HPP_
#define ROBOCON_COMMAND_BRIDGE__COMMAND_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "robocon_interfaces/msg/robot_command.hpp"

namespace robocon_command_bridge
{

class CommandBridge : public rclcpp::Node
{
public:
  explicit CommandBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CommandBridge() = default;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void robotCommandCallback(const robocon_interfaces::msg::RobotCommand::SharedPtr msg);

  // 订阅
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<robocon_interfaces::msg::RobotCommand>::SharedPtr robot_cmd_sub_;

  // 转发发布者（至机器人控制板）
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_;
  rclcpp::Publisher<robocon_interfaces::msg::RobotCommand>::SharedPtr robot_cmd_out_;

  // 状态
  enum class Mode { NAVIGATION, DIRECT };
  Mode mode_{Mode::NAVIGATION};
  rclcpp::Time last_robot_cmd_time_;
  double direct_mode_timeout_{2.0};  // 切换到 NAVIGATION 模式前的超时秒数
};

}  // namespace robocon_command_bridge

#endif  // ROBOCON_COMMAND_BRIDGE__COMMAND_BRIDGE_HPP_
