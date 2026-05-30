#include "robocon_command_bridge/command_bridge.hpp"

namespace robocon_command_bridge
{

CommandBridge::CommandBridge(const rclcpp::NodeOptions & options)
: Node("command_bridge", options)
{
  // 订阅者
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_nav", rclcpp::QoS(10),
    std::bind(&CommandBridge::cmdVelCallback, this, std::placeholders::_1));

  robot_cmd_sub_ = this->create_subscription<robocon_interfaces::msg::RobotCommand>(
    "/robot_command", rclcpp::QoS(10),
    std::bind(&CommandBridge::robotCommandCallback, this, std::placeholders::_1));

  // 发布者 (转发到控制板)
  cmd_vel_out_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10));

  robot_cmd_out_ = this->create_publisher<robocon_interfaces::msg::RobotCommand>(
    "/robot_command_out", rclcpp::QoS(10));

  last_robot_cmd_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "CommandBridge 已初始化");
  RCLCPP_INFO(this->get_logger(), "  模式: NAVIGATION (转发 /cmd_vel)");
}

void CommandBridge::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 仅在NAVIGATION模式下转发
  if (mode_ == Mode::NAVIGATION) {
    cmd_vel_out_->publish(*msg);
  }
  // 在DIRECT模式下, /cmd_vel 被抑制
}

void CommandBridge::robotCommandCallback(
  const robocon_interfaces::msg::RobotCommand::SharedPtr msg)
{
  last_robot_cmd_time_ = this->now();

  // 切换到DIRECT模式
  mode_ = Mode::DIRECT;

  // 转发机器人指令
  robot_cmd_out_->publish(*msg);

  // 检查超时: 如果在 direct_mode_timeout_ 内没有新的 robot_command,
  // 恢复到NAVIGATION模式
  auto elapsed = (this->now() - last_robot_cmd_time_).seconds();
  if (elapsed > direct_mode_timeout_) {
    mode_ = Mode::NAVIGATION;
    RCLCPP_INFO(this->get_logger(), "DIRECT模式超时, 恢复为NAVIGATION模式");
  }
}

}  // namespace robocon_command_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robocon_command_bridge::CommandBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
