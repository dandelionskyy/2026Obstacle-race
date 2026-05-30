#include "robocon_state_machine/handlers/bridge_handler.hpp"

namespace robocon_state_machine {

void BridgeHandler::init(rclcpp::Node * node) {
  this->node_ = node;
  sub_state_ = 0;
  RCLCPP_INFO(node_->get_logger(), "BridgeHandler 已初始化");
}

StateResult BridgeHandler::update(const SensorData & data) {
  StateResult result;
  result.command.command_type = robocon_interfaces::msg::RobotCommand::STOP;

  // 障碍物5/6: 木桥A和B
  // 子状态:
  //   0: 在桥面上向前行走
  //   1: 跨越桥面
  //   2: 到达另一侧平台

  switch (sub_state_) {
    case 0: {
      double dist = getObstacleDistance(data,
        robocon_interfaces::msg::ObstacleInfo::BRIDGE);
      if (dist < 1.0 && dist > 0.0) {
        RCLCPP_INFO(node_->get_logger(), "已到达桥面");
        sub_state_ = 1;
      } else {
        result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
        result.command.value = 0.15;
      }
      break;
    }

    case 1:
      // 过桥 — 慢速、小心行走
      result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
      result.command.value = 0.08;  // 缓慢平稳

      // 假设桥长约2m, 约15秒通过
      if ((node_->now() - state_start_time_).seconds() > 15.0) {
        sub_state_ = 2;
      }
      break;

    case 2:
      result.completed = true;
      RCLCPP_INFO(node_->get_logger(), "木桥已通过");
      break;
  }

  return result;
}

}  // namespace robocon_state_machine
