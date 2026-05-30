#include "robocon_state_machine/handlers/pole_weave_handler.hpp"

namespace robocon_state_machine {

void PoleWeaveHandler::init(rclcpp::Node * node) {
  this->node_ = node;
  sub_state_ = 0;
  RCLCPP_INFO(node_->get_logger(), "PoleWeaveHandler 已初始化");
}

StateResult PoleWeaveHandler::update(const SensorData & data) {
  StateResult result;
  result.command.command_type = robocon_interfaces::msg::RobotCommand::STOP;

  // 障碍物1: 直角穿杆 (S形路径)
  // 子状态:
  //   0: 走向第一根杆的入口区域
  //   1: 执行S形穿杆模式
  //   2: 到达出口区域

  switch (sub_state_) {
    case 0:
      // 朝向第一根杆导航。检测入口区域标签。
      if (isTagDetected(data, 1, 0.5)) {
        RCLCPP_INFO(node_->get_logger(), "检测到穿杆入口标签");
        sub_state_ = 1;
      } else {
        result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
        result.command.value = 0.2;
      }
      break;

    case 1: {
      // 执行S形穿杆: 在杆之间曲折行进。
      // 目前占位实现: 向前行走。
      // TODO: 实现带朝向变化的航点序列
      result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
      result.command.value = 0.15;

      if (isTagDetected(data, 2, 0.5)) {
        RCLCPP_INFO(node_->get_logger(), "检测到穿杆出口标签");
        sub_state_ = 2;
      }
      break;
    }

    case 2:
      result.completed = true;
      RCLCPP_INFO(node_->get_logger(), "穿杆障碍已完成");
      break;
  }

  return result;
}

}  // namespace robocon_state_machine
