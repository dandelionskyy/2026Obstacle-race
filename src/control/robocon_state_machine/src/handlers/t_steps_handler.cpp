#include "robocon_state_machine/handlers/t_steps_handler.hpp"

namespace robocon_state_machine {

void TStepsHandler::init(rclcpp::Node * node) {
  this->node_ = node;
  sub_state_ = 0;
  RCLCPP_INFO(node_->get_logger(), "TStepsHandler 已初始化");
}

StateResult TStepsHandler::update(const SensorData & data) {
  StateResult result;
  result.command.command_type = robocon_interfaces::msg::RobotCommand::STOP;

  // 障碍物7: T形台阶
  // 子状态:
  //   0: 接近台阶
  //   1: 爬台阶上升
  //   2: 到达平台顶部
  //   3: 下台阶下降
  //   4: 完成

  switch (sub_state_) {
    case 0: {
      double dist = getObstacleDistance(data,
        robocon_interfaces::msg::ObstacleInfo::T_STEPS);
      if (dist < 0.5 && dist > 0.0) {
        RCLCPP_INFO(node_->get_logger(), "已到达T形台阶, 开始攀爬");
        sub_state_ = 1;
      } else {
        result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
        result.command.value = 0.1;
      }
      break;
    }

    case 1:
      // 爬台阶 — 台阶步态
      result.command.command_type = robocon_interfaces::msg::RobotCommand::SPECIAL_GAIT;
      result.command.gait_type = robocon_interfaces::msg::RobotCommand::GAIT_STEP_CLIMB;
      result.command.value = 0.05;  // 极慢速爬台阶

      // 需要每个台阶面与机器人足部接触。
      // 持续时间取决于台阶数量 (待场地勘测确定)
      if ((node_->now() - state_start_time_).seconds() > 12.0) {
        sub_state_ = 2;
      }
      break;

    case 2:
      // 在平台顶部 — 走向下降点
      result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
      result.command.value = 0.1;

      if ((node_->now() - state_start_time_).seconds() > 3.0) {
        sub_state_ = 3;
      }
      break;

    case 3:
      // 下台阶
      result.command.command_type = robocon_interfaces::msg::RobotCommand::SPECIAL_GAIT;
      result.command.gait_type = robocon_interfaces::msg::RobotCommand::GAIT_STEP_CLIMB;
      result.command.value = 0.05;

      if ((node_->now() - state_start_time_).seconds() > 12.0) {
        sub_state_ = 4;
      }
      break;

    case 4:
      result.completed = true;
      RCLCPP_INFO(node_->get_logger(), "T形台阶已完成");
      break;
  }

  return result;
}

}  // namespace robocon_state_machine
