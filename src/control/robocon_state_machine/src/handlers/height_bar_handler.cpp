#include "robocon_state_machine/handlers/height_bar_handler.hpp"

namespace robocon_state_machine {

void HeightBarHandler::init(rclcpp::Node * node) {
  this->node_ = node;
  sub_state_ = 0;
  RCLCPP_INFO(node_->get_logger(), "HeightBarHandler 已初始化");
}

StateResult HeightBarHandler::update(const SensorData & data) {
  StateResult result;
  result.command.command_type = robocon_interfaces::msg::RobotCommand::STOP;

  // 障碍物3: 限高杆 (300mm高度)
  // 子状态:
  //   0: 接近横杆, 确认检测
  //   1: 下蹲指令
  //   2: 匍匐通过横杆下方
  //   3: 站起, 确认已通过

  switch (sub_state_) {
    case 0: {
      double dist = getObstacleDistance(data,
        robocon_interfaces::msg::ObstacleInfo::HEIGHT_BAR);
      if (dist < 0.5 && dist > 0.0) {
        RCLCPP_INFO(node_->get_logger(), "限高杆位于 %.2fm, 准备下蹲", dist);
        sub_state_ = 1;
      } else {
        result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
        result.command.value = 0.1;
      }
      break;
    }

    case 1:
      // 发送下蹲指令
      result.command.command_type = robocon_interfaces::msg::RobotCommand::CROUCH;
      result.command.value = 0.2;  // 下蹲至20cm高度

      // 等待机器人确认已下蹲状态
      if (data.robot_status && data.robot_status->is_crouching) {
        RCLCPP_INFO(node_->get_logger(), "机器人已下蹲, 正在匍匐通过横杆");
        sub_state_ = 2;
      }
      break;

    case 2:
      // 在横杆下方匍匐前进
      result.command.command_type = robocon_interfaces::msg::RobotCommand::SPECIAL_GAIT;
      result.command.gait_type = robocon_interfaces::msg::RobotCommand::GAIT_CRAWL;
      result.command.value = 0.08;  // 极慢速匍匐

      // 检查是否已通过横杆 (匍匐约2m)
      if ((node_->now() - state_start_time_).seconds() > 8.0) {
        sub_state_ = 3;
      }
      break;

    case 3:
      // 站起
      result.command.command_type = robocon_interfaces::msg::RobotCommand::STAND;
      result.completed = true;
      RCLCPP_INFO(node_->get_logger(), "限高杆已通过");
      break;
  }

  return result;
}

}  // namespace robocon_state_machine
