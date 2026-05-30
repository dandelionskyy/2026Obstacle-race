#include "robocon_state_machine/handlers/high_wall_handler.hpp"

namespace robocon_state_machine {

void HighWallHandler::init(rclcpp::Node * node) {
  this->node_ = node;
  sub_state_ = 0;
  RCLCPP_INFO(node_->get_logger(), "HighWallHandler 已初始化");
}

StateResult HighWallHandler::update(const SensorData & data) {
  StateResult result;
  result.command.command_type = robocon_interfaces::msg::RobotCommand::STOP;

  // 障碍物8: 高墙
  // 子状态:
  //   0: 接近墙壁, 垂直对齐
  //   1: 执行跳跃/翻越墙壁
  //   2: 落地并恢复
  //   3: 完成

  switch (sub_state_) {
    case 0: {
      double dist = getObstacleDistance(data,
        robocon_interfaces::msg::ObstacleInfo::HIGH_WALL);

      if (dist < 0.5 && dist > 0.0) {
        // 检查对齐 — 朝向角应接近0 (垂直于墙壁)
        double heading = data.obstacle_info ? data.obstacle_info->heading : 999;
        if (std::abs(heading) < 0.1) {  // 在垂直方向约6°以内
          RCLCPP_INFO(node_->get_logger(), "墙壁位于 %.2fm, 已对齐。正在翻越!", dist);
          sub_state_ = 1;
        } else {
          // 需要对齐 — 原地转向
          result.command.command_type =
            (heading > 0) ? robocon_interfaces::msg::RobotCommand::TURN_LEFT
                          : robocon_interfaces::msg::RobotCommand::TURN_RIGHT;
          result.command.value = 0.2;  // 慢速转向
        }
      } else {
        result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
        result.command.value = 0.15;
      }
      break;
    }

    case 1:
      // 执行翻墙/跳跃
      result.command.command_type = robocon_interfaces::msg::RobotCommand::CLIMB;
      result.command.value = 1.0;  // 翻墙模式

      if (data.robot_status && data.robot_status->climbing_progress > 0.9) {
        RCLCPP_INFO(node_->get_logger(), "翻墙即将完成");
        sub_state_ = 2;
      }
      // 基于超时的回退方案
      if ((node_->now() - state_start_time_).seconds() > 10.0) {
        sub_state_ = 2;
      }
      break;

    case 2:
      // 落地并恢复
      result.command.command_type = robocon_interfaces::msg::RobotCommand::STAND;
      result.command.value = 0.0;

      if ((node_->now() - state_start_time_).seconds() > 2.0) {
        sub_state_ = 3;
      }
      break;

    case 3:
      result.completed = true;
      RCLCPP_INFO(node_->get_logger(), "高墙已完成");
      break;
  }

  return result;
}

}  // namespace robocon_state_machine
