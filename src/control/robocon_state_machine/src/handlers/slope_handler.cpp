#include "robocon_state_machine/handlers/slope_handler.hpp"

namespace robocon_state_machine {

void SlopeHandler::init(rclcpp::Node * node) {
  this->node_ = node;
  sub_state_ = 0;
  RCLCPP_INFO(node_->get_logger(), "SlopeHandler 已初始化");
}

StateResult SlopeHandler::update(const SensorData & data) {
  StateResult result;
  result.command.command_type = robocon_interfaces::msg::RobotCommand::STOP;

  // 障碍物4: 大斜坡 (11.3°)
  // 子状态:
  //   0: 接近斜坡
  //   1: 上坡
  //   2: 沿坡顶行走 (≥1m)
  //   3: 下坡
  //   4: 完成

  switch (sub_state_) {
    case 0: {
      double dist = getObstacleDistance(data,
        robocon_interfaces::msg::ObstacleInfo::SLOPE);
      if (dist < 1.0 && dist > 0.0) {
        RCLCPP_INFO(node_->get_logger(), "检测到斜坡, 开始上坡");
        sub_state_ = 1;
      } else {
        result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
        result.command.value = 0.15;
      }
      break;
    }

    case 1:
      // 爬坡
      result.command.command_type = robocon_interfaces::msg::RobotCommand::CLIMB;
      result.command.value = 0.1;

      // 检查IMU俯仰角以检测是否已在斜坡上
      if (data.robot_status &&
          std::abs(data.robot_status->imu_pitch) > 0.17) {  // > 10° 俯仰角
        sub_state_ = 2;
        RCLCPP_INFO(node_->get_logger(), "已到坡顶, 正在沿坡顶行走");
      }
      break;

    case 2:
      // 沿坡顶行走 (需要≥1m)
      result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
      result.command.value = 0.1;

      if ((node_->now() - state_start_time_).seconds() > 10.0) {
        sub_state_ = 3;
      }
      break;

    case 3:
      // 下坡
      result.command.command_type = robocon_interfaces::msg::RobotCommand::CLIMB;
      result.command.value = 0.08;

      if (data.robot_status &&
          std::abs(data.robot_status->imu_pitch) < 0.05) {  // 回到平地
        sub_state_ = 4;
      }
      break;

    case 4:
      result.completed = true;
      RCLCPP_INFO(node_->get_logger(), "斜坡已完成");
      break;
  }

  return result;
}

}  // namespace robocon_state_machine
