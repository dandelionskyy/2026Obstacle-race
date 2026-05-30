#include "robocon_state_machine/handlers/gravel_pit_handler.hpp"

namespace robocon_state_machine {

void GravelPitHandler::init(rclcpp::Node * node) {
  this->node_ = node;
  sub_state_ = 0;
  RCLCPP_INFO(node_->get_logger(), "GravelPitHandler 已初始化");
}

StateResult GravelPitHandler::update(const SensorData & data) {
  StateResult result;
  result.command.command_type = robocon_interfaces::msg::RobotCommand::STOP;

  // 障碍物2: 砾石与木屑坑
  // 子状态:
  //   0: 接近砾石坑
  //   1: 切换为砾石步态, 穿越砾石区域
  //   2: 切换为正常步态, 穿越木屑区域
  //   3: 离开坑区

  switch (sub_state_) {
    case 0: {
      double dist = getObstacleDistance(data,
        robocon_interfaces::msg::ObstacleInfo::GRAVEL_PIT);
      if (dist < 0.5) {
        sub_state_ = 1;
        // 不break，直接进入case 1继续执行
      } else {
        result.command.command_type = robocon_interfaces::msg::RobotCommand::WALK;
        result.command.value = 0.15;
        break;
      }
    }

    case 1:
      // 砾石区域: 特殊步态
      result.command.command_type = robocon_interfaces::msg::RobotCommand::SPECIAL_GAIT;
      result.command.gait_type = robocon_interfaces::msg::RobotCommand::GAIT_GRAVEL;
      result.command.value = 0.1;

      // 检测木屑区域 (TODO: 强度变化检测)
      if ((node_->now() - state_start_time_).seconds() > 5.0) {
        sub_state_ = 2;
      }
      break;

    case 2:
      // 木屑区域: 正常步态
      result.command.command_type = robocon_interfaces::msg::RobotCommand::SPECIAL_GAIT;
      result.command.gait_type = robocon_interfaces::msg::RobotCommand::GAIT_NORMAL;
      result.command.value = 0.15;

      // 检查是否已通过坑区
      if ((node_->now() - state_start_time_).seconds() > 5.0) {
        sub_state_ = 3;
      }
      break;

    case 3:
      result.completed = true;
      RCLCPP_INFO(node_->get_logger(), "砾石坑已完成");
      break;
  }

  return result;
}

}  // namespace robocon_state_machine
