#include "robocon_state_machine/mission_fsm.hpp"

#include <fstream>
#include <filesystem>

namespace robocon_state_machine
{

MissionFSM::MissionFSM(const rclcpp::NodeOptions & options)
: Node("mission_fsm", options)
{
  // 加载参数
  this->declare_parameter("obstacle_count", 8);
  this->declare_parameter("nav_approach_distance", 2.0);
  this->declare_parameter("fine_approach_distance", 0.3);
  this->declare_parameter("mission_time_limit", 210.0);

  nav_approach_distance_ = this->get_parameter("nav_approach_distance").as_double();
  fine_approach_distance_ = this->get_parameter("fine_approach_distance").as_double();
  mission_time_limit_ = this->get_parameter("mission_time_limit").as_double();

  // 订阅者
  obstacle_sub_ = this->create_subscription<robocon_interfaces::msg::ObstacleInfo>(
    "/obstacle_info", rclcpp::QoS(10),
    std::bind(&MissionFSM::obstacleCallback, this, std::placeholders::_1));

  tag_sub_ = this->create_subscription<robocon_interfaces::msg::TagDetectionArray>(
    "/tag_detections", rclcpp::QoS(10),
    std::bind(&MissionFSM::tagCallback, this, std::placeholders::_1));

  status_sub_ = this->create_subscription<robocon_interfaces::msg::RobotStatus>(
    "/robot_status", rclcpp::QoS(10),
    std::bind(&MissionFSM::statusCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", rclcpp::QoS(10),
    std::bind(&MissionFSM::odomCallback, this, std::placeholders::_1));

  // 发布者
  command_pub_ = this->create_publisher<robocon_interfaces::msg::RobotCommand>(
    "/robot_command", rclcpp::QoS(10));

  // 导航动作客户端
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

  // 初始化障碍物处理器 (在任务进行中按需创建)
  // 处理器按任务推进顺序实例化

  // 设置航点 (TODO: 从配置文件加载)
  // 这些是占位位置 — 实际位置来自场地勘测
  for (int i = 0; i < 8; i++) {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.pose.position.x = 1.0 + i * 1.5;
    wp.pose.position.y = 0.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.w = 1.0;
    obstacle_waypoints_.push_back(wp);
  }
  end_zone_pose_.header.frame_id = "map";
  end_zone_pose_.pose.position.x = 0.0;
  end_zone_pose_.pose.orientation.w = 1.0;

  // 加载检查点
  loadCheckpoint();

  // 主循环定时器
  tick_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionFSM::tick, this));

  RCLCPP_INFO(this->get_logger(), "MissionFSM 已初始化。状态: %s", stateName(state_));
}

void MissionFSM::tick()
{
  // 检查任务超时
  auto elapsed = (get_clock()->now() - mission_start_time_).seconds();
  if (state_ != INIT && state_ != MISSION_COMPLETE &&
      elapsed > mission_time_limit_) {
    RCLCPP_WARN(this->get_logger(), "任务超时! 已耗时 %.1fs", elapsed);
    transit(RECOVERY);
  }

  switch (state_) {
    case INIT:
      // 等待定位系统就绪 (TF map->odom 可用)
      // 目前直接切换状态 (TODO: 检查TF)
      RCLCPP_INFO(this->get_logger(), "INIT — 等待启动信号...");
      state_ = WAITING_FOR_START;
      break;

    case WAITING_FOR_START:
      // 等待外部启动信号 (操作员或定时器)
      // 目前2秒后自动启动
      if ((get_clock()->now() - state_start_time_).seconds() > 2.0) {
        RCLCPP_INFO(this->get_logger(), "任务启动!");
        mission_start_time_ = get_clock()->now();
        transit(NAV_TO_OBSTACLE);
      }
      break;

    case NAV_TO_OBSTACLE:
    {
      // 导航至下一个障碍物航点
      if (current_obstacle_ >= static_cast<int>(obstacle_waypoints_.size())) {
        transit(NAV_TO_END);
        break;
      }
      sendNavGoal(obstacle_waypoints_[current_obstacle_]);
      transit(APPROACH_OBSTACLE);
      break;
    }

    case APPROACH_OBSTACLE:
    {
      // 检查是否足够接近以触发精细接近
      if (sensor_data_.obstacle_info &&
          sensor_data_.obstacle_info->distance < nav_approach_distance_ &&
          sensor_data_.obstacle_info->distance > fine_approach_distance_) {
        // TODO: 取消Nav2, 使用直接 /robot_command 开始精细接近
        auto cmd = robocon_interfaces::msg::RobotCommand();
        cmd.command_type = robocon_interfaces::msg::RobotCommand::WALK;
        cmd.value = 0.1;  // 慢速接近速度
        command_pub_->publish(cmd);
      } else if (sensor_data_.obstacle_info &&
                 sensor_data_.obstacle_info->distance <= fine_approach_distance_) {
        // 切换到障碍物穿越状态
        transit(CROSS_OBSTACLE);
      }
      break;
    }

    case CROSS_OBSTACLE:
    {
      // 委托给当前障碍物处理器
      // TODO: 实例化相应的处理器类并调用 update()
      // 目前占位实现: 发送STOP指令5秒后标记完成
      auto cmd = robocon_interfaces::msg::RobotCommand();
      cmd.command_type = robocon_interfaces::msg::RobotCommand::STOP;
      command_pub_->publish(cmd);

      if ((get_clock()->now() - state_start_time_).seconds() > 5.0) {
        RCLCPP_INFO(this->get_logger(), "障碍物 %d 穿越完成", current_obstacle_ + 1);
        current_obstacle_++;
        saveCheckpoint();
        transit(NAV_TO_OBSTACLE);
      }
      break;
    }

    case NAV_TO_END:
    {
      RCLCPP_INFO(this->get_logger(), "正在导航至终点区域");
      sendNavGoal(end_zone_pose_);
      // 等待到达 (简化)
      transit(MISSION_COMPLETE);
      break;
    }

    case MISSION_COMPLETE:
      RCLCPP_INFO(this->get_logger(), "任务完成!");
      break;

    case RECOVERY:
    {
      auto cmd = robocon_interfaces::msg::RobotCommand();
      cmd.command_type = robocon_interfaces::msg::RobotCommand::STOP;
      command_pub_->publish(cmd);

      // 简单恢复: 等待3秒后重试当前障碍物
      if ((get_clock()->now() - state_start_time_).seconds() > 3.0) {
        RCLCPP_INFO(this->get_logger(), "恢复: 重新尝试障碍物 %d", current_obstacle_ + 1);
        transit(NAV_TO_OBSTACLE);
      }
      break;
    }
  }
}

void MissionFSM::transit(MissionState new_state)
{
  RCLCPP_INFO(this->get_logger(), "状态切换: %s -> %s",
              stateName(state_), stateName(new_state));
  state_ = new_state;
  state_start_time_ = get_clock()->now();
}

const char * MissionFSM::stateName(MissionState state) const
{
  switch (state) {
    case INIT: return "INIT";
    case WAITING_FOR_START: return "WAITING_FOR_START";
    case NAV_TO_OBSTACLE: return "NAV_TO_OBSTACLE";
    case APPROACH_OBSTACLE: return "APPROACH_OBSTACLE";
    case CROSS_OBSTACLE: return "CROSS_OBSTACLE";
    case NAV_TO_END: return "NAV_TO_END";
    case MISSION_COMPLETE: return "MISSION_COMPLETE";
    case RECOVERY: return "RECOVERY";
    default: return "UNKNOWN";
  }
}

void MissionFSM::sendNavGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2动作服务器不可用");
    return;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&MissionFSM::navResultCallback, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void MissionFSM::cancelNavGoal()
{
  nav_client_->async_cancel_all_goals();
}

void MissionFSM::navResultCallback(
  const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "导航目标成功");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "导航目标中止");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "导航目标取消");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "导航目标结果未知");
  }
}

void MissionFSM::obstacleCallback(
  const robocon_interfaces::msg::ObstacleInfo::SharedPtr msg)
{
  sensor_data_.obstacle_info = msg;
}

void MissionFSM::tagCallback(
  const robocon_interfaces::msg::TagDetectionArray::SharedPtr msg)
{
  sensor_data_.tag_detections = msg;
}

void MissionFSM::statusCallback(
  const robocon_interfaces::msg::RobotStatus::SharedPtr msg)
{
  sensor_data_.robot_status = msg;
}

void MissionFSM::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_msg->header = msg->header;
  pose_msg->pose = msg->pose.pose;
  sensor_data_.current_pose = pose_msg;
}

void MissionFSM::saveCheckpoint()
{
  try {
    std::string path = "config/checkpoint/mission_state.yaml";
    std::ofstream file(path);
    file << "current_obstacle: " << current_obstacle_ << "\n";
    file << "state: " << static_cast<int>(state_) << "\n";
    file.close();
    RCLCPP_INFO(this->get_logger(), "检查点已保存: obstacle=%d", current_obstacle_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "保存检查点失败: %s", e.what());
  }
}

void MissionFSM::loadCheckpoint()
{
  try {
    std::string path = "config/checkpoint/mission_state.yaml";
    if (!std::filesystem::exists(path)) return;

    // 简单YAML解析 (TODO: 使用 yaml-cpp)
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
      if (line.find("current_obstacle:") != std::string::npos) {
        current_obstacle_ = std::stoi(line.substr(line.find(':') + 1));
      }
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "检查点已加载: obstacle=%d", current_obstacle_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "加载检查点失败: %s", e.what());
  }
}

}  // namespace robocon_state_machine

