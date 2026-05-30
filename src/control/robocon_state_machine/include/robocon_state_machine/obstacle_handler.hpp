#ifndef ROBOCON_STATE_MACHINE__OBSTACLE_HANDLER_HPP_
#define ROBOCON_STATE_MACHINE__OBSTACLE_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "robocon_interfaces/msg/obstacle_info.hpp"
#include "robocon_interfaces/msg/robot_command.hpp"
#include "robocon_interfaces/msg/robot_status.hpp"
#include "robocon_interfaces/msg/tag_detection_array.hpp"

namespace robocon_state_machine
{

struct SensorData {
  robocon_interfaces::msg::ObstacleInfo::ConstSharedPtr obstacle_info;
  robocon_interfaces::msg::TagDetectionArray::ConstSharedPtr tag_detections;
  robocon_interfaces::msg::RobotStatus::ConstSharedPtr robot_status;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;  // 来自 /Odometry
};

struct StateResult {
  bool completed = false;    // 障碍物通过完成
  bool failed = false;       // 障碍物失败，需要恢复
  std::string message;       // 调试信息
  robocon_interfaces::msg::RobotCommand command;  // 本周期发送的指令
};

class ObstacleHandler
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ObstacleHandler() = default;
  virtual ~ObstacleHandler() = default;

  virtual void init(rclcpp::Node * node) = 0;
  virtual StateResult update(const SensorData & data) = 0;
  virtual const char * name() const = 0;

  // 通用辅助函数
  bool isTagDetected(const SensorData & data, int tag_id, double min_conf = 0.5) const;
  double getObstacleDistance(const SensorData & data, uint8_t obs_type) const;

protected:
  rclcpp::Node * node_{nullptr};
  int sub_state_{0};
  rclcpp::Time state_start_time_;
};

}  // namespace robocon_state_machine

#endif  // ROBOCON_STATE_MACHINE__OBSTACLE_HANDLER_HPP_
