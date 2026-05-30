#ifndef ROBOCON_STATE_MACHINE__MISSION_FSM_HPP_
#define ROBOCON_STATE_MACHINE__MISSION_FSM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "robocon_interfaces/msg/obstacle_info.hpp"
#include "robocon_interfaces/msg/robot_command.hpp"
#include "robocon_interfaces/msg/robot_status.hpp"
#include "robocon_interfaces/msg/tag_detection_array.hpp"

#include "robocon_state_machine/obstacle_handler.hpp"

namespace robocon_state_machine
{

class MissionFSM : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  enum MissionState {
    INIT = 0,             // 初始化
    WAITING_FOR_START,    // 等待启动
    NAV_TO_OBSTACLE,      // 导航至障碍物
    APPROACH_OBSTACLE,    // 接近障碍物
    CROSS_OBSTACLE,       // 通过障碍物
    NAV_TO_END,           // 导航至终点区
    MISSION_COMPLETE,     // 任务完成
    RECOVERY,             // 恢复模式
  };

  explicit MissionFSM(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MissionFSM() = default;

private:
  void tick();  // 主循环，10 Hz
  void transit(MissionState new_state);
  const char * stateName(MissionState state) const;

  // 导航动作客户端
  void sendNavGoal(const geometry_msgs::msg::PoseStamped & goal);
  void cancelNavGoal();
  void navResultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result);

  // 回调函数
  void obstacleCallback(const robocon_interfaces::msg::ObstacleInfo::SharedPtr msg);
  void tagCallback(const robocon_interfaces::msg::TagDetectionArray::SharedPtr msg);
  void statusCallback(const robocon_interfaces::msg::RobotStatus::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // 检查点持久化
  void saveCheckpoint();
  void loadCheckpoint();

  // 订阅
  rclcpp::Subscription<robocon_interfaces::msg::ObstacleInfo>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<robocon_interfaces::msg::TagDetectionArray>::SharedPtr tag_sub_;
  rclcpp::Subscription<robocon_interfaces::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // 发布者
  rclcpp::Publisher<robocon_interfaces::msg::RobotCommand>::SharedPtr command_pub_;

  // 导航动作客户端
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // 状态
  MissionState state_{INIT};
  int current_obstacle_{0};
  rclcpp::Time state_start_time_;
  rclcpp::Time mission_start_time_;
  double mission_time_limit_{210.0};

  // 传感器数据缓存（最新）
  SensorData sensor_data_;

  // 障碍物处理器
  std::vector<std::unique_ptr<ObstacleHandler>> handlers_;

  // 导航路径点（预测量地图坐标系坐标）
  std::vector<geometry_msgs::msg::PoseStamped> obstacle_waypoints_;
  geometry_msgs::msg::PoseStamped end_zone_pose_;

  // 障碍物接近距离（米）
  double nav_approach_distance_{2.0};
  double fine_approach_distance_{0.3};

  // 定时器
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace robocon_state_machine

#endif  // ROBOCON_STATE_MACHINE__MISSION_FSM_HPP_
