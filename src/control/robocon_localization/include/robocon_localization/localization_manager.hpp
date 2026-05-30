#ifndef ROBOCON_LOCALIZATION__LOCALIZATION_MANAGER_HPP_
#define ROBOCON_LOCALIZATION__LOCALIZATION_MANAGER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

namespace robocon_localization
{

class LocalizationManager : public rclcpp::Node
{
public:
  explicit LocalizationManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalizationManager();

private:
  // 回调函数
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void tagCorrectionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // TF 广播循环
  void tfBroadcastLoop();

  // EMA 滤波
  void applyEmaCorrection(const Eigen::Matrix4d & correction);

  // 订阅
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr correction_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  // TF 广播器
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<std::thread> tf_thread_;

  // 状态
  bool initialized_{false};
  Eigen::Matrix4d map_to_odom_;  // 当前的 map->odom 变换
  std::mutex mutex_;

  // EMA 参数
  double ema_alpha_{0.3};
  double max_translation_jump_{0.5};
  double max_yaw_jump_{15.0};  // 度

  // 坐标系名称
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  // TF 发布频率
  double tf_rate_{100.0};
};

}  // namespace robocon_localization

#endif  // ROBOCON_LOCALIZATION__LOCALIZATION_MANAGER_HPP_
