#include "robocon_localization/localization_manager.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace robocon_localization
{

LocalizationManager::LocalizationManager(const rclcpp::NodeOptions & options)
: Node("localization_manager", options)
{
  // 参数
  map_frame_ = this->declare_parameter("map_frame", "map");
  odom_frame_ = this->declare_parameter("odom_frame", "odom");
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  ema_alpha_ = this->declare_parameter("ema_alpha", 0.3);
  max_translation_jump_ = this->declare_parameter("max_translation_jump", 0.5);
  max_yaw_jump_ = this->declare_parameter("max_yaw_jump", 15.0);
  tf_rate_ = this->declare_parameter("tf_rate", 100.0);

  // 初始化 map_to_odom 为单位矩阵
  map_to_odom_ = Eigen::Matrix4d::Identity();

  // 订阅者
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", rclcpp::QoS(10),
    std::bind(&LocalizationManager::odomCallback, this, std::placeholders::_1));

  correction_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/tag_pose_correction", rclcpp::QoS(10),
    std::bind(&LocalizationManager::tagCorrectionCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(10),
    std::bind(&LocalizationManager::initialPoseCallback, this, std::placeholders::_1));

  // TF广播器
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // 启动TF广播线程
  tf_thread_ = std::make_unique<std::thread>(
    &LocalizationManager::tfBroadcastLoop, this);

  RCLCPP_INFO(this->get_logger(), "LocalizationManager 已初始化");
  RCLCPP_INFO(this->get_logger(), "  TF树: %s -> %s -> %s",
              map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
}

LocalizationManager::~LocalizationManager()
{
  if (tf_thread_ && tf_thread_->joinable()) {
    tf_thread_->join();
  }
}

void LocalizationManager::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // FAST-LIO2里程计 — 用于前向传播
  // 在简化模型中, map->odom在校正之间保持不变,
  // FAST-LIO2漂移由AprilTag观测进行校正。
  // 更复杂的实现会在此处进行前向传播。
  (void)msg;
}

void LocalizationManager::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // ICP初始对齐结果
  std::lock_guard<std::mutex> lock(mutex_);

  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);

  map_to_odom_.block<3,3>(0,0) = q.toRotationMatrix();
  map_to_odom_(0,3) = msg->pose.pose.position.x;
  map_to_odom_(1,3) = msg->pose.pose.position.y;
  map_to_odom_(2,3) = msg->pose.pose.position.z;

  initialized_ = true;

  RCLCPP_INFO(this->get_logger(),
              "初始位姿已设置: (%.2f, %.2f, %.2f) yaw=%.1f°",
              map_to_odom_(0,3), map_to_odom_(1,3), map_to_odom_(2,3),
              std::atan2(map_to_odom_(1,0), map_to_odom_(0,0)) * 180.0 / M_PI);
}

void LocalizationManager::tagCorrectionCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // 基于AprilTag的校正
  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);

  Eigen::Matrix4d T_map_base = Eigen::Matrix4d::Identity();
  T_map_base.block<3,3>(0,0) = q.toRotationMatrix();
  T_map_base(0,3) = msg->pose.pose.position.x;
  T_map_base(1,3) = msg->pose.pose.position.y;
  T_map_base(2,3) = msg->pose.pose.position.z;

  // 计算 map->odom = T_map_base * inv(T_odom_base)
  // 在简化模型中 T_odom_base 为单位矩阵 (odom == base 在原点)
  // 更准确: T_map_odom = T_map_base * T_base_odom
  // 其中 T_base_odom = inv(T_odom_base) 来自FAST-LIO2

  // 将校正值作为新的 map->odom
  // (在简化模型中, T_odom_base = I, 所以 map_to_odom = T_map_base)
  applyEmaCorrection(T_map_base);
}

void LocalizationManager::applyEmaCorrection(const Eigen::Matrix4d & correction)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_) {
    map_to_odom_ = correction;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "从AprilTag获取初始定位");
    return;
  }

  // 离群值检查
  Eigen::Vector3d delta = correction.block<3,1>(0,3) - map_to_odom_.block<3,1>(0,3);
  double dist = delta.norm();

  Eigen::AngleAxisd delta_aa(
    correction.block<3,3>(0,0) * map_to_odom_.block<3,3>(0,0).transpose());
  double angle_deg = std::abs(delta_aa.angle()) * 180.0 / M_PI;

  if (dist > max_translation_jump_ || angle_deg > max_yaw_jump_) {
    RCLCPP_WARN(this->get_logger(),
                "已拒绝大幅度校正: d=%.3fm, θ=%.1f°", dist, angle_deg);
    return;
  }

  // EMA指数移动平均滤波
  double a = ema_alpha_;
  map_to_odom_.block<3,1>(0,3) = a * correction.block<3,1>(0,3) +
                                 (1.0 - a) * map_to_odom_.block<3,1>(0,3);

  Eigen::Quaterniond q_new(correction.block<3,3>(0,0));
  Eigen::Quaterniond q_old(map_to_odom_.block<3,3>(0,0));
  Eigen::Quaterniond q_ema = q_old.slerp(a, q_new);
  map_to_odom_.block<3,3>(0,0) = q_ema.toRotationMatrix();

  RCLCPP_DEBUG(this->get_logger(),
               "已应用校正: d=%.3fm, θ=%.1f° (EMA α=%.2f)",
               dist, angle_deg, a);
}

void LocalizationManager::tfBroadcastLoop()
{
  rclcpp::Rate rate(tf_rate_);

  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (initialized_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;

        tf.transform.translation.x = map_to_odom_(0,3);
        tf.transform.translation.y = map_to_odom_(1,3);
        tf.transform.translation.z = map_to_odom_(2,3);

        Eigen::Quaterniond q(map_to_odom_.block<3,3>(0,0));
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(tf);
      }
    }
    rate.sleep();
  }
}

}  // namespace robocon_localization


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robocon_localization::LocalizationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
