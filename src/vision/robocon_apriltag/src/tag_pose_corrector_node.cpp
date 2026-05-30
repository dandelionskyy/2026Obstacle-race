#include "robocon_apriltag/tag_pose_corrector.hpp"

#include <yaml-cpp/yaml.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <filesystem>

namespace robocon_apriltag
{

Eigen::Matrix4d TagWorldPose::toMatrix() const
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
  T.block<3,3>(0,0) = q.toRotationMatrix();
  T(0,3) = x; T(1,3) = y; T(2,3) = z;
  return T;
}

TagPoseCorrectorNode::TagPoseCorrectorNode(const rclcpp::NodeOptions & options)
: Node("tag_pose_corrector", options)
{
  // 加载标签世界位姿
  std::string tag_poses_file = this->declare_parameter(
    "tag_poses_file",
    std::string("config/tag_world_poses.yaml"));

  if (!loadTagWorldPoses(tag_poses_file)) {
    RCLCPP_ERROR(this->get_logger(),
                 "加载标签世界位姿失败: %s", tag_poses_file.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(),
                "已加载 %zu 个标签世界位姿", tag_poses_.size());
  }

  // 参数
  ema_alpha_ = this->declare_parameter("ema_alpha", 0.3);
  max_translation_jump_ = this->declare_parameter("max_translation_jump", 0.5);
  max_rotation_jump_ = this->declare_parameter("max_rotation_jump", 15.0);
  min_correction_interval_ = this->declare_parameter("min_correction_interval", 0.5);

  // 订阅者
  detection_sub_ = this->create_subscription<robocon_interfaces::msg::TagDetectionArray>(
    "/tag_detections", rclcpp::QoS(10),
    std::bind(&TagPoseCorrectorNode::detectionCallback, this, std::placeholders::_1));

  // 发布者
  correction_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/tag_pose_correction", rclcpp::QoS(10));

  last_correction_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "TagPoseCorrectorNode 已初始化");
}

bool TagPoseCorrectorNode::loadTagWorldPoses(const std::string & file_path)
{
  try {
    YAML::Node config = YAML::LoadFile(file_path);

    if (!config["tags"] || !config["tags"].IsSequence()) {
      RCLCPP_ERROR(this->get_logger(), "标签世界位姿文件格式无效");
      return false;
    }

    for (const auto & tag_node : config["tags"]) {
      int id = tag_node["id"].as<int>();
      TagWorldPose pose;
      pose.x = tag_node["position"]["x"].as<double>();
      pose.y = tag_node["position"]["y"].as<double>();
      pose.z = tag_node["position"]["z"].as<double>();
      pose.roll  = tag_node["orientation"]["roll"].as<double>() * M_PI / 180.0;
      pose.pitch = tag_node["orientation"]["pitch"].as<double>() * M_PI / 180.0;
      pose.yaw   = tag_node["orientation"]["yaw"].as<double>() * M_PI / 180.0;
      tag_poses_[id] = pose;

      RCLCPP_INFO(this->get_logger(), "  Tag %d: (%.2f, %.2f, %.2f) yaw=%.1f°",
                  id, pose.x, pose.y, pose.z, pose.yaw * 180.0 / M_PI);
    }
    return true;
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "YAML解析错误: %s", e.what());
    return false;
  }
}

void TagPoseCorrectorNode::detectionCallback(
  const robocon_interfaces::msg::TagDetectionArray::SharedPtr msg)
{
  if (tag_poses_.empty()) return;

  // 限速校正
  auto now = this->now();
  if ((now - last_correction_time_).seconds() < min_correction_interval_) {
    return;
  }

  // 找到最置信且已知世界位姿的标签
  const robocon_interfaces::msg::TagDetection * best_detection = nullptr;
  double best_conf = 0.0;
  int best_tag_id = -1;

  for (const auto & det : msg->detections) {
    if (tag_poses_.count(det.tag_id) && det.confidence > best_conf) {
      best_conf = det.confidence;
      best_detection = &det;
      best_tag_id = det.tag_id;
    }
  }

  if (!best_detection || best_tag_id < 0) return;

  // 通过标签观测计算 map->base_link
  // T_map_base = T_map_tag * T_tag_camera * T_camera_base
  //            = T_map_tag * inv(T_camera_tag) * T_camera_base

  Eigen::Matrix4d T_map_tag = tag_poses_[best_tag_id].toMatrix();

  // 从检测结果获取 T_camera_tag
  const auto & p = best_detection->pose;
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x,
                       p.orientation.y, p.orientation.z);
  Eigen::Matrix4d T_camera_tag = Eigen::Matrix4d::Identity();
  T_camera_tag.block<3,3>(0,0) = q.toRotationMatrix();
  T_camera_tag(0,3) = p.position.x;
  T_camera_tag(1,3) = p.position.y;
  T_camera_tag(2,3) = p.position.z;

  // T_tag_camera = inv(T_camera_tag)
  Eigen::Matrix4d T_tag_camera = T_camera_tag.inverse();

  // 目前假设相机位于 base_link 处 (简化模型)。
  // TODO: 使用TF查询实际相机外参。
  // 实际: T_map_base = T_map_tag * inv(T_camera_tag) * T_camera_base
  Eigen::Matrix4d T_camera_base = Eigen::Matrix4d::Identity();
  T_camera_base(0,3) = 0.25;  // 相机相对于base_link的前向偏移
  T_camera_base(2,3) = 0.30;  // 相机高度

  Eigen::Matrix4d T_map_base = T_map_tag * T_tag_camera * T_camera_base;

  // 离群值剔除
  if (!first_correction_) {
    Eigen::Vector3d delta_trans = T_map_base.block<3,1>(0,3) -
                                  current_map_to_base_.block<3,1>(0,3);
    double delta_dist = delta_trans.norm();

    Eigen::Matrix3d delta_rot = T_map_base.block<3,3>(0,0) *
                                current_map_to_base_.block<3,3>(0,0).transpose();
    Eigen::AngleAxisd delta_aa(delta_rot);
    double delta_angle = std::abs(delta_aa.angle()) * 180.0 / M_PI;

    if (delta_dist > max_translation_jump_ || delta_angle > max_rotation_jump_) {
      RCLCPP_WARN(this->get_logger(),
                  "已拒绝离群校正: tag=%d, d_dist=%.3fm, d_angle=%.1f°",
                  best_tag_id, delta_dist, delta_angle);
      return;
    }

    // EMA指数移动平均滤波
    double alpha = ema_alpha_;
    T_map_base.block<3,1>(0,3) = alpha * T_map_base.block<3,1>(0,3) +
                                 (1.0 - alpha) * current_map_to_base_.block<3,1>(0,3);

    Eigen::Quaterniond q_new(T_map_base.block<3,3>(0,0));
    Eigen::Quaterniond q_old(current_map_to_base_.block<3,3>(0,0));
    Eigen::Quaterniond q_ema = q_old.slerp(alpha, q_new);
    T_map_base.block<3,3>(0,0) = q_ema.toRotationMatrix();
  }

  current_map_to_base_ = T_map_base;
  first_correction_ = false;
  last_correction_time_ = now;

  // 发布校正值
  auto correction = geometry_msgs::msg::PoseWithCovarianceStamped();
  correction.header.stamp = now;
  correction.header.frame_id = "map";
  correction.pose.pose.position.x = T_map_base(0,3);
  correction.pose.pose.position.y = T_map_base(1,3);
  correction.pose.pose.position.z = T_map_base(2,3);

  Eigen::Quaterniond q_out(T_map_base.block<3,3>(0,0));
  correction.pose.pose.orientation.w = q_out.w();
  correction.pose.pose.orientation.x = q_out.x();
  correction.pose.pose.orientation.y = q_out.y();
  correction.pose.pose.orientation.z = q_out.z();

  // 协方差 (对角矩阵, 依赖于标签的不确定度)
  for (int i = 0; i < 36; i++) correction.pose.covariance[i] = 0.0;
  correction.pose.covariance[0]  = 0.01;   // x方差
  correction.pose.covariance[7]  = 0.01;   // y方差
  correction.pose.covariance[14] = 0.02;   // z方差
  correction.pose.covariance[21] = 0.01;   // roll方差
  correction.pose.covariance[28] = 0.01;   // pitch方差
  correction.pose.covariance[35] = 0.005;  // yaw方差 (标签约束最紧的)

  correction_pub_->publish(correction);

  RCLCPP_DEBUG(this->get_logger(),
               "标签 %d 校正: (%.2f, %.2f, %.2f) yaw=%.1f° conf=%.2f",
               best_tag_id,
               T_map_base(0,3), T_map_base(1,3), T_map_base(2,3),
               std::atan2(T_map_base(1,0), T_map_base(0,0)) * 180.0 / M_PI,
               best_conf);
}

}  // namespace robocon_apriltag


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robocon_apriltag::TagPoseCorrectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
