#ifndef ROBOCON_APRILTAG__TAG_POSE_CORRECTOR_HPP_
#define ROBOCON_APRILTAG__TAG_POSE_CORRECTOR_HPP_

#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <Eigen/Dense>

#include "robocon_interfaces/msg/tag_detection_array.hpp"

namespace robocon_apriltag
{

struct TagWorldPose {
  double x, y, z;
  double roll, pitch, yaw;
  Eigen::Matrix4d toMatrix() const;
};

class TagPoseCorrectorNode : public rclcpp::Node
{
public:
  explicit TagPoseCorrectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TagPoseCorrectorNode() = default;

private:
  void detectionCallback(const robocon_interfaces::msg::TagDetectionArray::SharedPtr msg);
  bool loadTagWorldPoses(const std::string & file_path);

  // 订阅
  rclcpp::Subscription<robocon_interfaces::msg::TagDetectionArray>::SharedPtr detection_sub_;

  // 发布者
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr correction_pub_;

  // 标签世界位姿数据库
  std::map<int, TagWorldPose> tag_poses_;

  // EMA 滤波器
  double ema_alpha_{0.3};
  bool first_correction_{true};
  Eigen::Matrix4d current_map_to_base_{Eigen::Matrix4d::Identity()};

  // 异常值剔除阈值
  double max_translation_jump_{0.5};   // 米
  double max_rotation_jump_{15.0};     // 度

  // 两次修正之间的最小间隔
  rclcpp::Time last_correction_time_;
  double min_correction_interval_{0.5};  // 秒
};

}  // namespace robocon_apriltag

#endif  // ROBOCON_APRILTAG__TAG_POSE_CORRECTOR_HPP_
