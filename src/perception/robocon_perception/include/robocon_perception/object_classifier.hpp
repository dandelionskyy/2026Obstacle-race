#ifndef ROBOCON_PERCEPTION__OBJECT_CLASSIFIER_HPP_
#define ROBOCON_PERCEPTION__OBJECT_CLASSIFIER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <vector>

#include "robocon_interfaces/msg/obstacle_info.hpp"

namespace robocon_perception
{

struct ClusterFeatures {
  // 包围盒
  pcl::PointXYZI min_pt, max_pt;
  double length, width, height;  // x, y, z 方向跨度

  // 质心
  Eigen::Vector3f centroid;

  // 基于特征值的形状描述子（λ1 >= λ2 >= λ3）
  double linearity;   // (λ1 - λ2) / λ1  — 杆状/线缆
  double planarity;   // (λ2 - λ3) / λ1  — 墙面/平面
  double sphericity;  // λ3 / λ1          — 紧凑块状

  // 离地高度
  double height_above_ground;
  double bottom_z;

  // 朝向（用于平面类物体）
  Eigen::Vector3f surface_normal;
};

class ObjectClassifier
{
public:
  ObjectClassifier();

  void setGroundPlane(const Eigen::Vector4f & plane_coeff);

  robocon_interfaces::msg::ObstacleInfo classify(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cluster);

private:
  ClusterFeatures extractFeatures(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cluster);

  bool isPole(const ClusterFeatures & f) const;
  bool isGravelPit(const pcl::PointCloud<pcl::PointXYZI>::Ptr & ground_cloud) const;
  bool isHeightBar(const ClusterFeatures & f) const;
  bool isSlope(const ClusterFeatures & f) const;
  bool isBridge(const ClusterFeatures & f) const;
  bool isTSteps(const ClusterFeatures & f) const;
  bool isHighWall(const ClusterFeatures & f) const;

  // 参数
  double pole_min_height_{0.5}, pole_max_diameter_{0.08};
  double bar_z_min_{0.25}, bar_z_max_{0.35}, bar_min_length_{0.8}, bar_max_thickness_{0.1};
  double wall_min_height_{0.3}, wall_min_width_{0.5};
  double slope_min_inclination_{5.0};  // 度
  double step_min_height_{0.08}, step_min_depth_{0.2};

  Eigen::Vector4f ground_plane_{0, 0, 1, 0};  // 默认：z轴朝上
};

}  // namespace robocon_perception

#endif  // ROBOCON_PERCEPTION__OBJECT_CLASSIFIER_HPP_
