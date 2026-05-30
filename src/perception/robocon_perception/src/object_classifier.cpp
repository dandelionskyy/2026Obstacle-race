#include "robocon_perception/object_classifier.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <cmath>
#include <rclcpp/logging.hpp>

namespace robocon_perception
{

ObjectClassifier::ObjectClassifier()
{
}

void ObjectClassifier::setGroundPlane(const Eigen::Vector4f & plane_coeff)
{
  ground_plane_ = plane_coeff;
}

ClusterFeatures ObjectClassifier::extractFeatures(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cluster)
{
  ClusterFeatures f;

  // 包围盒
  pcl::getMinMax3D(*cluster, f.min_pt, f.max_pt);
  f.length = f.max_pt.x - f.min_pt.x;
  f.width  = f.max_pt.y - f.min_pt.y;
  f.height = f.max_pt.z - f.min_pt.z;
  f.bottom_z = f.min_pt.z;

  // 质心
  Eigen::Vector4f centroid4f;
  pcl::compute3DCentroid(*cluster, centroid4f);
  f.centroid = centroid4f.head<3>();

  // 地面平面上方高度
  float a = ground_plane_[0], b = ground_plane_[1], c = ground_plane_[2], d = ground_plane_[3];
  float denom = std::sqrt(a * a + b * b + c * c);
  if (denom > 1e-6) {
    f.height_above_ground = std::abs(a * f.centroid[0] + b * f.centroid[1] + c * f.centroid[2] + d) / denom;
  } else {
    f.height_above_ground = f.centroid[2];
  }

  // PCA主成分分析用于形状分析
  pcl::PCA<pcl::PointXYZI> pca;
  pca.setInputCloud(cluster);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();

  // 降序排列
  if (eigenvalues[0] < eigenvalues[1]) std::swap(eigenvalues[0], eigenvalues[1]);
  if (eigenvalues[1] < eigenvalues[2]) std::swap(eigenvalues[1], eigenvalues[2]);
  if (eigenvalues[0] < eigenvalues[1]) std::swap(eigenvalues[0], eigenvalues[1]);

  float l1 = eigenvalues[0], l2 = eigenvalues[1], l3 = eigenvalues[2];
  if (l1 > 1e-6) {
    f.linearity  = (l1 - l2) / l1;
    f.planarity  = (l2 - l3) / l1;
    f.sphericity = l3 / l1;
  } else {
    f.linearity = f.planarity = f.sphericity = 0;
  }

  // 表面法向量 (最小特征值对应的特征向量)
  f.surface_normal = pca.getEigenVectors().col(2);

  return f;
}

robocon_interfaces::msg::ObstacleInfo ObjectClassifier::classify(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cluster)
{
  robocon_interfaces::msg::ObstacleInfo info;
  info.type = robocon_interfaces::msg::ObstacleInfo::UNKNOWN;
  info.confidence = 0.0;

  if (cluster->empty()) {
    return info;
  }

  auto features = extractFeatures(cluster);

  // 按优先级顺序分类
  if (isPole(features)) {
    info.type = robocon_interfaces::msg::ObstacleInfo::POLE;
    info.confidence = 0.85;
  } else if (isHeightBar(features)) {
    info.type = robocon_interfaces::msg::ObstacleInfo::HEIGHT_BAR;
    info.confidence = 0.75;
  } else if (isHighWall(features)) {
    info.type = robocon_interfaces::msg::ObstacleInfo::HIGH_WALL;
    info.confidence = 0.8;
  } else if (isTSteps(features)) {
    info.type = robocon_interfaces::msg::ObstacleInfo::T_STEPS;
    info.confidence = 0.65;
  } else if (isSlope(features)) {
    info.type = robocon_interfaces::msg::ObstacleInfo::SLOPE;
    info.confidence = 0.7;
  } else if (isBridge(features)) {
    info.type = robocon_interfaces::msg::ObstacleInfo::BRIDGE;
    info.confidence = 0.65;
  }

  // 距离和朝向角
  info.distance = features.centroid.norm();
  info.heading = std::atan2(features.centroid[1], features.centroid[0]);

  return info;
}

bool ObjectClassifier::isPole(const ClusterFeatures & f) const
{
  // 垂直、细长、高
  return f.linearity > 0.8 &&
         f.height >= pole_min_height_ &&
         std::max(f.length, f.width) <= pole_max_diameter_;
}

bool ObjectClassifier::isGravelPit(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & ground_cloud) const
{
  // 检查地面点的强度方差 (砾石 = 高方差)
  if (ground_cloud->size() < 100) return false;

  double sum = 0, sum_sq = 0;
  for (const auto & pt : ground_cloud->points) {
    sum += pt.intensity;
    sum_sq += pt.intensity * pt.intensity;
  }
  double mean = sum / ground_cloud->size();
  double variance = sum_sq / ground_cloud->size() - mean * mean;

  // 砾石/碎石产生高方差的激光雷达强度值
  return variance > 500.0;  // 阈值待实际数据确定
}

bool ObjectClassifier::isHeightBar(const ClusterFeatures & f) const
{
  // 地面上方特定高度的水平横杆
  return f.height_above_ground >= bar_z_min_ &&
         f.height_above_ground <= bar_z_max_ &&
         f.length >= bar_min_length_ &&
         std::min(f.height, f.width) <= bar_max_thickness_ &&
         f.linearity > 0.7;
}

bool ObjectClassifier::isSlope(const ClusterFeatures & f) const
{
  // 倾斜平面
  if (f.planarity < 0.6) return false;

  // 表面法向量与垂直方向(重力)之间的夹角
  float vertical_angle = std::acos(
    std::abs(f.surface_normal.dot(Eigen::Vector3f::UnitZ()))
  );
  float vertical_degrees = vertical_angle * 180.0 / M_PI;

  return vertical_degrees >= slope_min_inclination_ &&
         vertical_degrees <= 45.0 &&
         f.length > 1.0 && f.width > 1.0;
}

bool ObjectClassifier::isBridge(const ClusterFeatures & f) const
{
  // 抬高的平坦平台
  return f.planarity > 0.7 &&
         f.height_above_ground > 0.1 &&
         f.length > 0.5 &&
         f.width > 0.5;
}

bool ObjectClassifier::isTSteps(const ClusterFeatures & f) const
{
  // 台阶: 中等高度, 有显著深度
  return f.height > step_min_height_ * 2 &&
         f.length > step_min_depth_ * 2 &&
         f.width > 0.3 &&
         f.planarity < 0.5;  // 非单一平面 (多个台阶 = 非平面)
}

bool ObjectClassifier::isHighWall(const ClusterFeatures & f) const
{
  // 垂直平面
  return f.planarity > 0.7 &&
         f.height >= wall_min_height_ &&
         std::max(f.length, f.width) >= wall_min_width_;
}

}  // namespace robocon_perception
