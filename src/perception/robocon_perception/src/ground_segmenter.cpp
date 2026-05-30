#include "robocon_perception/ground_segmenter.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <rclcpp/logging.hpp>

namespace robocon_perception
{

GroundSegmenter::GroundSegmenter()
{
}

void GroundSegmenter::setDistanceThreshold(double threshold)
{
  distance_threshold_ = threshold;
}

GroundPlane GroundSegmenter::segment(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  pcl::PointCloud<pcl::PointXYZI>::Ptr & ground_cloud,
  pcl::PointCloud<pcl::PointXYZI>::Ptr & non_ground_cloud)
{
  GroundPlane result;

  if (cloud->empty()) {
    return result;
  }

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);
  seg.setMaxIterations(max_iterations_);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("GroundSegmenter"),
                "未检测到地面平面");
    // 回退: 所有点视为非地面点
    *non_ground_cloud = *cloud;
    ground_cloud->clear();
    return result;
  }

  // 存储平面系数: ax + by + cz + d = 0
  result.valid = true;
  result.coefficients = Eigen::Vector4f(
    coefficients->values[0],
    coefficients->values[1],
    coefficients->values[2],
    coefficients->values[3]
  );

  // 提取地面点和非地面点
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  extract.setNegative(false);
  extract.filter(*ground_cloud);

  extract.setNegative(true);
  extract.filter(*non_ground_cloud);

  return result;
}

}  // namespace robocon_perception
