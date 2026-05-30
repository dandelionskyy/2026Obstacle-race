#ifndef ROBOCON_PERCEPTION__GROUND_SEGMENTER_HPP_
#define ROBOCON_PERCEPTION__GROUND_SEGMENTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace robocon_perception
{

struct GroundPlane {
  bool valid = false;
  Eigen::Vector4f coefficients;  // ax + by + cz + d = 0（平面方程）
};

class GroundSegmenter
{
public:
  GroundSegmenter();

  void setDistanceThreshold(double threshold);

  GroundPlane segment(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & ground_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & non_ground_cloud);

private:
  double distance_threshold_{0.03};
  int max_iterations_{200};
};

}  // namespace robocon_perception

#endif  // ROBOCON_PERCEPTION__GROUND_SEGMENTER_HPP_
