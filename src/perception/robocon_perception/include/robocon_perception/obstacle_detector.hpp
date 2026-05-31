#ifndef ROBOCON_PERCEPTION__OBSTACLE_DETECTOR_HPP_
#define ROBOCON_PERCEPTION__OBSTACLE_DETECTOR_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "robocon_interfaces/msg/obstacle_info.hpp"
#include "robocon_perception/ground_segmenter.hpp"
#include "robocon_perception/object_classifier.hpp"

namespace robocon_perception
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ObstacleDetectorNode() = default;

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishObstacles(const std::vector<robocon_interfaces::msg::ObstacleInfo> & obstacles);
  void stabilizeDetections(const robocon_interfaces::msg::ObstacleInfo & detection);

  // 参数加载
  void loadParameters();

  // 订阅
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // 发布者
  rclcpp::Publisher<robocon_interfaces::msg::ObstacleInfo>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 调试点云发布者 (PointXYZRGB → PointCloud2)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_non_ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_clusters_pub_;

  /// 辅助: 将 pcl::PointCloud<PointXYZRGB> 发布为 sensor_msgs::PointCloud2
  void publishDebugCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub);

  // 处理模块
  GroundSegmenter ground_segmenter_;
  ObjectClassifier classifier_;

  // 检测稳定化
  std::deque<robocon_interfaces::msg::ObstacleInfo> recent_detections_;
  int stabilization_frames_{5};
  double stabilization_threshold_{0.3};  // 米 — 最大位置方差

  // 参数
  std::string cloud_topic_;
  std::string base_frame_{"base_link"};
  double voxel_leaf_size_{0.05};
  double roi_min_x_{-5.0}, roi_max_x_{5.0};
  double roi_min_y_{-5.0}, roi_max_y_{5.0};
  double roi_min_z_{-1.0}, roi_max_z_{1.5};
  double cluster_tolerance_{0.08};
  int min_cluster_size_{15};
  int max_cluster_size_{20000};
  double ground_distance_threshold_{0.03};

  bool publish_debug_clouds_{false};
  int debug_publish_interval_{3};   // 每 N 帧发布一次调试云
  uint64_t frame_count_{0};

  std::mutex mutex_;
};

}  // namespace robocon_perception

#endif  // ROBOCON_PERCEPTION__OBSTACLE_DETECTOR_HPP_
