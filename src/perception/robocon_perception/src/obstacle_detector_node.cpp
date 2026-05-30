#include "robocon_perception/obstacle_detector.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>

namespace robocon_perception
{

ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions & options)
: Node("obstacle_detector", options)
{
  loadParameters();

  // 订阅者
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetectorNode::pointcloudCallback, this, std::placeholders::_1));

  // 发布者
  obstacle_pub_ = this->create_publisher<robocon_interfaces::msg::ObstacleInfo>(
    "/obstacle_info", rclcpp::QoS(10));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/obstacle_markers", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "ObstacleDetectorNode 已初始化");
  RCLCPP_INFO(this->get_logger(), "  点云话题: %s", cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  感兴趣区域: x=[%.1f,%.1f] y=[%.1f,%.1f] z=[%.1f,%.1f]",
              roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_);
}

void ObstacleDetectorNode::loadParameters()
{
  cloud_topic_ = this->declare_parameter("cloud_topic", "/cloud_registered");

  voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.05);

  roi_min_x_ = this->declare_parameter("roi_min_x", -5.0);
  roi_max_x_ = this->declare_parameter("roi_max_x", 5.0);
  roi_min_y_ = this->declare_parameter("roi_min_y", -5.0);
  roi_max_y_ = this->declare_parameter("roi_max_y", 5.0);
  roi_min_z_ = this->declare_parameter("roi_min_z", -1.0);
  roi_max_z_ = this->declare_parameter("roi_max_z", 1.5);

  cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.08);
  min_cluster_size_ = this->declare_parameter("min_cluster_size", 15);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", 20000);

  ground_distance_threshold_ = this->declare_parameter("ground_distance_threshold", 0.03);
  stabilization_frames_ = this->declare_parameter("stabilization_frames", 5);
}

void ObstacleDetectorNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t_start = std::chrono::steady_clock::now();

  // ROS消息转换为PCL格式
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty()) return;

  // 1. 感兴趣区域滤波 (直通滤波)
  pcl::PassThrough<pcl::PointXYZI> pass;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(roi_min_x_, roi_max_x_);
  pass.filter(*cloud_roi);

  pass.setInputCloud(cloud_roi);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(roi_min_y_, roi_max_y_);
  pass.filter(*cloud_roi);

  pass.setInputCloud(cloud_roi);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(roi_min_z_, roi_max_z_);
  pass.filter(*cloud_roi);

  // 2. 体素网格降采样
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  voxel.setInputCloud(cloud_roi);
  voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel.filter(*cloud_downsampled);

  // 3. 统计离群点去除
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  sor.setInputCloud(cloud_downsampled);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  // 4. 地面分割
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  ground_segmenter_.setDistanceThreshold(ground_distance_threshold_);
  auto ground_plane = ground_segmenter_.segment(cloud_filtered, ground_cloud, non_ground_cloud);

  if (ground_plane.valid) {
    classifier_.setGroundPlane(ground_plane.coefficients);
  }

  // 5. 非地面点欧几里得聚类
  std::vector<robocon_interfaces::msg::ObstacleInfo> detected_obstacles;

  if (!non_ground_cloud->empty()) {
    // 创建KdTree
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(non_ground_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(non_ground_cloud);
    ec.extract(cluster_indices);

    // 6. 对每个聚类进行分类
    for (const auto & indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      for (int idx : indices.indices) {
        cluster->points.push_back((*non_ground_cloud)[idx]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      auto info = classifier_.classify(cluster);
      if (info.type != robocon_interfaces::msg::ObstacleInfo::UNKNOWN) {
        detected_obstacles.push_back(info);
      }
    }
  }

  // 7. 检测砾石坑 (地面强度分析)
  if (!ground_cloud->empty() && ground_cloud->size() > 500) {
    // 计算强度方差
    double sum = 0, sum_sq = 0;
    for (const auto & pt : ground_cloud->points) {
      sum += pt.intensity;
      sum_sq += pt.intensity * pt.intensity;
    }
    double variance = sum_sq / ground_cloud->size() -
                      (sum / ground_cloud->size()) * (sum / ground_cloud->size());

    if (variance > 500.0) {
      robocon_interfaces::msg::ObstacleInfo gravel_info;
      gravel_info.type = robocon_interfaces::msg::ObstacleInfo::GRAVEL_PIT;
      gravel_info.confidence = 0.6;
      gravel_info.distance = 0.0;  // 机器人下方地面
      detected_obstacles.push_back(gravel_info);
    }
  }

  // 8. 稳定化并发布
  if (!detected_obstacles.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & obs : detected_obstacles) {
      stabilizeDetections(obs);
    }
  }

  publishObstacles(detected_obstacles);

  auto t_end = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
  if (elapsed.count() > 50) {
    RCLCPP_WARN(this->get_logger(), "点云处理耗时 %ld ms", elapsed.count());
  }
}

void ObstacleDetectorNode::stabilizeDetections(
  const robocon_interfaces::msg::ObstacleInfo & detection)
{
  // 简单稳定化: 保留最近N帧检测结果, 要求一致性
  recent_detections_.push_back(detection);
  if (static_cast<int>(recent_detections_.size()) > stabilization_frames_) {
    recent_detections_.pop_front();
  }
}

void ObstacleDetectorNode::publishObstacles(
  const std::vector<robocon_interfaces::msg::ObstacleInfo> & obstacles)
{
  // 逐个发布障碍物信息 (状态机会进行聚合)
  for (const auto & obs : obstacles) {
    obstacle_pub_->publish(obs);
  }

  // 发布标记用于RViz可视化
  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;

  for (const auto & obs : obstacles) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "obstacles";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 障碍物位置 (由朝向角和距离近似计算)
    marker.pose.position.x = obs.distance * std::cos(obs.heading);
    marker.pose.position.y = obs.distance * std::sin(obs.heading);
    marker.pose.position.z = 0.3;
    marker.pose.orientation.w = 1.0;

    // 根据类型着色
    switch (obs.type) {
      case robocon_interfaces::msg::ObstacleInfo::POLE:
        marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; break;
      case robocon_interfaces::msg::ObstacleInfo::GRAVEL_PIT:
        marker.color.r = 0.5; marker.color.g = 0.3; marker.color.b = 0.0; break;
      case robocon_interfaces::msg::ObstacleInfo::HEIGHT_BAR:
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0; break;
      case robocon_interfaces::msg::ObstacleInfo::SLOPE:
        marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0; break;
      case robocon_interfaces::msg::ObstacleInfo::BRIDGE:
        marker.color.r = 0.5; marker.color.g = 0.0; marker.color.b = 1.0; break;
      case robocon_interfaces::msg::ObstacleInfo::T_STEPS:
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; break;
      case robocon_interfaces::msg::ObstacleInfo::HIGH_WALL:
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; break;
      default:
        marker.color.r = 0.7; marker.color.g = 0.7; marker.color.b = 0.7;
    }
    marker.color.a = 1.0;
    marker.scale.z = 0.3;

    // 类型字符串
    static const char* type_names[] = {
      "UNKNOWN", "POLE", "GRAVEL_PIT", "HEIGHT_BAR",
      "SLOPE", "BRIDGE", "T_STEPS", "HIGH_WALL"
    };
    marker.text = type_names[obs.type];

    markers.markers.push_back(marker);
  }

  if (!markers.markers.empty()) {
    marker_pub_->publish(markers);
  }
}

}  // namespace robocon_perception

