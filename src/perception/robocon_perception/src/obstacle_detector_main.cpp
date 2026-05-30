#include <rclcpp/rclcpp.hpp>
#include "robocon_perception/obstacle_detector.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robocon_perception::ObstacleDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
