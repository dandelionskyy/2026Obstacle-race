#include "robocon_state_machine/obstacle_handler.hpp"

namespace robocon_state_machine
{

bool ObstacleHandler::isTagDetected(const SensorData & data, int tag_id, double min_conf) const
{
  if (!data.tag_detections) return false;
  for (const auto & det : data.tag_detections->detections) {
    if (det.tag_id == tag_id && det.confidence >= min_conf) {
      return true;
    }
  }
  return false;
}

double ObstacleHandler::getObstacleDistance(const SensorData & data, uint8_t obs_type) const
{
  if (!data.obstacle_info) return std::numeric_limits<double>::infinity();
  if (data.obstacle_info->type == obs_type) {
    return data.obstacle_info->distance;
  }
  return std::numeric_limits<double>::infinity();
}

}  // namespace robocon_state_machine
