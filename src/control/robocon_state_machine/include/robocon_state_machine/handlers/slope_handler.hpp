#ifndef ROBOCON_STATE_MACHINE__HANDLERS__SLOPE_HANDLER_HPP_
#define ROBOCON_STATE_MACHINE__HANDLERS__SLOPE_HANDLER_HPP_
#include "robocon_state_machine/obstacle_handler.hpp"
namespace robocon_state_machine {
class SlopeHandler : public ObstacleHandler {
public:
  void init(rclcpp::Node * node) override;
  StateResult update(const SensorData & data) override;
  const char * name() const override { return "Slope"; }
};
}  // namespace robocon_state_machine
#endif
