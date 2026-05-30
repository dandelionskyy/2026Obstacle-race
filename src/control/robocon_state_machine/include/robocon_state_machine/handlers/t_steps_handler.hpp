#ifndef ROBOCON_STATE_MACHINE__HANDLERS__T_STEPS_HANDLER_HPP_
#define ROBOCON_STATE_MACHINE__HANDLERS__T_STEPS_HANDLER_HPP_
#include "robocon_state_machine/obstacle_handler.hpp"
namespace robocon_state_machine {
class TStepsHandler : public ObstacleHandler {
public:
  void init(rclcpp::Node * node) override;
  StateResult update(const SensorData & data) override;
  const char * name() const override { return "TSteps"; }
};
}  // namespace robocon_state_machine
#endif
