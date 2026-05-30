#include <rclcpp/rclcpp.hpp>
#include "robocon_state_machine/mission_fsm.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robocon_state_machine::MissionFSM>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
