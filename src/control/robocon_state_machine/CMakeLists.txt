cmake_minimum_required(VERSION 3.8)
project(robocon_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robocon_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

add_library(robocon_fsm_lib SHARED
  src/mission_fsm_node.cpp
  src/obstacle_handler.cpp
  src/handlers/pole_weave_handler.cpp
  src/handlers/gravel_pit_handler.cpp
  src/handlers/height_bar_handler.cpp
  src/handlers/slope_handler.cpp
  src/handlers/bridge_handler.cpp
  src/handlers/t_steps_handler.cpp
  src/handlers/high_wall_handler.cpp
)

target_include_directories(robocon_fsm_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robocon_fsm_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  robocon_interfaces
  tf2
  tf2_ros
  yaml_cpp_vendor
)

add_executable(mission_fsm_node src/mission_fsm_main.cpp)
target_link_libraries(mission_fsm_node robocon_fsm_lib)
ament_target_dependencies(mission_fsm_node rclcpp)

install(TARGETS
  robocon_fsm_lib
  mission_fsm_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/waypoint_nav_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
