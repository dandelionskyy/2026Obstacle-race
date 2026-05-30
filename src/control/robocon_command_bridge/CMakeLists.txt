cmake_minimum_required(VERSION 3.8)
project(robocon_command_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robocon_interfaces REQUIRED)

include_directories(include)

add_executable(command_bridge_node
  src/command_bridge_node.cpp
)

target_include_directories(command_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(command_bridge_node
  rclcpp
  geometry_msgs
  robocon_interfaces
)

install(TARGETS command_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
