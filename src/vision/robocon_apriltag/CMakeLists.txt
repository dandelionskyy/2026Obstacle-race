cmake_minimum_required(VERSION 3.8)
project(robocon_apriltag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robocon_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# C++ tag_pose_corrector node
add_executable(tag_pose_corrector_node
  src/tag_pose_corrector_node.cpp
)

target_include_directories(tag_pose_corrector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tag_pose_corrector_node
  rclcpp
  geometry_msgs
  robocon_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  yaml-cpp
)

target_link_libraries(tag_pose_corrector_node
  ${YAML_CPP_LIBRARIES}
)

# Python apriltag detector node
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(PROGRAMS
  src/apriltag_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tag_pose_corrector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
