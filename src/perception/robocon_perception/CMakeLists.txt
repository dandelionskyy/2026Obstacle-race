cmake_minimum_required(VERSION 3.8)
project(robocon_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robocon_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation filters kdtree search features)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(obstacle_detector_lib SHARED
  src/obstacle_detector_node.cpp
  src/ground_segmenter.cpp
  src/object_classifier.cpp
)

target_include_directories(obstacle_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(obstacle_detector_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  robocon_interfaces
  pcl_conversions
  PCL
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

add_executable(obstacle_detector_node src/obstacle_detector_main.cpp)
target_link_libraries(obstacle_detector_node obstacle_detector_lib)
ament_target_dependencies(obstacle_detector_node rclcpp)

install(TARGETS
  obstacle_detector_lib
  obstacle_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
