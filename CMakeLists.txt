cmake_minimum_required(VERSION 2.9.3)
project(tams_glass)

set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
  roscpp
  std_srvs
  visualization_msgs
  interactive_markers
  image_transport
  sensor_msgs
  cv_bridge
  eigen_conversions
  rosbag
  moveit_msgs
  moveit_ros_planning_interface
  image_geometry
  camera_info_manager
  camera_calibration_parsers
  pcl_conversions
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf
  tf_conversions
  sensor_msgs
  cv_bridge
  actionlib_msgs
  image_transport
  eigen_conversions
  rosbag
  moveit_msgs
  moveit_ros_planning_interface
  image_geometry
  camera_info_manager
  camera_calibration_parsers
  pcl_conversions
  pcl_ros
)

FIND_PACKAGE