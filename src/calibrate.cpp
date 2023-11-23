#include "common.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "tams_glass_calibrate", 0);

  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  if (argc < 2) {
    ROS_ERROR_STREAM("usage: tams_glass calibrate <data.bag>");
    return -1;
  }

  std::string name_prefix = "";
  std::string camera_image_topic = "/camera_snapshot/ir/image";
  std::string camera_info_topic = "/camera_snapshot/ir/camera_info";

  if (1) {
    name_prefix = "";
    camera_image_topic = "/camera_snapshot/ir/image";
    camera_info_topic = "/camera_snapshot/ir/camera_info";
  }

  std::string tip_link = "ur5_tool0";
  std::string base_link = "table_top";

  moveit::planning_interface::MoveGroupInterface move_group("arm");

  moveit::core::RobotState robot_state = *move_group.getCurrentState();

  cv: