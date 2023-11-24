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

  cv::Size chessboard_size(5, 4);
  double scale = 0.0245;

  std::vector<cv::Point3f> corners3d;
  for (int i = 0; i < chessboard_size.height; i++) {
    for (int j = 0; j < chessboard_size.width; j++) {
      corners3d.emplace_back(j * scale, i * scale, 0);
    }
  }

  std::vector<std::vector<cv::Point3f>> c3d;
  std::vector<std::vector<cv::Point2f>> c2d;

  std::vector<sensor_msgs::JointState> joint_states;

  cv::Size image_size;

  ros::Publisher display_planned_path =
      node.advertise<moveit_msgs::DisplayTrajectory>(
          "/move_group/display_planned_path", 1, true);

  ros::Publisher vis_pub = node.advertise<visualization_msgs::MarkerArray>(
      "/tams_glass/visualization", 10);

  ROS_INFO_STREAM("reading and analyzing images");

  sensor_msgs::CameraInfo::Ptr camera_info;

  {
