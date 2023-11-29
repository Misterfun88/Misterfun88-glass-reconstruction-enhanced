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
    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read);
    ros::Time last_time = ros::Time(0);
    cv::Mat image_accumulator;
    sensor_msgs::JointState last_joint_state, image_joint_state;

    auto processImage = [&](cv::Mat image) {
      double lo, hi;
      cv::minMaxIdx(image, &lo, &hi);
      image = image * (254.0 / hi);
      image.convertTo(image, CV_8U);

      image_size = cv::Size(image.cols, image.rows);

      std::vector<cv::Point2f> chessboard_corners;
      bool chessboard_found = cv::findChessboardCorners(
          image, chessboard_size, chessboard_corners,
          cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_FILTER_QUADS);
      if (!chessboard_found) {
        ROS_INFO_STREAM("no chessboard found in image");
        return;
      }

      cv::cornerSubPix(
          image, chessboard_corners, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

      c2d.emplace_back(chessboard_corners);
      c3d.emplace_back(corners3d);

      joint_states.emplace_back(image_joint_state);

      cv::drawChessboardCorners(image, chessboard_size, chessboard_corners,
                                true);

      cv::circle(image, chessboard_corners[0], 15, cv::Scalar(255));
      cv::circle(image, chessboard_corners[1], 10, cv::Scalar(255));

      cv::imshow("image", image);
      cv::waitKey(1);
    };

    for (const rosbag::MessageInstance &bag_message : rosbag::View(bag)) {

      if (!ros::ok()) {
        ROS_ERROR_STREAM("canceled");
        return -1;
      }

      if (bag_message.getTopic() == "/joint_states") {
        if (auto joint_state_message =
                bag_message.instantiate<sensor_msgs::JointState>()) {
          last_joint_state = *joint_state_message;
        }
      }

      if (bag_message.getTopic() == camera_info_topic) {
        if (!camera_info) {
          if (auto camera_info_msg =
                  bag_message.instantiate<sensor_msgs::CameraInfo>()) {
            camera_info =
                boost::make_shared<sensor_msgs::CameraInfo>(*camera_info_msg);
          }
        }
      }

      if (bag_message.getTopic() == camera_image_topic) {
        if (auto image_message =
                bag_message.instantiate<sensor_msgs::Image>()) {

          ros::Time image_time = image_message->header.stamp;

          auto cv_image = cv_bridge::toCvCopy(image_message);

          cv::Mat image = cv_image->image;

          if (image.channels() > 1) {
            cv::cvtColor(image, image, CV_BGR2GRAY);
          }

          image.convertTo(image, CV_32FC1);

          if ((image_time - last_time).toSec() > 0.5 &&
              image_accumulator.rows != 0 && image_accumulator.cols != 0) {
            processImage(image_accumulator);
            image_accumulator = cv::Mat();
          }

          if (image_accumulator.rows == 0 || image_accumulator.cols == 0) {
            image_accumulator = image;
          } else {
            image_accumulator += image;
          }
          image_joint_state = last_joint_state;

          last_time = image_time;
        }
      }
    }
  }

  ROS_INFO_STREAM("display robot states");
  {
    ros::Duration(0.1).sleep();
    moveit_msgs::DisplayTrajectory msg;
    for (auto &state : joint_states) {
      msg.trajectory.emplace_back();
      msg.trajectory.back().joint_trajectory.joint_names = state.name;
      msg.trajectory.back().joint_trajectory.points.emplace_back();
      msg.trajectory.back().joint_trajectory.points.back().positions =
          state.position;
    }
    msg.trajectory_start.joint_state = joint_states.front();
    display_planned_path.publish(msg);
    ros::Duration(0.1).sleep();
  }

  if (0) {
    ROS_INFO_STREAM("calibrating intrinsics");
    cv::Mat camera_matrix, dist_coeffs, tvec, rvec;
    double err = cv::calibrateCamera(c3d, c2d, image_size, camera_matrix,
                                     dist_coeffs, tvec, rvec);
    ROS_INFO_STREAM("ready");
    ROS_INFO_STREAM("average reprojection error: " << err);
    ROS_INFO_STREAM("camera matrix\n" << camera_matrix);
    ROS_INFO_STREAM("distortion coefficients\n" << dist_coeffs);
    ROS_INFO_STREAM("translation vectors\n" << tvec);
    ROS_INFO_STREAM("rotation vectors\n" << rvec);

    camera_info->D.resize(dist_coeffs.rows * dist_coeffs.cols);
    for (size_t i = 0; i < dist_coeffs.rows * dist_coeffs.cols; i++) {
      camera_info->D[i] = dist_coeffs.at<double>(i);
    }

    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        ASSERT(row < camera_matrix.rows);
        ASSERT(col < camera_matrix.cols);
        camera_info->P[row * 4 + col] = camera_matrix.at<double>(row, col);
        camera_info->K[row * 3 + col] = camera_matrix.at<double>(row, col);
      }
    }
  }

  image_geometry::PinholeCameraModel camera;
  if (!camera.fromCameraInfo(camera_info)) {
    ROS_ERROR_STREAM("failed to load camera info");
    return -1;
  }

  struct Solution : Calibration {
    double scale = 1.0;
  };

  Solution solution;
  Solution temp_solution;

  solution.base_to_object = solution.base_to_object *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  solution.base_to_object =
      solution.base_to_object *
      Eigen::AngleAxisd(M_PI / -2, Eigen::Vector3d::UnitZ());

  solution.tip_to_camera =
      solution.tip_to_camera *
      Eigen::AngleAxisd(M_PI / -2, Eigen::Vector3d::UnitZ());

  if (1) {
    solution.tip_to_camera = Eigen::Isometry3d::Identity();
  }

  for (size_t iteration = 0; iteration < 500; iteration++) {

    if (!ros::ok()) {
      ROS_ERROR_STREAM("canceled");
      return -1;
    }

    ROS_INFO_STREAM("iteration " << iteration);

    size_t variable_count = 13;

    auto applyGradients = [&](const Eigen::VectorXd &gradients,
                              Solution &solution) {

      if (iteration > 200) {
        solution.tip_to_camera.translation() += gradients.segment<3>(0);
        solution.base_to_object.translation() += gradients.segment<3>(3);
      }

      {
        Eigen::Vector3d rotation = gradients.segment<3>(6);
        if (rotation.squaredNorm()) {
          solution.tip_to_camera =
              solution.tip_to_camera *
              Eigen::AngleAxisd(rotation.norm(), rotation.normalized());
        }
      }

      if (iteration > 100) {
        Eigen::Vector3d rotation = gradients.segment<3>(9);
        if (rotation.squaredNorm()) {
          solution.base_to_object =
              solution.base_to_object *
              Eigen::AngleAxisd(rotation.norm(), rotation.normalized());
        }
      }

    };

    Eigen::VectorXd temp_variables(variable_count);

    std::vector<Eigen::Triplet<double>> gradients;
    std::vector<double> residuals;

    visualization_msgs::Marker point_marker;
    point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.header.frame_id = "/world";
    point_marker.ns = "points";
    point_marker.pose.orientation.w = 1.0;
    point_marker.color.r = 1.0;
    point_marker.color.g = 1.0;
    point_marker.color.b = 1.0;
    point_marker.color.a = 1.0;
    point_marker.scale.x = 0.001;

    visualization_msgs::Marker ray_marker;
    ray_marker.type = visualization_msgs::Marker::LINE_LIST;
    ray_marker.action = visualization_msgs::Marker::ADD;
    