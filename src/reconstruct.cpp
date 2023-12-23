
#include "common.h"

#include <pcl/point_types.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

struct VoxelGrid {
  Eigen::Vector3d mCenter;
  size_t mSize;
  double mResolution;
  std::vector<double> mData;
  double mDummy = 0.0;

public:
  VoxelGrid(const Eigen::Vector3d &center, size_t size, double resolution)
      : mCenter(center), mSize(size), mResolution(resolution),
        mData(size * size * size, 0.0) {}
  inline size_t dataIndex(size_t ix, size_t iy, size_t iz) const {
    size_t i = iz;
    i *= mSize;
    i += iy;
    i *= mSize;
    i += ix;
    return i;
  }
  const std::vector<double> &data() const { return mData; }
  inline size_t dataIndex(const Eigen::Vector3i &index) const {
    return dataIndex(index.x(), index.y(), index.z());
  }
  inline size_t size() const { return mSize; }
  inline double resolution() const { return mResolution; }
  inline double at(ssize_t ix, ssize_t iy, ssize_t iz) const {
    return mData[dataIndex(ix, iy, iz)];
  }
  inline double &at(ssize_t ix, ssize_t iy, ssize_t iz) {
    return mData[dataIndex(ix, iy, iz)];
  }
  inline double at(const Eigen::Vector3i &indices) const {
    return at(indices.x(), indices.y(), indices.z());
  }
  inline double &at(const Eigen::Vector3i &indices) {
    return at(indices.x(), indices.y(), indices.z());
  }
  inline Eigen::Vector3i index(const Eigen::Vector3d &position) const {
    Eigen::Vector3d pos = position - mCenter;
    pos *= 1.0 / mResolution;
    pos.array() += mSize * 0.5;
    pos.array().round();
    return pos.cast<int>();
  }
  inline Eigen::Vector3d position(const Eigen::Vector3i &indices) const {
    return ((indices.cast<double>().array() - mSize * 0.5) * mResolution)
               .matrix() +
           mCenter;
  }
  inline Eigen::Vector3d position(int ix, int iy, int iz) const {
    return position(Eigen::Vector3i(ix, iy, iz));
  }
  inline bool checkIndices(ssize_t ix, ssize_t iy, ssize_t iz) const {
    if (ix < 0 || iy < 0 || iz < 0)
      return false;
    if (ix >= mSize || iy >= mSize || iz >= mSize)
      return false;
    return true;
  }
  inline bool checkIndices(const Eigen::Vector3i &index) const {
    return checkIndices(index.x(), index.y(), index.z());
  }
  inline const Eigen::Vector3d &center() const { return mCenter; }
};

class Timer {
  const char *mName = nullptr;
  ros::WallTime mStartTime;

public:
  Timer(const char *name) : mName(name) { mStartTime = ros::WallTime::now(); }
  ~Timer() {
    auto stopTime = ros::WallTime::now();
    ROS_INFO_STREAM_THROTTLE(1.0, "time " << mName << " "
                                          << (stopTime - mStartTime));
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "tams_glass_reconstruct", 1);

  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  if (argc < 2) {
    ROS_ERROR_STREAM("usage: tams_glass reconstruct <data.bag>");
    return -1;
  }

  static const double gamma = 1.2;
  static const double huber_delta = 0.05;
  static const double smoothness = 0;
  static const double regularization = 10;
  static const double penalty = 10;
  static const int grid_size = 70;
  static const double grid_resolution = 0.005;
  static const double voxel_threshold = 0.015;
  static const double mesh_threshold = 0.015;
  static const double symmetry = 200;
  static const size_t ray_count = 1000;

  size_t iteration = 0;

  static const auto huberLoss = [](double x) {
    if (std::abs(x) <= huber_delta) {
      return 0.5 * x * x;
    } else {
      return huber_delta * (std::abs(x) - 0.5 * huber_delta);
    }
  };

  mkdir("./meshes", 0777);
  std::string output_prefix;
  if (auto *tok = strrchr(argv[1], '/')) {
    output_prefix = tok;
  } else {
    output_prefix = argv[1];
  }
  while (output_prefix.size() && output_prefix[0] == '/') {
    output_prefix = output_prefix.substr(1);
  }
  output_prefix = "./meshes/" + output_prefix;
  ROS_INFO_STREAM("output prefix " << output_prefix);

  static const auto computeWeight = [](double x) {
    if (x * x != 0.0) {
      return std::sqrt(huberLoss(x) / (x * x));
    } else {
      return 0.0;
    }
  };

  std::string tip_link = "ur5_tool0";
  std::string base_link = "table_top";

  moveit::planning_interface::MoveGroupInterface move_group("arm");

  moveit::core::RobotState robot_state = *move_group.getCurrentState();

  sensor_msgs::CameraInfo camera_info;
  std::string camera_name;
  if (!camera_calibration_parsers::readCalibrationYml(
          ros::package::getPath("tams_glass") + "/config/camera.yaml",
          camera_name, camera_info)) {
    ROS_ERROR_STREAM("failed to read camera");
    return -1;
  }

  image_geometry::PinholeCameraModel camera;
  if (!camera.fromCameraInfo(camera_info)) {
    ROS_ERROR_STREAM("failed to load camera info");
    return -1;
  }

  ros::Publisher vis_pub = node.advertise<visualization_msgs::MarkerArray>(
      "/tams_glass/visualization", 10);
  auto visualizeVoxelGrid = [&](const VoxelGrid &voxel_grid) {
    visualization_msgs::MarkerArray marker_array;

    std::string iteration_output_prefix = output_prefix + ".s" +
                                          std::to_string(symmetry) + ".i" +
                                          std::to_string(iteration);

    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CUBE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.frame_id = "/world";
      marker.ns = "transparent";
      marker.pose.orientation.w = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.0;
      marker.scale.x = voxel_grid.resolution();
      marker.scale.y = voxel_grid.resolution();
      marker.scale.z = voxel_grid.resolution();

      double lo = std::numeric_limits<double>::max();
      double hi = 0.00000001;
      for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
        for (size_t iy = 0; iy < voxel_grid.size(); iy++) {
          for (size_t ix = 0; ix < voxel_grid.size(); ix++) {
            hi = std::max(hi, voxel_grid.at(ix, iy, iz));
            lo = std::min(lo, voxel_grid.at(ix, iy, iz));
          }
        }
      }

      for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
        for (size_t iy = 0; iy < voxel_grid.size(); iy++) {
          for (size_t ix = 0; ix < voxel_grid.size(); ix++) {
            Eigen::Vector3d pos = voxel_grid.position(ix, iy, iz);

            double a = voxel_grid.at(ix, iy, iz);
            a = std::pow(a, gamma);

            if (!(a > 0.001)) {
              continue;
            }
            marker.points.emplace_back();
            marker.points.back().x = pos.x();
            marker.points.back().y = pos.y();
            marker.points.back().z = pos.z();
            marker.colors.emplace_back();
            marker.colors.back().r = 0;
            marker.colors.back().g = 0;
            marker.colors.back().b = 0;
            marker.colors.back().a = a * 3;
          }
        }
      }
      marker_array.markers.push_back(marker);
    }

    {
      std::ofstream stream(iteration_output_prefix + ".volume.bvox");
      auto writeInt = [&](uint32_t i) {
        stream.write((const char *)&i, sizeof(i));
      };
      auto writeFloat = [&](float i) {
        stream.write((const char *)&i, sizeof(i));
      };
      writeInt(grid_size);
      writeInt(grid_size);
      writeInt(grid_size);
      writeInt(1);
      for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
        for (size_t iy = 0; iy < voxel_grid.size(); iy++) {
          for (size_t ix = 0; ix < voxel_grid.size(); ix++) {
            writeFloat(voxel_grid.at(ix, iy, iz));
          }
        }
      }
    }

    {
      std::ofstream stream(iteration_output_prefix + ".volume.raw");
      double hi = 0.0000001;
      for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
        for (size_t iy = 0; iy < voxel_grid.size(); iy++) {
          for (size_t ix = 0; ix < voxel_grid.size(); ix++) {
            hi = std::max(hi, voxel_grid.at(ix, iy, iz));
          }
        }
      }
      for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
        for (size_t iy = 0; iy < voxel_grid.size(); iy++) {
          for (size_t ix = 0; ix < voxel_grid.size(); ix++) {
            uint8_t v = std::max(
                0.0, std::min(255.0, voxel_grid.at(ix, iy, iz) * 255.0 / hi));
            stream.write((char *)&v, 1);
          }
        }
      }
    }

    {
      std::ofstream stream(iteration_output_prefix + ".points.xyz");

      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CUBE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.frame_id = "/world";
      marker.ns = "voxels";
      marker.pose.orientation.w = 1.0;
      marker.color.r = 0.9;
      marker.color.g = 0.9;
      marker.color.b = 0.9;
      marker.color.a = 1.0;
      marker.scale.x = voxel_grid.resolution();
      marker.scale.y = voxel_grid.resolution();
      marker.scale.z = voxel_grid.resolution();
      for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
        for (size_t iy = 0; iy < voxel_grid.size(); iy++) {
          for (size_t ix = 0; ix < voxel_grid.size(); ix++) {
            Eigen::Vector3d pos = voxel_grid.position(ix, iy, iz);

            const int d = 1;
            std::array<double, (d + d + 1) * (d + d + 1) * (d + d + 1)>
                sample_buffer;
            size_t sample_count = 0;
            for (int dz = -d; dz <= d; dz++) {
              for (int dy = -d; dy <= d; dy++) {
                for (int dx = -d; dx <= d; dx++) {
                  Eigen::Vector3i p(ix + dx, iy + dy, iz + dz);
                  if (!voxel_grid.checkIndices(p)) {
                    continue;
                  }
                  sample_buffer[sample_count] = voxel_grid.at(p);
                  sample_count++;
                }
              }
            }
            std::sort(sample_buffer.begin(),
                      sample_buffer.begin() + sample_count);
            double a = sample_buffer[sample_count / 2 + 1];

            if (!(a > voxel_threshold)) {
              continue;
            }

            stream << pos.x() << " " << pos.y() << " " << pos.z() << "\n";

            marker.points.emplace_back();
            marker.points.back().x = pos.x();
            marker.points.back().y = pos.y();
            marker.points.back().z = pos.z();
          }
        }
      }
      marker_array.markers.push_back(marker);
    }

    {
      ROS_INFO_STREAM("meshing");

      pcl::PointCloud<pcl::PointNormal>::Ptr cloud(
          new pcl::PointCloud<pcl::PointNormal>);

      for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
        for (size_t iy = 0; iy < voxel_grid.size(); iy++) {
          for (size_t ix = 0; ix < voxel_grid.size(); ix++) {
            if (ix == 0 || iy == 0 || iz == 0 || ix + 1 == voxel_grid.size() ||
                iy + 1 == voxel_grid.size() || iz + 1 == voxel_grid.size()) {
              Eigen::Vector3d pos = voxel_grid.position(ix, iy, iz);
              pcl::PointNormal point;
              point.x = pos.x();
              point.y = pos.y();
              point.z = pos.z();
              point.normal_x = 0;
              point.normal_y = 0;
              point.normal_z = 1;
              cloud->push_back(point);
            }
          }
        }
      }

      ROS_INFO_STREAM(cloud->size() << " points");

      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.frame_id = "/world";
      marker.ns = "mesh";
      marker.pose.orientation.w = 1.0;
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 1;
      marker.color.a = 1.0;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      if (cloud->size() >= 3) {

        struct MatchingCubesVoxelGrid
            : public pcl::MarchingCubes<pcl::PointNormal> {
          const VoxelGrid &voxel_grid;
          MatchingCubesVoxelGrid(const VoxelGrid &voxel_grid)
              : voxel_grid(voxel_grid) {}
          void voxelizeData() override {
#pragma omp parallel for
            for (int x = 0; x < res_x_; x++) {
              for (int y = 0; y < res_y_; y++) {
                for (int z = 0; z < res_z_; z++) {
                  Eigen::Vector3d point;
                  point[0] = min_p_[0] + (max_p_[0] - min_p_[0]) * x / res_x_;
                  point[1] = min_p_[1] + (max_p_[1] - min_p_[1]) * y / res_y_;