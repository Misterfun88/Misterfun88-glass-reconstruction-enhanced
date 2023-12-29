#include "common.h"

#include <pcl/point_types.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

int main(int argc, char **argv) {

  static const double grid_resolution = 0.005;
  static const double mesh_threshold = 0.02;

  Eigen::Vector3d grid_center(0.918642, 0.704307, 0.708361);

  ros::init(argc, argv, "tams_glass_remesh", 0);

  std::ifstream voxel_file(argv[1]);

  if (!voxel_file) {
    throw std::runtime_error("failed to open voxel file");
  }

  struct {
    uint32_t size_x = 0, size_y = 1, size_z = 2, reserved = 0;
  } voxel_header;

  voxel_file.read((char *)&voxel_header, sizeof(voxel_header));

  std::cout << "grid size " << v