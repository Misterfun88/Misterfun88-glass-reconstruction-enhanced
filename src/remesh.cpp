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

  std::cout << "grid size " << voxel_header.size_x << " " << voxel_header.size_y
            << " " << voxel_header.size_z << std::endl;

  if (voxel_header.size_x != voxel_header.size_y ||
      voxel_header.size_y != voxel_header.size_z) {
    throw std::runtime_error("currently only voxel cubes are supported");
  }

  VoxelGrid voxel_grid(grid_center, voxel_header.size_x, grid_resolution);

  for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
    for (size_t iy = 0; iy < voxel_grid.size(); iy++) {
      for (size_t ix = 0; ix < voxel_grid.size(); ix++) {
        float v = 0.0f;
        voxel_file.read((char *)&v, sizeof(v));
        voxel_grid.at(ix, iy, iz) = v;
      }
    }
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointNormal>);
  for (size_t iz = 0; iz < voxel_grid.size(); iz++) {
 