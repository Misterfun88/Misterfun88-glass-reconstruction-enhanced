
#include "common.h"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

static const double grid_resolution = 0.005;
static const double mesh_threshold = 0.002;
static const int grid_size = 70;
static const Eigen::Vector3d grid_center(0.918642, 0.704307, 0.708361);

struct MatchingCubesVoxelGrid : public pcl::MarchingCubes<pcl::PointNormal> {
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
          point[2] = min_p_[2] + (max_p_[2] - min_p_[2]) * z / res_z_;
          Eigen::Vector3i index = voxel_grid.index(point);
          if (voxel_grid.checkIndices(index)) {

            grid_[x * res_y_ * res_z_ + y * res_z_ + z] =
                (0.5 - voxel_grid.at(index));

          } else {
            grid_[x * res_y_ * res_z_ + y * res_z_ + z] = 1.0;
          }
        }
      }
    }
  }
};

void medianFilter(VoxelGrid &out) {
  VoxelGrid in = out;

  for (int ix = 0; ix < in.size(); ix++) {
    for (int iy = 0; iy < in.size(); iy++) {
      for (int iz = 0; iz < in.size(); iz++) {

        Eigen::Vector3i index(ix, iy, iz);

        const int d = 2;
        std::array<double, (d + d + 1) * (d + d + 1) * (d + d + 1)>
            sample_buffer;
        for (auto &v : sample_buffer) {
          v = 0.0;
        }
        size_t sample_count = 0;
        for (int dz = -d; dz <= d; dz++) {
          for (int dy = -d; dy <= d; dy++) {
            for (int dx = -d; dx <= d; dx++) {
              Eigen::Vector3i p(ix + dx, iy + dy, iz + dz);
              if (!in.checkIndices(p)) {
                continue;
              }
              sample_buffer[sample_count] = in.at(p);
              sample_count++;
            }
          }