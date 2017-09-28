#include <iostream>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

// TODO(eric.cousineau): Figure out how to make Bazel permit angle brackets.
#include "drake/systems/primitives/random_source.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef drake::systems::RandomState<std::uniform_real_distribution<double>>
    RandomStateT;

int main (int argc, char** argv) {
  PointCloudT::Ptr cloud(new PointCloudT());
  PointCloudT::Ptr cloud_filtered(new PointCloudT());

  const int num_points = 1000;
  const float radius = 1;

  RandomStateT rand;
  auto rand_pt = [&rand, radius](float pt[3]) {
    for (int i = 0; i < 3; ++i) {
      pt[i] = (rand.GetNextValue() * 2 - 1) * radius;
    }
  };

  cloud->resize(num_points);
  for (int i = 0; i < num_points; ++i) {
    auto& pt = cloud->points[i];
    rand_pt(pt.data);
  }

  std::cout << "PointCloud before filtering: " << cloud->size() << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(0.01f, 0.01f, 0.01f);
  filter.filter(*cloud_filtered);

  std::cout << "PointCloud after filtering: " << cloud->size() << std::endl;

  return 0;
}
