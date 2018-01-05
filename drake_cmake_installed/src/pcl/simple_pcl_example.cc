/*****************************************************************************
 * Copyright (c) 2017, Toyota Research Institute.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the Massachusetts Institute of Technology nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE MASSACHUSETTS INSTITUTE OF TECHNOLOGY AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE MASSACHUSETTS
 * INSTITUTE OF TECHNOLOGY OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/**
 * @file apps/simple_pcl_test.cc
 *
 * Simple PCL Example
 *
 * This is meant to test building / linking / executing with PCL.
 * This example just builds a random point cloud and downsamples it, printing
 * the size before and after.
 */

#include <iostream>
// #include <random>

// #include <drake/systems/primitives/random_source.h>
// #include <drake/common/autodiff.h>

// // Test Eigen header presence (fails if Drake's Eigen w/ additional Autodiff support is not utilised)
// #include <drake/solvers/mathematical_program.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#include <memory>
#include <string>

// #include <gflags/gflags.h>
#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/unused.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/rgbd_camera.h"

using std::cout;
using std::endl;
using std::string;

using drake::multibody::joints::kQuaternion;
using drake::multibody::joints::kFixed;

namespace drake {
namespace systems {
namespace sensors {
namespace {

bool ValidateSdf(const char* flagname, const std::string& filename) {
  if (filename.substr(filename.find_last_of(".") + 1) == "sdf") {
    return true;
  }
  cout << "Invalid filename for --" << flagname << ": " << filename << endl;
  return false;
}

bool ValidateDir(const char* flagname, const std::string& dir) {
  if (dir.empty()) {
    cout << "Invalid directory for --" << flagname << ": " << dir << endl;
    return false;
  }
  return true;
}

#define DEFINE_bool(name, value, ...) bool FLAGS_##name = value
#define DEFINE_double(name, value, ...) double FLAGS_##name = value
#define DEFINE_string(name, value, ...) std::string FLAGS_##name = value

DEFINE_bool(lookup, true,
            "If true, RgbdCamera faces a direction normal to the "
            "terrain plane.");
DEFINE_bool(show_window, true,
            "If true, RgbdCamera opens windows for displaying rendering "
            "context.");
DEFINE_double(duration, 5., "Total duration of the simulation in secondes.");
DEFINE_string(sdf_dir, "/home/eacousineau/proj/tri/repo/branches/drake-distro/master/systems/sensors/models",
              "The full path of directory where SDFs are located.");
DEFINE_string(sdf_fixed, "sphere.sdf",
              "The filename for a SDF that contains fixed base objects.");
DEFINE_string(sdf_floating, "box.sdf",
              "The filename for a SDF that contains floating base objects.");

constexpr double kCameraUpdatePeriod{0.01};

constexpr char kCameraBaseFrameName[] = "camera_base_frame";
constexpr char kColorCameraFrameName[] = "color_camera_optical_frame";
constexpr char kDepthCameraFrameName[] = "depth_camera_optical_frame";
constexpr char kLabelCameraFrameName[] = "label_camera_optical_frame";

constexpr char kImageArrayLcmChannelName[] = "DRAKE_RGBD_CAMERA_IMAGES";
constexpr char kPoseLcmChannelName[] = "DRAKE_RGBD_CAMERA_POSE";

struct CameraConfig {
  Eigen::Vector3d pos;
  Eigen::Vector3d rpy;
  double fov_y{};
  double depth_range_near{};
  double depth_range_far{};
};

}  // anonymous namespace

int main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FLAGS_sdf_dir + "/" + FLAGS_sdf_fixed, kFixed, tree.get());

  drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FLAGS_sdf_dir + "/" + FLAGS_sdf_floating, kQuaternion, tree.get());

  drake::multibody::AddFlatTerrainToWorld(tree.get());

  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem<RigidBodyPlant<double>>(move(tree));
  plant->set_name("rigid_body_plant");

  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(1e8)  // Pa
      .set_dissipation(1)  // s/m
      .set_friction(0.9, 0.5);
  plant->set_default_compliant_material(default_material);

  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = 2e-4;  // m
  model_parameters.v_stiction_tolerance = 0.01;  // m/s
  plant->set_contact_model_parameters(model_parameters);

  // Adds an RgbdCamera at a fixed pose.
  CameraConfig config;
  if (FLAGS_lookup) {
    config.pos = Eigen::Vector3d(0., -0.02, 0.05);
    config.rpy = Eigen::Vector3d(0., -M_PI_2, 0.);
    config.fov_y = 130. / 180 * M_PI;
    config.depth_range_near = 0.01;
    config.depth_range_far = 1.;
  } else {
    config.pos = Eigen::Vector3d(-1., 0., 1.);
    config.rpy = Eigen::Vector3d(0., M_PI_4, 0.);
    config.fov_y = M_PI_4;
    config.depth_range_near = 0.5;
    config.depth_range_far = 5.;
  }

  auto rgbd_camera =
      builder.AddSystem<RgbdCameraDiscrete>(
          std::make_unique<RgbdCamera>(
              "rgbd_camera", plant->get_rigid_body_tree(),
              config.pos, config.rpy,
              config.depth_range_near, config.depth_range_far,
              config.fov_y, FLAGS_show_window),
      kCameraUpdatePeriod);

  auto image_to_lcm_image_array =
      builder.template AddSystem<ImageToLcmImageArrayT>(
          kColorCameraFrameName, kDepthCameraFrameName, kLabelCameraFrameName);
  image_to_lcm_image_array->set_name("converter");

  ::drake::lcm::DrakeLcm lcm;
  auto drake_viz = builder.template AddSystem<DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm);
  drake_viz->set_publish_period(kCameraUpdatePeriod);

  auto image_array_lcm_publisher = builder.template AddSystem(
      lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          kImageArrayLcmChannelName, &lcm));
  image_array_lcm_publisher->set_name("publisher");
  image_array_lcm_publisher->set_publish_period(kCameraUpdatePeriod);

  rendering::PoseStampedTPoseVectorTranslator translator(kCameraBaseFrameName);
  auto pose_lcm_publisher = builder.template AddSystem<lcm::LcmPublisherSystem>(
      kPoseLcmChannelName, translator, &lcm);
  pose_lcm_publisher->set_name("pose_lcm_publisher");
  pose_lcm_publisher->set_publish_period(kCameraUpdatePeriod);

  builder.Connect(
      plant->get_output_port(0),
      rgbd_camera->state_input_port());

  builder.Connect(
      plant->get_output_port(0),
      drake_viz->get_input_port(0));

  builder.Connect(
      rgbd_camera->color_image_output_port(),
      image_to_lcm_image_array->color_image_input_port());

  builder.Connect(
      rgbd_camera->depth_image_output_port(),
      image_to_lcm_image_array->depth_image_input_port());

  builder.Connect(
      rgbd_camera->label_image_output_port(),
      image_to_lcm_image_array->label_image_input_port());

  builder.Connect(
      image_to_lcm_image_array->image_array_t_msg_output_port(),
      image_array_lcm_publisher->get_input_port(0));

  builder.Connect(
      rgbd_camera->camera_base_pose_output_port(),
      pose_lcm_publisher->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator = std::make_unique<systems::Simulator<double>>(
      *diagram, std::move(context));

  simulator->set_publish_at_initialization(true);
  simulator->set_publish_every_time_step(false);
  simulator->Initialize();
  simulator->StepTo(FLAGS_duration);

  return 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

void test_pcl() {
  PointCloudT::Ptr cloud(new PointCloudT());
  PointCloudT::Ptr cloud_filtered(new PointCloudT());

  const int num_points = 100000;
  const float radius = 1;

  cloud->resize(num_points);
  for (int i = 0; i < num_points; ++i) {
    cloud->points[i] = PointT(i, i, i);
  }

  std::cout << "PointCloud before filtering: " << cloud->size() << std::endl;

  // Create the filtering object.
  pcl::VoxelGrid<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(0.01f, 0.01f, 0.01f);
  filter.filter(*cloud_filtered);

  std::cout << "PointCloud after filtering: "
      << cloud_filtered->size() << std::endl;
}

int main() {
  test_pcl();
  return drake::systems::sensors::main();
}
