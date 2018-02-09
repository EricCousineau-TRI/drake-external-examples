/*****************************************************************************
 * Copyright (c) 2018, Toyota Research Institute.
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
 * @file
 *
 * Test a Drake camera simulation that has been linked with PCL.
 */

#include <iostream>
#include <memory>

// Test Eigen header presence (fails if Drake's Eigen w/ additional Autodiff
// support is not utilised).
#include <drake/common/autodiff.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/rigid_body_tree_construction.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/zero_order_hold.h>
#include <drake/systems/sensors/image.h>
#include <drake/systems/sensors/rgbd_camera.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using drake::multibody::AddFlatTerrainToWorld;
using drake::systems::DiagramBuilder;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using drake::systems::ZeroOrderHold;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::RgbdCameraDiscrete;
using drake::systems::sensors::RgbdCamera;

namespace shambhala {

int main() {
  // Creates a simple discrete RGB-D camera simulation.
  DiagramBuilder<double> builder;

  // Create RigidBodyTree that shows flat ground.
  auto tree = std::make_unique<RigidBodyTree<double>>();
  AddFlatTerrainToWorld(tree.get());
  auto plant = builder.AddSystem<RigidBodyPlant<double>>(move(tree));

  const double dt = 0.03;

  // Add RGB-D camera simulation.
  const RgbdCameraDiscrete* rgbd_camera =
      builder.AddSystem<RgbdCameraDiscrete>(
          std::make_unique<RgbdCamera>(
              "rgbd_camera", plant->get_rigid_body_tree(),
              Eigen::Vector3d(-1., 0., 1.),
              Eigen::Vector3d(0., M_PI_4, 0.),
              0.5, 5.,
              M_PI_4, false),
      dt);
  // Connect to RigidBodyTree scene.
  builder.Connect(
      plant->get_output_port(0),
      rgbd_camera->state_input_port());
  // Add ZOH to easily expose depth image as state.
  const Value<ImageDepth32F> depth_image_model(width, height);
  const auto* const zoh_depth =
      builder.AddSystem<ZeroOrderHold>(dt, depth_image_model);
  // Connect depth image to zero-order hold.
  build.Connect(
      rgbd_camera->depth_image_output_port(),
      zoh_depth->get_input_port());

  // Build diagram, and run simulation for 0.1 sec.
  auto diagram = builder.Build();
  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
      *diagram);
  simulator->StepTo(0.1);

  // TODO(eric.cousineau): Add in example of converting depth image to PCL
  // point cloud.
  const ImageDepth32F& depth_image =
      diagram->GetSubsystemContext(simulator->get_context(), *zoh_depth)
             ->get_abstract_state<ImageDepth32F>(0);

  Eigen::Matrix3Xf point_cloud_eigen;
  RgbdCamera::ConvertDepthImageToPointCloud(
      depth_image, rgbd_camera->depth_camera_info(), &point_cloud);

  PointCloudT::Ptr cloud(new PointCloudT());  
  cloud->resize(num_points);
  for (int i = 0; i < num_points; ++i) {
    auto& pt = cloud->points[i];
    rand_pt(pt.data);
  }



  std::cout << "Finished camera simulation." << std::endl;
  return 0;
}

}  // namespace shambhala

int main() {
  return shambhala::main();
}
