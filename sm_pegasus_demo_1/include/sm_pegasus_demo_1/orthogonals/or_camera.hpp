// Copyright 2026 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cl_generic_sensor/cl_generic_sensor.hpp>
#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>
#include <config/mission_constants.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

// The depth camera, as a point cloud: depth_image_proc turns the Pegasus depth
// image into a PointCloud2 (camera optical frame) that this orthogonal watches
// with a message watchdog (CpMessageTimeout -> EvTopicMessageTimeout<ClDepthCamera,
// OrCamera>) and the forward safety cone (CpForwardObstacleGuard -> CbObstacleGuard
// events).
class OrCamera : public smacc2::Orthogonal<OrCamera>
{
public:
  void onInitialize() override
  {
    using ClDepthCamera = cl_generic_sensor::ClGenericSensor<sensor_msgs::msg::PointCloud2>;
    auto client = this->createClient<ClDepthCamera>(
      std::string(kDepthCloudTopic), rclcpp::Duration(kDepthCloudTimeout));
    client->createComponent<cl_px4_mr::CpForwardObstacleGuard>(coneParams());
  }
};

}  // namespace sm_pegasus_demo_1
