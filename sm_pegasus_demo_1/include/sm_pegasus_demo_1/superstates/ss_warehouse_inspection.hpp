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

#include <cl_generic_sensor/components/cp_message_timeout.hpp>
#include <cl_px4_mr/client_behaviors/cb_obstacle_guard.hpp>
#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// SUPERSTATE: the nominal flight - ascend, fly the aisles, pan the camera, fly to
// the overview circle, orbit, return, descend. Its reactions catch the camera
// gating events from every leg below it: an obstacle in the safety cone stops the
// vehicle (StObstacleHold), a dead depth cloud aborts to the scan point
// (StReturnHome). Both targets are siblings of this superstate inside MsInFlight,
// so the guard behavior and the shared escape data survive the transition.
struct SsWarehouseInspection : smacc2::SmaccState<SsWarehouseInspection, MsInFlight, StAscend>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvObstacleTooClose<CbObstacleGuard, OrCamera>, StObstacleHold, ABORT>,
    Transition<cl_generic_sensor::components::EvTopicMessageTimeout<ClDepthCamera, OrCamera>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "=== SsWarehouseInspection: %d aisles, route %zu vertices %.0f m at %.1f m ===",
      kAisleCount, aisleRouteNed().size(), static_cast<double>(routeLengthM(aisleRouteNed())),
      static_cast<double>(kCruiseAltitudeM));
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "=== Exiting SsWarehouseInspection ===");
  }
};

}  // namespace sm_pegasus_demo_1
