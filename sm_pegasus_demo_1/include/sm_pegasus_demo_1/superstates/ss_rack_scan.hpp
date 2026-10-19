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
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// SUPERSTATE: the sideways rack scan in aisle 3 - camera on the rack face 1.9 m
// away, which is exactly what the forward safety cone would call an obstacle, so
// unlike SsWarehouseInspection this superstate does NOT react to the guard. A
// dead depth cloud still aborts (StReturnHome escapes over the leg-1 vertices,
// whose last one is the south end of this aisle).
struct SsRackScan : smacc2::SmaccState<SsRackScan, MsInFlight, StScanRackUpper>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<cl_generic_sensor::components::EvTopicMessageTimeout<ClDepthCamera, OrCamera>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "=== SsRackScan: aisle %d sideways, upper level %.1f m north then lower level %.1f m south ===",
      kRackScanAisle + 1, static_cast<double>(kRackScanUpperAltM), static_cast<double>(kRackScanLowerAltM));
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "=== Exiting SsRackScan ===");
  }
};

}  // namespace sm_pegasus_demo_1
