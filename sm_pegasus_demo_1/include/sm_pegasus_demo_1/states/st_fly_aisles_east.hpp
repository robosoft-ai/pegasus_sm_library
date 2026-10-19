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

#include <cl_px4_mr/client_behaviors/cb_follow_ned_path.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: aisle leg 3 - aisles 5 and 6 (the forklift) and back to the scan point
struct StFlyAislesEast : smacc2::SmaccState<StFlyAislesEast, SsWarehouseInspection>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbFollowNedPath, OrPx4>, StScanShelves, SUCCESS>,
    Transition<EvCbFailure<CbFollowNedPath, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbFollowNedPath>(aisleLegEastNed(), cruiseFollower());
  }

  void runtimeConfigure()
  {
    const std::vector<NedPoint> route = aisleLegEastNed();
    this->getClientBehavior<OrPx4, CbFollowNedPath>()->setTimeout(
      legTimeout(routeLengthM(route) + 10.0f, kCruiseSpeedMps));
    RCLCPP_INFO(
      getLogger(), "StFlyAislesEast: %zu vertices, %.0f m, ending at NED (%.1f, %.1f, %.1f)", route.size(),
      static_cast<double>(routeLengthM(route)), static_cast<double>(route.back().x),
      static_cast<double>(route.back().y), static_cast<double>(route.back().z));
  }

  void onEntry() { setInventoryZone(this, "aisles 5-6"); }

  void onExit() { recordTraversed(this, "StFlyAislesEast"); }
};

}  // namespace sm_pegasus_demo_1
