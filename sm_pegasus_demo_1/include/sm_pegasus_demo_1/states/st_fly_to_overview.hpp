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

#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: climb out of the aisle band and fly to the entry point of the overview
// circle over the open floor
struct StFlyToOverview : smacc2::SmaccState<StFlyToOverview, SsWarehouseInspection>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, StOverviewLoiter, SUCCESS>,
    Transition<EvCbFailure<CbGoToLocation, OrPx4>, StReturnToPad, ABORT>
  > reactions;

  static void staticConfigure()
  {
    const NedPoint p = overviewEntryNed();
    configure_orthogonal<OrPx4, CbGoToLocation>(p.x, p.y, p.z);
  }

  void runtimeConfigure()
  {
    auto & ms = this->context<MsInFlight>();
    ms.homeAltitudeM = kOverviewAltitudeM;  // above everything on the open floor from now on
    const NedPoint p = overviewEntryNed();
    const NedPoint s = scanPointNed(kCruiseAltitudeM);
    this->getClientBehavior<OrPx4, CbGoToLocation>()->setTimeout(
      legTimeout(cl_px4_mr::nedDistance(s, p) + 10.0f, kClimbRateMps));
  }

  void onEntry()
  {
    setInventoryZone(this, "overview");
    const NedPoint p = overviewEntryNed();
    RCLCPP_INFO(
      getLogger(), "StFlyToOverview: to the orbit entry NED (%.1f, %.1f, %.1f)", static_cast<double>(p.x),
      static_cast<double>(p.y), static_cast<double>(p.z));
  }

  void onExit() {}
};

}  // namespace sm_pegasus_demo_1
