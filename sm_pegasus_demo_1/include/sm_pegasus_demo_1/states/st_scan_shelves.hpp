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

#include <cl_px4_mr/client_behaviors/cb_yaw_scan.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace cl_ros2_timer;
using namespace smacc2::default_transition_tags;

// STATE: camera pan. Hold position on the south corridor in front of the pad and
// sweep the heading about north, so the camera pans across the rack ends and
// into the aisles. CbYawScan never finishes by itself: the timer orthogonal ends
// the state. The aisles are done, so from here an abort flies straight home over
// the open floor.
struct StScanShelves : smacc2::SmaccState<StScanShelves, SsWarehouseInspection>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StFlyToOverview, SUCCESS>,
    // no valid position to hold: skip the pan
    Transition<EvCbFailure<CbYawScan, OrPx4>, StFlyToOverview, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbYawScan>(kScanAmplitudeRad, kScanPeriodS, kScanBaseHeadingRad);
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(rclcpp::Duration(kScanDuration));
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    setInventoryZone(this, "rack ends pan");
    auto & ms = this->context<MsInFlight>();
    ms.traversed.clear();
    RCLCPP_INFO(
      getLogger(), "StScanShelves: panning +-%.0f deg about north for %ld s",
      static_cast<double>(kScanAmplitudeRad * 180.0f / static_cast<float>(M_PI)),
      static_cast<long>(kScanDuration.count()));
  }

  void onExit() {}
};

}  // namespace sm_pegasus_demo_1
