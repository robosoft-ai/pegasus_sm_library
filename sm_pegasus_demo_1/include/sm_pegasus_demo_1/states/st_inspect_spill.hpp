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

// STATE: look at the spill - hold low in front of it and pan slowly across the
// signs, cones, bucket and carton; the timer orthogonal ends the look
struct StInspectSpill : smacc2::SmaccState<StInspectSpill, SsWarehouseInspection>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StClimbFromSpill, SUCCESS>,
    Transition<EvCbFailure<CbYawScan, OrPx4>, StClimbFromSpill, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbYawScan>(kSpillLookAmplitudeRad, kSpillLookPeriodS, kSpillLookBaseHeadingRad);
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(rclcpp::Duration(kSpillLookDuration));
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StInspectSpill: investigating - panning +-%.0f deg for %ld s",
      static_cast<double>(kSpillLookAmplitudeRad * 180.0f / static_cast<float>(M_PI)),
      static_cast<long>(kSpillLookDuration.count()));
  }

  void onExit() {}
};

}  // namespace sm_pegasus_demo_1
