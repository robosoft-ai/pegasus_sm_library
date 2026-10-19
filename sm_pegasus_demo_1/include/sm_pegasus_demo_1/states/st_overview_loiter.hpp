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

#include <cl_px4_mr/client_behaviors/cb_loiter.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: finale, part 1 - loiter laps over the open floor at the overview altitude,
// entered at the nearest point of the circle, camera along the path
struct StOverviewLoiter : smacc2::SmaccState<StOverviewLoiter, SsWarehouseInspection>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbLoiter, OrPx4>, StFigureEight, SUCCESS>,
    Transition<EvCbFailure<CbLoiter, OrPx4>, StReturnToPad, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbLoiter>(overviewLoiterParams(), overviewFollower());
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrPx4, CbLoiter>()->setTimeout(
      legTimeout(overviewOrbitLengthM() + 10.0f, kOverviewSpeedMps));
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StOverviewLoiter: %d lap(s), radius %.0f m, %.1f m up, camera along the path",
      kOverviewOrbits, static_cast<double>(kOverviewRadiusM), static_cast<double>(kOverviewAltitudeM));
  }

  void onExit() {}
};

}  // namespace sm_pegasus_demo_1
