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

#include <cl_px4_mr/client_behaviors/cb_ascend_to_altitude.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: climb from the takeoff altitude to the aisle cruise altitude
struct StAscend : smacc2::SmaccState<StAscend, SsWarehouseInspection>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbAscendToAltitude, OrPx4>, StFlyAislesWest, SUCCESS>,
    Transition<EvCbFailure<CbAscendToAltitude, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbAscendToAltitude>(kCruiseAltitudeM, kClimbRateMps);
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrPx4, CbAscendToAltitude>()->setTimeout(
      legTimeout(kCruiseAltitudeM - kTakeoffAltitudeM, kClimbRateMps));
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StAscend: climbing to %.1f m", static_cast<double>(kCruiseAltitudeM));
  }

  void onExit() {}
};

}  // namespace sm_pegasus_demo_1
