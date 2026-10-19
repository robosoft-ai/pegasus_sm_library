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
#include <cl_px4_mr/client_behaviors/cb_hold_position.hpp>
#include <cl_px4_mr/client_behaviors/cb_obstacle_guard.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: something entered the camera's forward safety cone - stop and hover.
// Once it clears (or after kObstacleHoldMaxS) the leg is treated as aborted:
// escape back to the corridors and the scan point. Too many holds -> land here.
struct StObstacleHold : smacc2::SmaccState<StObstacleHold, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvObstacleCleared<CbObstacleGuard, OrCamera>, StReturnHome, SUCCESS>,
    Transition<EvCbSuccess<CbHoldPosition, OrPx4>, StReturnHome, ABORT>,
    Transition<EvCbFailure<CbHoldPosition, OrPx4>, StReturnHome, ABORT>,
    Transition<cl_generic_sensor::components::EvTopicMessageTimeout<ClDepthCamera, OrCamera>, MsLanding, ABORT>,
    Transition<EvLandHere, MsLanding, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbHoldPosition>(kObstacleHoldMaxS);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    auto & ms = this->context<MsInFlight>();
    ++ms.obstacleHolds;
    RCLCPP_WARN(
      getLogger(), "StObstacleHold: OBSTACLE in the safety cone - holding (hold %d/%d, up to %.0f s)",
      ms.obstacleHolds, kMaxObstacleHolds, static_cast<double>(kObstacleHoldMaxS));
    if (ms.obstacleHolds > kMaxObstacleHolds)
    {
      RCLCPP_ERROR(getLogger(), "StObstacleHold: stopped too often - landing here");
      this->postEvent<EvLandHere>();
    }
  }

  void onExit() {}
};

}  // namespace sm_pegasus_demo_1
