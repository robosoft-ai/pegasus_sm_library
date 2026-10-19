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
#include <cl_px4_mr/client_behaviors/cb_obstacle_guard.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: abort leg - escape from wherever the aisle route was interrupted: back
// over the traversed vertices to the nearest corridor, along it and down the
// escape aisle if needed, to the scan point (a straight line home would cross the
// racks). The demo then carries on with the camera pan and the overview. An
// obstacle on the way stops the vehicle again.
struct StReturnHome : smacc2::SmaccState<StReturnHome, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbFollowNedPath, OrPx4>, StScanShelves, SUCCESS>,
    Transition<EvCbFailure<CbFollowNedPath, OrPx4>, MsLanding, ABORT>,
    Transition<EvObstacleTooClose<CbObstacleGuard, OrCamera>, StObstacleHold, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbFollowNedPath>(std::vector<NedPoint>{}, cruiseFollower());
  }

  void runtimeConfigure()
  {
    auto & ms = this->context<MsInFlight>();
    auto * cb = this->getClientBehavior<OrPx4, CbFollowNedPath>();
    const std::vector<NedPoint> path = escapePath(ms.traversed, ms.homeAltitudeM);
    cb->setPath(path);
    cb->setTimeout(legTimeout(routeLengthM(path) + 30.0f, kCruiseSpeedMps));
    RCLCPP_WARN(
      getLogger(), "StReturnHome: AISLE LEG ABORTED - escaping over %zu vertices (%.0f m) to the scan point at %.1f m",
      path.size(), static_cast<double>(routeLengthM(path)), static_cast<double>(ms.homeAltitudeM));
  }

  void onEntry() {}

  void onExit()
  {
    // whatever was flown back is no longer ahead of the vehicle
    this->context<MsInFlight>().traversed.clear();
  }
};

}  // namespace sm_pegasus_demo_1
