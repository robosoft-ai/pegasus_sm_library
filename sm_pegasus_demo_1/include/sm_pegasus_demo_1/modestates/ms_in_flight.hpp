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

#include <algorithm>
#include <vector>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;

// MODE STATE: airborne. Hosts the one CbObstacleGuard for the whole flight
// (container-state behavior: one instance, one event per detection, survives
// every inner transition) and the data the abort leg shares.
//
// The nominal legs live in SsWarehouseInspection; StObstacleHold and
// StReturnHome are its siblings here. Reactions must NOT be declared on this
// state with an inner target: Boost.Statechart transitions are external, so
// that would exit and re-enter MsInFlight, destroying the guard and this data.
struct MsInFlight : smacc2::SmaccState<MsInFlight, SmPegasusDemo1, SsWarehouseInspection>
{
  using SmaccState::SmaccState;

  // aisle-route vertices the vehicle has actually flown past (cruise altitude);
  // an abort escapes back over them to the nearest corridor. Written by
  // StFlyAisles, consumed by StReturnHome, cleared once the aisles are done.
  std::vector<NedPoint> traversed;
  // altitude an abort flies at: cruise in the aisles, the overview altitude
  // once the vehicle is above the open floor
  float homeAltitudeM = kCruiseAltitudeM;
  int obstacleHolds = 0;

  typedef mpl::list<
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrCamera, CbObstacleGuard>();
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsInFlight ---");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsInFlight ---");
  }
};

// Shared onExit of every aisle leg: remember the route vertices actually passed
// (StReturnHome escapes back over them). Called from the state machine thread.
template <typename TState>
inline void recordTraversed(TState * state, const char * name)
{
  auto & ms = state->template context<MsInFlight>();
  auto * cb = state->template getClientBehavior<OrPx4, CbFollowNedPath>();
  const auto & route = cb->path();
  const size_t n = std::min(cb->reachedCount(), route.size());
  ms.traversed.assign(route.begin(), route.begin() + n);
  RCLCPP_INFO(state->getLogger(), "%s: reached %zu/%zu route vertices", name, n, route.size());
}

}  // namespace sm_pegasus_demo_1
