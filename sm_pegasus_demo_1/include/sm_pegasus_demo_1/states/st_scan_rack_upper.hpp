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

// STATE: rack scan, upper level - climb to the upper shelf height, turn to face the
// rack and fly the aisle sideways to the north end, camera head-on to the cartons
struct StScanRackUpper : smacc2::SmaccState<StScanRackUpper, SsRackScan>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbFollowNedPath, OrPx4>, StScanRackLower, SUCCESS>,
    Transition<EvCbFailure<CbFollowNedPath, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbFollowNedPath>(rackScanUpperNed(), rackScanFollower());
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrPx4, CbFollowNedPath>()->setTimeout(
      legTimeout(routeLengthM(rackScanUpperNed()) + 5.0f, kRackScanSpeedMps));
  }

  void onEntry()
  {
    setInventoryZone(this, "aisle 3 upper, head-on");
    RCLCPP_INFO(
      getLogger(), "StScanRackUpper: sideways north at %.1f m, facing east, %.1f m/s",
      static_cast<double>(kRackScanUpperAltM), static_cast<double>(kRackScanSpeedMps));
  }

  void onExit() {}
};

}  // namespace sm_pegasus_demo_1
