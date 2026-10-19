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

#include <cl_px4_mr/client_behaviors/cb_lemniscate.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_pegasus_demo_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: finale, part 2 - figure eights about the overview centre; the lobe axis
// points at the loiter's entry point, so the vehicle starts at a lobe tip right
// where the loiter left it, camera along the path
struct StFigureEight : smacc2::SmaccState<StFigureEight, SsWarehouseInspection>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbLemniscate, OrPx4>, StReturnToPad, SUCCESS>,
    Transition<EvCbFailure<CbLemniscate, OrPx4>, StReturnToPad, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbLemniscate>(figureEightParams(), overviewFollower());
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrPx4, CbLemniscate>()->setTimeout(
      legTimeout(flightPatternFigureEightLength(figureEightParams()) + 10.0f, kOverviewSpeedMps));
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StFigureEight: %d loop(s), lobes %.0f m from the centre", kEightLoops,
      static_cast<double>(kEightSizeM));
  }

  void onExit() {}
};

}  // namespace sm_pegasus_demo_1
