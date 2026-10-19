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

#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_generic_sensor/cl_generic_sensor.hpp>
#include <cl_px4_mr/cl_px4_mr.hpp>
#include <cl_ros2_timer/cl_ros2_timer.hpp>

// COMPONENTS (beyond the ClPx4Mr defaults)
#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>
#include <cl_px4_mr/components/cp_tf_broadcaster.hpp>

// CLIENT BEHAVIORS
#include <cl_px4_mr/client_behaviors/cb_arm_px4.hpp>
#include <cl_px4_mr/client_behaviors/cb_ascend_to_altitude.hpp>
#include <cl_px4_mr/client_behaviors/cb_connect_micro_ros_agent.hpp>
#include <cl_px4_mr/client_behaviors/cb_disarm_px4.hpp>
#include <cl_px4_mr/client_behaviors/cb_follow_ned_path.hpp>
#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <cl_px4_mr/client_behaviors/cb_hold_position.hpp>
#include <cl_px4_mr/client_behaviors/cb_land.hpp>
#include <cl_px4_mr/client_behaviors/cb_obstacle_guard.hpp>
#include <cl_px4_mr/client_behaviors/cb_lemniscate.hpp>
#include <cl_px4_mr/client_behaviors/cb_loiter.hpp>
#include <cl_px4_mr/client_behaviors/cb_takeoff.hpp>
#include <cl_px4_mr/client_behaviors/cb_wait_for_heading_stable.hpp>
#include <cl_px4_mr/client_behaviors/cb_yaw_scan.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <smacc2/client_behaviors/cb_sleep_for.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

// CONFIG (mission constants and the helpers derived from them)
#include <config/mission_constants.hpp>

using namespace boost;
using namespace smacc2;

namespace sm_pegasus_demo_1
{

// the depth camera client: a generic PointCloud2 sensor with a message watchdog
using ClDepthCamera = cl_generic_sensor::ClGenericSensor<sensor_msgs::msg::PointCloud2>;

// posted by StObstacleHold when the vehicle has been stopped too often
struct EvLandHere : sc::event<EvLandHere>
{
};

}  // namespace sm_pegasus_demo_1

// ORTHOGONALS
#include <sm_pegasus_demo_1/orthogonals/or_camera.hpp>
#include <sm_pegasus_demo_1/orthogonals/or_px4.hpp>
#include <sm_pegasus_demo_1/orthogonals/or_timer.hpp>
#include <sm_pegasus_demo_1/orthogonals/or_tracker.hpp>

namespace sm_pegasus_demo_1
{

// MODE STATES (forward declarations)
class MsDisarmedOnGround;
class MsArmedOnGround;
class MsTakeoff;
class MsInFlight;
class MsLanding;
class MsLanded;

// SUPERSTATES (forward declarations)
class SsWarehouseInspection;
class SsRackScan;

// STATES (forward declarations), in mission order
class StPause;
class StConnectMicroROSAgent;
class StWaitForReady;
class StArmPX4;
class StPrepareForTakeoff;
class StTakeoff;
class StAscend;
class StFlyAislesWest;
class StScanRackUpper;
class StScanRackLower;
class StFlyAislesMid;
class StDescendToSpill;
class StInspectSpill;
class StClimbFromSpill;
class StFlyAislesEast;
class StScanShelves;
class StFlyToOverview;
class StOverviewLoiter;
class StFigureEight;
class StReturnToPad;
class StPreLandDescent;
class StObstacleHold;
class StReturnHome;
class StLand;
class StLanded;

// STATE MACHINE
struct SmPegasusDemo1 : public smacc2::SmaccStateMachineBase<SmPegasusDemo1, MsDisarmedOnGround>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrPx4>();
    this->createOrthogonal<OrCamera>();
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrTracker>();
  }
};

}  // namespace sm_pegasus_demo_1

// MODE STATE INCLUDES (after the SM definition, before regular states)
#include <sm_pegasus_demo_1/modestates/ms_disarmed_on_ground.hpp>
#include <sm_pegasus_demo_1/modestates/ms_armed_on_ground.hpp>
#include <sm_pegasus_demo_1/modestates/ms_takeoff.hpp>
#include <sm_pegasus_demo_1/modestates/ms_in_flight.hpp>
#include <sm_pegasus_demo_1/modestates/ms_landing.hpp>
#include <sm_pegasus_demo_1/modestates/ms_landed.hpp>

// SUPERSTATE INCLUDES (the nominal flight legs live inside it)
#include <sm_pegasus_demo_1/superstates/ss_warehouse_inspection.hpp>
#include <sm_pegasus_demo_1/superstates/ss_rack_scan.hpp>

// REGULAR STATE INCLUDES, in mission order
#include <sm_pegasus_demo_1/states/st_pause.hpp>
#include <sm_pegasus_demo_1/states/st_connect_micro_ros_agent.hpp>
#include <sm_pegasus_demo_1/states/st_wait_for_ready.hpp>
#include <sm_pegasus_demo_1/states/st_arm_px4.hpp>
#include <sm_pegasus_demo_1/states/st_prepare_for_takeoff.hpp>
#include <sm_pegasus_demo_1/states/st_takeoff.hpp>
#include <sm_pegasus_demo_1/states/st_ascend.hpp>
#include <sm_pegasus_demo_1/states/st_fly_aisles_west.hpp>
#include <sm_pegasus_demo_1/states/st_scan_rack_upper.hpp>
#include <sm_pegasus_demo_1/states/st_scan_rack_lower.hpp>
#include <sm_pegasus_demo_1/states/st_fly_aisles_mid.hpp>
#include <sm_pegasus_demo_1/states/st_descend_to_spill.hpp>
#include <sm_pegasus_demo_1/states/st_inspect_spill.hpp>
#include <sm_pegasus_demo_1/states/st_climb_from_spill.hpp>
#include <sm_pegasus_demo_1/states/st_fly_aisles_east.hpp>
#include <sm_pegasus_demo_1/states/st_scan_shelves.hpp>
#include <sm_pegasus_demo_1/states/st_fly_to_overview.hpp>
#include <sm_pegasus_demo_1/states/st_overview_loiter.hpp>
#include <sm_pegasus_demo_1/states/st_figure_eight.hpp>
#include <sm_pegasus_demo_1/states/st_return_to_pad.hpp>
#include <sm_pegasus_demo_1/states/st_pre_land_descent.hpp>
#include <sm_pegasus_demo_1/states/st_obstacle_hold.hpp>
#include <sm_pegasus_demo_1/states/st_return_home.hpp>
#include <sm_pegasus_demo_1/states/st_land.hpp>
#include <sm_pegasus_demo_1/states/st_landed.hpp>
