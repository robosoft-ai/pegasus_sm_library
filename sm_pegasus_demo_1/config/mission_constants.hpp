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

#include <cl_px4_mr/client_behaviors/cb_px4_path_follower_base.hpp>
#include <cl_px4_mr/client_behaviors/cb_wait_for_heading_stable.hpp>
#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>
#include <cl_px4_mr/components/cp_tf_broadcaster.hpp>
#include <cl_px4_mr/utils/geo_utils.hpp>
#include <cl_px4_mr/utils/pattern_generators.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

// Every mission tunable, in one place:
//   1. flight envelope     - altitudes, speeds, leash, tolerances
//   2. camera gating       - depth cloud topic, watchdog, safety cone, hold policy
//   3. watchdog timing     - the formula every state derives its timeout from
//   4. the warehouse       - spawn point, corridors, aisles and the finale, in the
//                            Isaac Sim world frame
//   5. derived helpers     - frame conversion, routes, follower / TF / cone params
//
// Frames: the Isaac Sim world (Full Warehouse, Pegasus "map") is ENU with x east,
// y north, z up. PX4's local NED origin is the spawn point (first GPS fix), so NED
// north = world y - spawn.y, east = world x - spawn.x, down = -(world z - spawn.z).
// With the vehicle spawned at the world origin the PX4 TF frame `map` (ENU, from
// CpTfBroadcaster) and Pegasus's ground-truth `map` coincide.
//
// The warehouse, measured from the USD (scratch script warehouse_bbox.py): hall
// interior x -26.1..5.3, y -23.2..30.6, ceiling 9.0 m, lamps hanging from 5.83 m.
// Seven single rack rows (1.08 m wide, 6 m tall, shelves to 4.3 m) run north-south
// between y 8.74 and 24.97 at x -25.32, -20.36, -15.41, -10.46, -5.50, -0.55, 4.42,
// leaving six 3.9 m wide aisles. The floor south of the racks (y < 8.7) and the
// strip north of them (y 25..30.6) are open. A forklift (2.15 m tall) is parked in
// the east aisle at x 1.4..4.9, y 10.2..11.4.

namespace sm_pegasus_demo_1
{

using cl_px4_mr::NedPoint;

// ===========================================================================
// 1. Flight envelope
// ===========================================================================
constexpr std::chrono::seconds kInitialPause{5};  // StPause: hold on the ground before starting
// preflight heading gate (StWaitForReady): the parked vehicle faces world +x =
// NED east; arm only once the EKF heading holds still and agrees with that
constexpr float kSpawnHeadingRad = static_cast<float>(M_PI) / 2.0f;
constexpr double kHeadingGateMaxDriftDegS = 0.5;
constexpr double kHeadingGateWindowS = 5.0;
constexpr double kHeadingGateTimeoutS = 60.0;
constexpr float kHeadingGateTolDeg = 15.0f;
constexpr float kTakeoffAltitudeM = 1.5f;            // CbTakeOff target (above the spawn floor)
constexpr std::chrono::seconds kTakeoffTimeout{40};  // then re-arm and retry (StTakeoff -> StArmPX4)
// aisle cruise: above the second shelf (2.83 m) is not needed - the vehicle flies the
// aisle centre line, 1.9 m from the rack faces; 2.5 m keeps the camera looking at the
// middle shelves and puts the forklift roof (2.15 m) 0.35 m below the camera
constexpr float kCruiseAltitudeM = 2.5f;
constexpr float kClimbRateMps = 1.0f;
constexpr float kCruiseSpeedMps = 1.5f;  // carrot advance rate (3.9 m wide aisles)
// leash: must exceed 1.2 x (speed / 0.95) = 1.9; short so the vehicle tracks the
// polyline closely and turns the aisle-end corners tightly
constexpr float kCruiseLeashM = 2.5f;
constexpr float kArrivalXyTolM = 0.8f;
constexpr float kArrivalZTolM = 0.5f;
// camera pan at the end of the aisles (StScanShelves): hold position, sweep the
// heading +-kScanAmplitudeRad about the base heading, for kScanDuration
constexpr float kScanAmplitudeRad = 0.6f;  // +-34 deg
constexpr float kScanPeriodS = 10.0f;      // peak yaw rate 0.38 rad/s, just above the cone's yaw gate
constexpr float kScanBaseHeadingRad = 0.0f;  // NED north: looking at the rack ends and into the aisles
constexpr std::chrono::seconds kScanDuration{20};
// finale over the open floor, above everything but the lamps (5.83 m), camera
// along the path throughout (path-follower patterns, tangent yaw):
//   loiter laps -> figure eight -> back over the pad
constexpr float kOverviewAltitudeM = 4.5f;
constexpr float kOverviewSpeedMps = 2.5f;  // open floor: faster than the aisles
constexpr float kOverviewLeashM = 4.0f;    // > 1.2 x speed / 0.95
constexpr float kOverviewRadiusM = 6.0f;
constexpr int kOverviewOrbits = 3;
constexpr float kEightSizeM = 6.0f;        // centre to lobe tip (= the loiter radius: the loiter's entry point is a lobe tip)
constexpr int kEightLoops = 2;
// pre-landing descent in offboard, holding the spawn point, then AUTO_LAND
constexpr float kPreLandAltitudeM = 0.8f;
constexpr float kPreLandDescentRateMps = 0.5f;
constexpr float kPreLandXyTolM = 0.3f;

// ===========================================================================
// 2. Camera gating
// ===========================================================================
// PointCloud2 built by depth_image_proc from the Pegasus depth image, in the camera
// OPTICAL frame (frame_id "camera": z forward, x right, y down)
constexpr const char * kDepthCloudTopic = "/camera/points";
constexpr std::chrono::seconds kDepthCloudTimeout{2};  // no cloud for this long -> abort
// forward safety cone (CpForwardObstacleGuard) about the camera axis. The camera has
// a 70 deg horizontal field of view (+-35, +-27 vertical at 4:3), so a 20 deg half
// angle cone is fully inside the image. Keep the trigger range below half the aisle
// width (1.9 m from the centre line to the rack faces) plus margin, so the racks
// never enter the cone while cruising, and the aisle-end turns are gated by yaw rate.
constexpr float kConeHalfAngleDeg = 20.0f;
constexpr float kConeTriggerRangeM = 2.5f;
constexpr float kConeClearRangeM = 3.5f;
constexpr float kConeMinRangeM = 0.6f;     // the airframe's own returns (prop tips)
constexpr float kConeFloorCutoffM = 0.8f;  // returns more than this below the camera are floor, not obstacle
constexpr int kConeMinHits = 50;           // a dense depth cloud: hundreds of hits per real obstacle
constexpr int kConeTriggerClouds = 2;      // 0.13 s at 15 Hz
constexpr int kConeClearClouds = 10;       // 0.7 s
constexpr float kConeMaxYawRateRadS = 0.35f;  // clouds taken while yawing faster than this are ignored
constexpr float kObstacleHoldMaxS = 20.0f;  // StObstacleHold: give up waiting after this
constexpr int kMaxObstacleHolds = 3;        // one more -> land where we are

// ===========================================================================
// 3. Watchdog timing (the behaviors' watchdogs run on the wall clock; a slow
//    simulation stretches every leg, hence kRtfMargin)
// ===========================================================================
constexpr float kTimeoutMarginFactor = 2.0f;
constexpr float kTimeoutBaseS = 30.0f;
constexpr float kRtfMargin = 2.0f;

// ===========================================================================
// 4. The warehouse (Isaac Sim world frame, ENU)
// ===========================================================================
struct WorldPoint
{
  float x = 0.0f;  // east
  float y = 0.0f;  // north
  float z = 0.0f;  // up (floor height at the point; the route flies an altitude above it)
};

// where the Pegasus GUI spawns the vehicle (Position 0, 0, 0.07, Rotation 0, 0, 0,
// i.e. facing +x) - keep in sync with the GUI fields
constexpr WorldPoint kSpawnWorld{0.0f, 0.0f, 0.07f};

// the two open corridors the aisle route turns in: south of the rack ends (racks
// start at y 8.74, their hanging aisle signs at 8.83) and north of them (racks end
// at 24.97; the ceiling lamps at y 27.2..28.0 hang from 5.83 m)
constexpr float kSouthCorridorY = 5.5f;
constexpr float kNorthCorridorY = 26.5f;
// aisle centre lines (midway between adjacent rack rows), west to east
constexpr float kAisleX[] = {-22.84f, -17.89f, -12.93f, -7.98f, -3.02f, 1.94f};
constexpr int kAisleCount = 6;
// the aisle an abort escapes through (back to the south corridor from the north
// one): aisle 5, which has nothing on its floor
constexpr float kEscapeAisleX = -3.02f;
// where the aisle phase ends and the camera pan happens: on the south corridor in
// front of the pad, facing the racks
constexpr WorldPoint kScanPointWorld{0.0f, kSouthCorridorY, 0.0f};

// Rack scan (SsRackScan): in aisle 3 the vehicle turns to face the rack on the east
// side of the aisle (x -10.46, face 1.9 m from the lens: one shelf level fills the
// camera's vertical field) and flies the aisle SIDEWAYS twice, north at the upper
// level and back south at the lower level, so the box tracker sees the cartons
// head-on. Shelf decks are at 0.96-1.33, 2.46-2.83 and 3.76-4.13 m; cartons sit at
// 1.3-2.4 m (lower level) and 2.8-3.8 m (upper level).
constexpr int kRackScanAisle = 2;  // index into kAisleX (aisle 3, x -12.93)
constexpr float kRackScanYawRad = static_cast<float>(M_PI) / 2.0f;  // NED east: camera on the rack to the east
constexpr float kRackScanUpperAltM = 3.3f;
constexpr float kRackScanLowerAltM = 1.8f;
constexpr float kRackScanSpeedMps = 0.8f;  // slow: stable tracks on the stacked cartons
constexpr float kRackScanLeashM = 1.5f;    // > 1.2 x speed / 0.95

// Spill (SsWarehouseInspection): the wet-floor signs (x -3.7..-1.5, y 8.6..8.9),
// cones (y 9.2 and 14.3), bucket and dropped carton sit at the south mouth of aisle
// 5. The vehicle stops on the corridor line in front of them, drops down, pans
// across the scene and carries on. From the hover point the signs are 3.1 m ahead:
// outside the safety cone's trigger range, so the guard stays armed.
constexpr WorldPoint kSpillHoverWorld{kAisleX[4], kSouthCorridorY, 0.0f};
constexpr float kSpillLookAltM = 1.2f;  // floor 1.2 m below the lens: beyond the 0.8 m floor cutoff
constexpr float kSpillLookBaseHeadingRad = 0.0f;  // north, up the aisle
constexpr float kSpillLookAmplitudeRad = 0.44f;   // +-25 deg
constexpr float kSpillLookPeriodS = 8.0f;
constexpr std::chrono::seconds kSpillLookDuration{10};
constexpr float kSpillDescentRateMps = 0.6f;
// overview orbit over the open floor south of the pad (lamps at y -7.8..-7.0 hang
// from 5.83 m: above the orbit)
constexpr WorldPoint kOverviewCenterWorld{-10.5f, -8.0f, 0.0f};

// Aisle serpentine at cruise altitude: from the pad north to the south corridor,
// west along it to aisle 1, then north and south through every aisle in turn,
// turning in the corridors, and back to the scan point. It is flown in three legs
// with the rack scan (after aisle 2, in aisle 3) and the spill stop (at the mouth
// of aisle 5) between them. The sixth (east) aisle is blocked by the parked
// forklift: the safety cone stops the vehicle there (StObstacleHold) and the
// mission escapes via aisle 5 (StReturnHome) before the camera pan and the
// overview - the built-in negative test of the demo.
inline void appendAisle(std::vector<WorldPoint> & r, int i)
{
  const bool northbound = (i % 2 == 0);
  const float yIn = northbound ? kSouthCorridorY : kNorthCorridorY;
  const float yOut = northbound ? kNorthCorridorY : kSouthCorridorY;
  r.push_back({kAisleX[i], yIn, 0.0f});
  r.push_back({kAisleX[i], yOut, 0.0f});
}

// leg 1: pad -> south corridor -> aisles 1 and 2 -> the south end of aisle 3
inline std::vector<WorldPoint> aisleLegWestWorld()
{
  std::vector<WorldPoint> r;
  r.push_back({kSpawnWorld.x, kSouthCorridorY, 0.0f});
  appendAisle(r, 0);
  appendAisle(r, 1);
  r.push_back({kAisleX[kRackScanAisle], kSouthCorridorY, 0.0f});
  return r;
}

// leg 2 (after the rack scan, which ends where leg 1 did): aisle 3 north, aisle 4
// south -> the spill hover point at the mouth of aisle 5
inline std::vector<WorldPoint> aisleLegMidWorld()
{
  std::vector<WorldPoint> r;
  appendAisle(r, 2);
  appendAisle(r, 3);
  r.push_back(kSpillHoverWorld);
  return r;
}

// leg 3 (after the spill stop): aisle 5 north, aisle 6 south (the forklift) -> scan point
inline std::vector<WorldPoint> aisleLegEastWorld()
{
  std::vector<WorldPoint> r;
  appendAisle(r, 4);
  appendAisle(r, 5);
  r.push_back({kScanPointWorld.x, kScanPointWorld.y, 0.0f});
  return r;
}

// the whole serpentine (for logging)
inline std::vector<WorldPoint> aisleRouteWorld()
{
  std::vector<WorldPoint> r = aisleLegWestWorld();
  for (const auto & v : aisleLegMidWorld()) r.push_back(v);
  for (const auto & v : aisleLegEastWorld()) r.push_back(v);
  return r;
}

// the two sideways rack-scan passes (NED, yaw fixed by the follower)
inline std::vector<WorldPoint> rackScanUpperWorld()
{
  return {{kAisleX[kRackScanAisle], kSouthCorridorY, 0.0f}, {kAisleX[kRackScanAisle], kNorthCorridorY, 0.0f}};
}
inline std::vector<WorldPoint> rackScanLowerWorld()
{
  return {{kAisleX[kRackScanAisle], kNorthCorridorY, 0.0f}, {kAisleX[kRackScanAisle], kSouthCorridorY, 0.0f}};
}

// ===========================================================================
// 5. Derived helpers - not tunables
// ===========================================================================

// world point, flown altitudeM above its floor height -> PX4 local NED (the NED
// origin is the spawn point)
inline NedPoint worldToNed(WorldPoint p, float altitudeM)
{
  NedPoint n;
  n.x = p.y - kSpawnWorld.y;                 // north
  n.y = p.x - kSpawnWorld.x;                 // east
  n.z = -(p.z + altitudeM - kSpawnWorld.z);  // down
  return n;
}

inline std::vector<NedPoint> toNedRoute(const std::vector<WorldPoint> & w, float altitudeM)
{
  std::vector<NedPoint> route;
  for (const WorldPoint & p : w)
  {
    route.push_back(worldToNed(p, altitudeM));
  }
  return route;
}

inline std::vector<NedPoint> aisleRouteNed() { return toNedRoute(aisleRouteWorld(), kCruiseAltitudeM); }
inline std::vector<NedPoint> aisleLegWestNed() { return toNedRoute(aisleLegWestWorld(), kCruiseAltitudeM); }
inline std::vector<NedPoint> aisleLegMidNed() { return toNedRoute(aisleLegMidWorld(), kCruiseAltitudeM); }
inline std::vector<NedPoint> aisleLegEastNed() { return toNedRoute(aisleLegEastWorld(), kCruiseAltitudeM); }
inline std::vector<NedPoint> rackScanUpperNed() { return toNedRoute(rackScanUpperWorld(), kRackScanUpperAltM); }
inline std::vector<NedPoint> rackScanLowerNed() { return toNedRoute(rackScanLowerWorld(), kRackScanLowerAltM); }
inline NedPoint spillHoverNed(float altitudeM) { return worldToNed(kSpillHoverWorld, altitudeM); }
inline NedPoint scanPointNed(float altitudeM) { return worldToNed(kScanPointWorld, altitudeM); }
inline NedPoint overviewCenterNed() { return worldToNed(kOverviewCenterWorld, kOverviewAltitudeM); }
inline NedPoint homeAt(float altitudeM) { return worldToNed(kSpawnWorld, altitudeM); }

// the point of the orbit circle nearest the scan point (CbOrbitLocation starts
// at the bearing of the entry position, so fly there first)
inline NedPoint overviewEntryNed()
{
  const NedPoint c = overviewCenterNed();
  const NedPoint s = scanPointNed(kOverviewAltitudeM);
  const float dx = s.x - c.x;
  const float dy = s.y - c.y;
  const float d = std::max(std::hypot(dx, dy), 0.01f);
  NedPoint p = c;
  p.x += kOverviewRadiusM * dx / d;
  p.y += kOverviewRadiusM * dy / d;
  return p;
}

inline cl_px4_mr::PathFollowerParams overviewFollower()
{
  cl_px4_mr::PathFollowerParams f;
  f.groundSpeed = kOverviewSpeedMps;
  f.leash = kOverviewLeashM;
  f.arrivalXyTol = kArrivalXyTolM;
  f.arrivalZTol = kArrivalZTolM;
  f.yawMode = cl_px4_mr::YawMode::TANGENT;  // camera along the path (the patterns set this too)
  f.prependCurrentPosition = true;
  f.autoTimeoutFactor = 0.0f;
  return f;
}

// bearing (NED yaw) from the overview centre to the scan point: the loiter is
// entered there and the figure eight's lobe axis points there, so it starts at a
// lobe tip right where the loiter ends
inline float overviewEntryBearingRad()
{
  const NedPoint c = overviewCenterNed();
  const NedPoint s = scanPointNed(kOverviewAltitudeM);
  return std::atan2(s.y - c.y, s.x - c.x);
}

inline cl_px4_mr::FlightPatternLoiterParams overviewLoiterParams()
{
  const NedPoint c = overviewCenterNed();
  cl_px4_mr::FlightPatternLoiterParams p;
  p.centerX = c.x;
  p.centerY = c.y;
  p.altitudeAgl = kOverviewAltitudeM;
  p.radius = kOverviewRadiusM;
  p.count = kOverviewOrbits;
  p.faceCenter = false;  // tangent yaw: camera along the path, not at the centre
  return p;
}
inline float overviewOrbitLengthM()
{
  return kOverviewOrbits * 2.0f * static_cast<float>(M_PI) * kOverviewRadiusM;
}

inline cl_px4_mr::FlightPatternFigureEightParams figureEightParams()
{
  const NedPoint c = overviewCenterNed();
  cl_px4_mr::FlightPatternFigureEightParams p;
  p.centerX = c.x;
  p.centerY = c.y;
  p.altitudeAgl = kOverviewAltitudeM;
  p.size = kEightSizeM;
  p.heading = overviewEntryBearingRad();
  p.count = kEightLoops;
  return p;
}

inline cl_px4_mr::HeadingStableParams headingGateParams()
{
  cl_px4_mr::HeadingStableParams p;
  p.maxDriftRadS = kHeadingGateMaxDriftDegS * M_PI / 180.0;
  p.windowS = kHeadingGateWindowS;
  p.timeoutS = kHeadingGateTimeoutS;
  p.expectedHeadingRad = kSpawnHeadingRad;
  p.headingTolRad = kHeadingGateTolDeg * static_cast<float>(M_PI) / 180.0f;
  return p;
}

inline float routeLengthM(const std::vector<NedPoint> & path)
{
  float len = 0.0f;
  for (size_t i = 1; i < path.size(); ++i)
  {
    len += cl_px4_mr::nedDistance(path[i - 1], path[i]);
  }
  return len;
}

// corridor tests on NED points (NED x is world north)
inline bool inNorthCorridor(const NedPoint & p)
{
  return std::fabs(p.x - (kNorthCorridorY - kSpawnWorld.y)) < 0.6f;
}
inline bool inSouthCorridor(const NedPoint & p)
{
  return std::fabs(p.x - (kSouthCorridorY - kSpawnWorld.y)) < 0.6f;
}

// Abort leg. The vehicle is somewhere on the aisle route with `traversed` = the
// vertices it has passed; a straight line to the pad would cross the racks, so:
// back over the traversed vertices to the nearest corridor vertex, then - if that
// is the north corridor - along it to the escape aisle and down it to the south
// corridor, then to the scan point at `altitudeM`. With nothing traversed (an
// abort before the aisles, or in the open area during the finale) it is just the
// scan point.
inline std::vector<NedPoint> escapePath(const std::vector<NedPoint> & traversed, float altitudeM)
{
  std::vector<NedPoint> path;
  bool north = false;
  for (auto it = traversed.rbegin(); it != traversed.rend(); ++it)
  {
    NedPoint v = *it;
    v.z = worldToNed({0.0f, 0.0f, 0.0f}, altitudeM).z;  // escape at the requested altitude
    path.push_back(v);
    if (inNorthCorridor(v))
    {
      north = true;
      break;
    }
    if (inSouthCorridor(v))
    {
      break;
    }
  }
  if (north)
  {
    path.push_back(worldToNed({kEscapeAisleX, kNorthCorridorY, 0.0f}, altitudeM));
    path.push_back(worldToNed({kEscapeAisleX, kSouthCorridorY, 0.0f}, altitudeM));
  }
  path.push_back(scanPointNed(altitudeM));
  return path;
}

inline cl_px4_mr::PathFollowerParams cruiseFollower()
{
  cl_px4_mr::PathFollowerParams f;
  f.groundSpeed = kCruiseSpeedMps;
  f.leash = kCruiseLeashM;
  f.arrivalXyTol = kArrivalXyTolM;
  f.arrivalZTol = kArrivalZTolM;
  f.yawMode = cl_px4_mr::YawMode::TANGENT;  // nose along the path: the camera looks ahead
  f.prependCurrentPosition = true;
  f.autoTimeoutFactor = 0.0f;  // every state arms its own watchdog
  return f;
}

// sideways rack-scan passes: slow, short leash, heading fixed on the rack
inline cl_px4_mr::PathFollowerParams rackScanFollower()
{
  cl_px4_mr::PathFollowerParams f = cruiseFollower();
  f.groundSpeed = kRackScanSpeedMps;
  f.leash = kRackScanLeashM;
  f.yawMode = cl_px4_mr::YawMode::FIXED;
  f.fixedYaw = kRackScanYawRad;
  return f;
}

inline cl_px4_mr::TfBroadcasterParams tfParams()
{
  cl_px4_mr::TfBroadcasterParams p;  // map -> base_link / base_link_follow, static base_link -> camera
  p.sensorFrame = "camera";          // Pegasus MonocularCamera("camera"): frame_id of image, depth, camera_info
  p.sensorXyz[0] = 0.30;             // Pegasus default camera position, body FLU
  p.sensorXyz[1] = 0.0;
  p.sensorXyz[2] = 0.0;
  // body FLU -> camera OPTICAL (z forward, x right, y down): roll -90, yaw -90
  p.sensorRpy[0] = -M_PI / 2.0;
  p.sensorRpy[1] = 0.0;
  p.sensorRpy[2] = -M_PI / 2.0;
  return p;
}

inline cl_px4_mr::ForwardObstacleGuardParams coneParams()
{
  cl_px4_mr::ForwardObstacleGuardParams p;
  p.halfAngleRad = kConeHalfAngleDeg * static_cast<float>(M_PI) / 180.0f;
  p.triggerRangeM = kConeTriggerRangeM;
  p.clearRangeM = kConeClearRangeM;
  p.minRangeM = kConeMinRangeM;
  p.floorCutoffM = kConeFloorCutoffM;
  p.minHits = kConeMinHits;
  p.triggerClouds = kConeTriggerClouds;
  p.clearClouds = kConeClearClouds;
  p.maxYawRateRadS = kConeMaxYawRateRadS;
  p.opticalFrameInput = true;  // depth_image_proc cloud: camera optical frame
  return p;
}

// timeout = (length / speed * margin + base) * rtf margin
inline std::chrono::seconds legTimeout(float lengthM, float speedMps)
{
  const float seconds =
    (lengthM / std::max(speedMps, 0.1f) * kTimeoutMarginFactor + kTimeoutBaseS) * kRtfMargin;
  return std::chrono::seconds(static_cast<long>(std::ceil(seconds)));
}

}  // namespace sm_pegasus_demo_1
