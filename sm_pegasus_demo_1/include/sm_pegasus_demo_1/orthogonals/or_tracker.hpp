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

#include <cl_generic_sensor/cl_generic_sensor.hpp>
#include <sm_pegasus_demo_1/components/cp_box_inventory.hpp>
#include <smacc2/smacc.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace sm_pegasus_demo_1
{

using ClTracker = cl_generic_sensor::ClGenericSensor<vision_msgs::msg::Detection2DArray>;

// The box tracker's detections (a generic sensor client, no watchdog: the
// tracker is optional) and the per-zone inventory built from them
class OrTracker : public smacc2::Orthogonal<OrTracker>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<ClTracker>(std::string("/box_tracker/detections"));
    client->createComponent<CpBoxInventory>();
  }
};

// the inventory component, from any state (nullptr if the orthogonal is missing)
template <typename TState>
inline CpBoxInventory * boxInventory(TState * state)
{
  ClTracker * client = nullptr;
  auto * orthogonal = state->template getOrthogonal<OrTracker>();
  if (orthogonal == nullptr || !orthogonal->requiresClient(client) || client == nullptr) return nullptr;
  return client->template getComponent<CpBoxInventory>();
}

template <typename TState>
inline void setInventoryZone(TState * state, const char * zone)
{
  if (auto * inv = boxInventory(state)) inv->setZone(zone);
}

}  // namespace sm_pegasus_demo_1
