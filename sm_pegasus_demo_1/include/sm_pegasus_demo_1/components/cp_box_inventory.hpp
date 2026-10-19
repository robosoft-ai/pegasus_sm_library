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

#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <chrono>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace sm_pegasus_demo_1
{

// Per-zone box inventory, local to this mission. Listens to the box tracker's
// detections (CpTopicSubscriber<Detection2DArray> on the same client) and keeps
// the set of unique track ids seen in the current zone; the states name the
// zone as they go (aisle legs, rack passes, the spill stop, the overview). The
// table is published as a std_msgs/String (one "zone: n" line per zone plus the
// total) for the tracker to draw along the bottom of its image, and logged at
// the end of the mission. Track ids are ByteTrack's: a carton seen again later
// gets a new id, so this is an upper bound - good enough for the overlay.
// Threading: onDetections runs on the ROS executor, setZone on the state
// machine thread; both take the mutex and touch no state machine APIs.
class CpBoxInventory : public smacc2::ISmaccComponent
{
public:
  explicit CpBoxInventory(std::string topic = "/inspection/inventory") : topic_(topic) {}

  void onInitialize() override
  {
    this->requiresComponent(subscriber_);
    subscriber_->onMessageReceived(&CpBoxInventory::onDetections, this);
    pub_ = getNode()->create_publisher<std_msgs::msg::String>(topic_, rclcpp::QoS(1).transient_local());
    timer_ = getNode()->create_wall_timer(std::chrono::seconds(1), [this]() { publish(); });
    RCLCPP_INFO(getLogger(), "CpBoxInventory: publishing the inventory table on %s", topic_.c_str());
  }

  void setZone(const std::string & zone)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (zone == zone_) return;
    zone_ = zone;
    if (zones_.find(zone) == zones_.end())
    {
      zones_[zone] = {};
      order_.push_back(zone);
    }
  }

  std::string table() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string out;
    for (const auto & z : order_)
    {
      out += z + ": " + std::to_string(zones_.at(z).size()) + "\n";
    }
    out += "total unique: " + std::to_string(all_.size());
    return out;
  }

  size_t total() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return all_.size();
  }

private:
  void onDetections(const vision_msgs::msg::Detection2DArray & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (zone_.empty()) return;
    for (const auto & d : msg.detections)
    {
      if (d.id.empty() || d.results.empty() || d.results[0].hypothesis.class_id != "box") continue;
      zones_[zone_].insert(d.id);
      all_.insert(d.id);
    }
  }

  void publish()
  {
    std_msgs::msg::String s;
    s.data = table();
    pub_->publish(s);
  }

  std::string topic_;
  smacc2::client_core_components::CpTopicSubscriber<vision_msgs::msg::Detection2DArray> * subscriber_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex mutex_;
  std::string zone_;
  std::map<std::string, std::set<std::string>> zones_;
  std::vector<std::string> order_;
  std::set<std::string> all_;
};

}  // namespace sm_pegasus_demo_1
