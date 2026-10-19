#!/usr/bin/env python3
# Copyright 2026 RobosoftAI Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Restamp, pair and throttle the Pegasus depth camera stream.

Isaac Sim stamps its ROS 2 camera topics with SIMULATION time (seconds since
Play) while everything else in this mission - PX4's TF tree, the flown trail,
Pegasus's own ground-truth pose - is stamped in ROS (wall) time, so RViz can
never transform the depth cloud and silently drops it. Isaac also publishes
camera_info at a different rate and with different stamps than the depth image,
so image_transport's exact-time pairing in depth_image_proc only matches a few
frames, and it ignores the requested camera frequency.

This node forwards every depth image with header.stamp = now, republishes the
latest camera_info with the SAME stamp and frame, and throttles to max_rate_hz.
depth_image_proc then gets perfectly paired messages and its cloud lands in a
frame and time RViz can resolve.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraRestamp(Node):
    def __init__(self):
        super().__init__("camera_restamp")
        self.declare_parameter("in_depth", "/drone0/camera/depth")
        self.declare_parameter("in_info", "/drone0/camera/color/camera_info")
        self.declare_parameter("out_depth", "/camera/depth")
        self.declare_parameter("out_info", "/camera/camera_info")
        self.declare_parameter("max_rate_hz", 15.0)
        self.declare_parameter("frame_id", "")  # empty: keep the incoming frame_id
        # optional: also relay the color image (restamped, same throttle) for RViz
        self.declare_parameter("in_color", "")
        self.declare_parameter("out_color", "/camera/color/image_raw")

        in_depth = self.get_parameter("in_depth").value
        in_info = self.get_parameter("in_info").value
        out_depth = self.get_parameter("out_depth").value
        out_info = self.get_parameter("out_info").value
        max_rate = float(self.get_parameter("max_rate_hz").value)
        self._frame_id = self.get_parameter("frame_id").value
        self._min_period_ns = int(1e9 / max_rate) if max_rate > 0.0 else 0

        qos_out = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self._pub_depth = self.create_publisher(Image, out_depth, qos_out)
        self._pub_info = self.create_publisher(CameraInfo, out_info, qos_out)
        self._info = None
        self._last_pub_ns = None
        self._warned = False
        self._count = 0
        self.create_subscription(CameraInfo, in_info, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, in_depth, self._on_depth, qos_profile_sensor_data)
        in_color = self.get_parameter("in_color").value
        self._pub_color = None
        self._last_color_ns = None
        if in_color:
            self._pub_color = self.create_publisher(Image, self.get_parameter("out_color").value, qos_out)
            self.create_subscription(Image, in_color, self._on_color, qos_profile_sensor_data)
            self.get_logger().info("camera_restamp: color %s -> %s" % (in_color, self.get_parameter("out_color").value))
        self.get_logger().info(
            "camera_restamp: %s + %s -> %s + %s at <= %.0f Hz, ROS time stamps"
            % (in_depth, in_info, out_depth, out_info, max_rate)
        )

    def _on_info(self, msg):
        self._info = msg

    def _on_color(self, msg):
        now = self.get_clock().now()
        if self._last_color_ns is not None and (now.nanoseconds - self._last_color_ns) < self._min_period_ns:
            return
        self._last_color_ns = now.nanoseconds
        msg.header.stamp = now.to_msg()
        if self._frame_id:
            msg.header.frame_id = self._frame_id
        self._pub_color.publish(msg)

    def _on_depth(self, msg):
        if self._info is None:
            if not self._warned:
                self.get_logger().warn("camera_restamp: depth arriving but no camera_info yet")
                self._warned = True
            return
        now = self.get_clock().now()
        if self._last_pub_ns is not None and (now.nanoseconds - self._last_pub_ns) < self._min_period_ns:
            return
        self._last_pub_ns = now.nanoseconds
        stamp = now.to_msg()
        msg.header.stamp = stamp
        if self._frame_id:
            msg.header.frame_id = self._frame_id
        info = self._info
        info.header.stamp = stamp
        info.header.frame_id = msg.header.frame_id
        self._pub_info.publish(info)
        self._pub_depth.publish(msg)
        self._count += 1
        if self._count == 1:
            self.get_logger().info(
                "camera_restamp: first pair out (%dx%d %s, frame '%s')"
                % (msg.width, msg.height, msg.encoding, msg.header.frame_id)
            )


def main():
    rclpy.init()
    node = CameraRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
