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

# Terminal 4 of sm_pegasus_demo_1. Isaac Sim + Pegasus (playing), PX4
# (start_pegasus_px4.sh), QGC and the micro-ROS agent are started separately.
# This launch brings up:
#   1. camera_restamp.py: the Pegasus depth image + camera_info, restamped in ROS
#      time (Isaac stamps them with simulation time, which RViz cannot transform),
#      paired and throttled -> /camera/depth + /camera/camera_info; the color image
#      likewise -> /camera/color/image_raw (RViz's Image display lagged by minutes
#      on the raw 36 Hz sim-time-stamped stream)
#   2. depth_image_proc: that pair -> PointCloud2 (/camera/points, camera optical
#      frame) that OrCamera watches
#   3. RViz with the warehouse view (fixed frame map: camera image, depth cloud,
#      PX4 TF + flown trail, Pegasus ground-truth pose)
#   4. box_tracker.sh: YOLO-World + ByteTrack on the relayed color image ->
#      /box_tracker/image (annotated) + /box_tracker/detections (tracker:=false skips it)
#   5. the state machine (run_sm:=false brings up only the sensors and RViz)
#
# Every run's complete console output (stdout + stderr, unbuffered, no color
# codes) is tee'd to a fixed path so it can be read after the fact.
#
# Pegasus publishes the camera under /<drone_ns>/camera/... where drone_ns is
# "drone" + the Vehicle ID entered in the Pegasus GUI (default 0 -> drone0).

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RUNTIME_LOG = "/tmp/sm_pegasus_demo_1_latest.log"
RUN_STAMP = time.strftime("%Y%m%d_%H%M%S")
RUNTIME_LOG_DATED = f"/tmp/sm_pegasus_demo_1_{RUN_STAMP}.log"


def generate_launch_description():
    drone_ns = LaunchConfiguration("drone_ns")
    cloud_topic = LaunchConfiguration("cloud_topic")
    rviz = LaunchConfiguration("rviz")
    run_sm = LaunchConfiguration("run_sm")
    camera_rate = LaunchConfiguration("camera_rate")
    tracker = LaunchConfiguration("tracker")

    tee_prefix = (
        'bash -c \'stdbuf -oL -eL "$@" 2>&1 | tee '
        + RUNTIME_LOG
        + " "
        + RUNTIME_LOG_DATED
        + "' --"
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("sm_pegasus_demo_1"), "config", "sm_pegasus_demo_1.rviz"]
    )

    in_depth = ["/", drone_ns, "/camera/depth"]
    in_info = ["/", drone_ns, "/camera/color/camera_info"]
    in_color = ["/", drone_ns, "/camera/color/image_raw"]
    out_color = "/camera/color/image_raw"  # restamped + throttled copy for RViz
    # the restamped pair lives under one namespace, so image_transport's camera
    # subscriber in depth_image_proc derives /camera/camera_info by itself
    out_depth = "/camera/depth"
    out_info = "/camera/camera_info"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drone_ns",
                default_value="drone0",
                description="Pegasus ROS 2 namespace: 'drone' + the GUI Vehicle ID",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/camera/points",
                description="PointCloud2 built from the depth image (OrCamera subscribes here; "
                "must match kDepthCloudTopic in config/mission_constants.hpp)",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "run_sm",
                default_value="true",
                description="false: only the camera pipeline and RViz (look at the sensors without flying)",
            ),
            DeclareLaunchArgument(
                "tracker",
                default_value="true",
                description="run box_tracker.sh (YOLO-World + ByteTrack on the color image -> /box_tracker/image)",
            ),
            DeclareLaunchArgument(
                "camera_rate",
                default_value="15.0",
                description="max depth/cloud rate in Hz (Isaac ignores the camera frequency setting)",
            ),
            Node(
                package="sm_pegasus_demo_1",
                executable="camera_restamp.py",
                name="camera_restamp",
                output="screen",
                parameters=[
                    {
                        "in_depth": in_depth,
                        "in_info": in_info,
                        "out_depth": out_depth,
                        "out_info": out_info,
                        "in_color": in_color,
                        "out_color": out_color,
                        "max_rate_hz": camera_rate,
                    }
                ],
            ),
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="depth_to_cloud",
                output="screen",
                remappings=[
                    ("image_rect", out_depth),
                    ("points", cloud_topic),
                ],
                parameters=[{"queue_size": 2}],
            ),
            Node(
                package="sm_pegasus_demo_1",
                executable="box_tracker.sh",
                name="box_tracker",
                output="screen",
                condition=IfCondition(tracker),
                parameters=[
                    {
                        "image_topic": out_color,
                        "max_rate_hz": camera_rate,
                        # weights next to the venv (ultralytics otherwise downloads into the cwd).
                        # The large YOLO-World with this prompt set finds most cartons on the
                        # stocked racks; the small one found almost none (sweep 2026-10-16)
                        "model": os.path.expanduser("~/venvs/box_tracker/yolov8l-worldv2.pt"),
                        "classes": ["cardboard box", "carton", "package", "parcel", "traffic cone", "forklift"],
                        "conf": 0.15,
                        "label_min_conf": 0.3,
                    }
                ],
            ),
            # Without the state machine nobody broadcasts PX4's TF tree (CpTfBroadcaster
            # lives in OrPx4), so hang the camera frame off Pegasus's ground-truth
            # map -> drone_base_link instead: same mount, body FLU -> camera optical
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_static_tf",
                output="screen",
                condition=UnlessCondition(run_sm),
                arguments=[
                    "--x", "0.30", "--y", "0", "--z", "0",
                    "--roll", "-1.5707963", "--pitch", "0", "--yaw", "-1.5707963",
                    "--frame-id", "drone_base_link", "--child-frame-id", "camera",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
            ),
            Node(
                package="sm_pegasus_demo_1",
                executable="sm_pegasus_demo_1_node",
                output="screen",
                condition=IfCondition(run_sm),
                prefix=tee_prefix,
                additional_env={
                    "RCUTILS_LOGGING_BUFFERED_STREAM": "0",
                    "RCUTILS_COLORIZED_OUTPUT": "0",
                },
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
        ]
    )
