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

"""Cardboard-box tracker for the warehouse inspection camera.

Runs an image-based detector (YOLO-World, zero-shot from the text prompt
"cardboard box"; a fine-tuned YOLO weight file drops in through the `model`
parameter) and ByteTrack on the relayed color image, draws the tracked boxes
with their IDs, and publishes:
  - /box_tracker/image        the annotated image (bgr8) for RViz
  - /box_tracker/detections   vision_msgs/Detection2DArray, id = track id
It needs torch + ultralytics: run it through box_tracker.sh, which uses the
venv (default ~/venvs/box_tracker, override with SM_PEGASUS_DEMO_1_TRACKER_PYTHON).
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


class BoxTracker(Node):
    def __init__(self):
        super().__init__("box_tracker")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("out_image", "/box_tracker/image")
        self.declare_parameter("out_detections", "/box_tracker/detections")
        self.declare_parameter("model", "yolov8s-worldv2.pt")  # YOLO-World weights, or a fine-tuned YOLO .pt
        self.declare_parameter("classes", ["cardboard box"])   # YOLO-World text prompts (ignored for plain YOLO weights)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.5)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("label_min_conf", 0.4)  # draw the id/confidence text only above this
        self.declare_parameter("max_rate_hz", 15.0)
        self.declare_parameter("inventory_topic", "/inspection/inventory")  # std_msgs/String, drawn along the bottom
        # a track counts (total, published id) only after this many frames: ByteTrack
        # hands out a new id whenever a track breaks, which inflates unique counts
        self.declare_parameter("min_track_hits", 5)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._conf = float(p("conf"))
        self._iou = float(p("iou"))
        self._imgsz = int(p("imgsz"))
        self._device = str(p("device"))
        self._tracker = str(p("tracker"))
        self._label_min_conf = float(p("label_min_conf"))
        self._min_period_ns = int(1e9 / float(p("max_rate_hz"))) if float(p("max_rate_hz")) > 0 else 0

        self._model = self._load_model(str(p("model")), list(p("classes")))

        self._bridge = CvBridge()
        qos_out = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self._pub_image = self.create_publisher(Image, str(p("out_image")), qos_out)
        self._pub_det = self.create_publisher(Detection2DArray, str(p("out_detections")), qos_out)
        self.create_subscription(Image, str(p("image_topic")), self._on_image, qos_profile_sensor_data)

        self._inventory = ""
        self.create_subscription(String, str(p("inventory_topic")), self._on_inventory, 10)
        self._seen_ids = set()  # confirmed unique track ids so far (the running total, top right)
        self._hits = {}         # track id -> consecutive frames seen
        self._min_hits = int(p("min_track_hits"))
        self._last_ns = None
        self._frames = 0
        self._infer_s = 0.0
        self._t_report = time.time()
        self.get_logger().info(
            "box_tracker: %s on %s, classes %s, conf %.2f, <= %.0f Hz, publishing %s"
            % (p("model"), self._device, list(p("classes")), self._conf, float(p("max_rate_hz")), p("out_image"))
        )

    # class name (prompt or fine-tuned label) -> (key, BGR colour)
    @staticmethod
    def _style(name):
        n = str(name).lower()
        if "cone" in n:
            return "cone", (255, 140, 0)      # blue
        if "forklift" in n or "truck" in n:
            return "forklift", (60, 200, 60)  # green
        return "box", (40, 200, 255)          # orange: cardboard box / carton / package / parcel

    def _on_inventory(self, msg):
        self._inventory = msg.data

    def _load_model(self, weights, classes):
        import torch
        from ultralytics import YOLO, YOLOWorld

        if not torch.cuda.is_available() and self._device.startswith("cuda"):
            self.get_logger().warn("box_tracker: CUDA not available, falling back to CPU")
            self._device = "cpu"
        if "world" in weights.lower():
            model = YOLOWorld(weights)
            model.set_classes(classes)
        else:
            model = YOLO(weights)
        # warm up so the first camera frame is not the slow one
        import numpy as np
        model.predict(np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8), device=self._device, verbose=False)
        return model

    def _on_image(self, msg):
        now = self.get_clock().now()
        if self._last_ns is not None and (now.nanoseconds - self._last_ns) < self._min_period_ns:
            return
        self._last_ns = now.nanoseconds

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        t0 = time.time()
        results = self._model.track(
            frame, persist=True, conf=self._conf, iou=self._iou, imgsz=self._imgsz, device=self._device,
            tracker=self._tracker, verbose=False)
        self._infer_s += time.time() - t0
        self._frames += 1

        det_msg = Detection2DArray()
        det_msg.header = msg.header
        r = results[0]
        boxes = r.boxes
        n = 0
        present = set()
        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else [-1] * len(confs)
            clss = boxes.cls.cpu().numpy().astype(int)
            names = r.names
            for (x1, y1, x2, y2), c, tid, k in zip(xyxy, confs, ids, clss):
                cname = names.get(int(k), str(int(k))) if isinstance(names, dict) else names[int(k)]
                key, ccolor = self._style(cname)
                confirmed = False
                if tid >= 0:
                    present.add(int(tid))
                    self._hits[int(tid)] = self._hits.get(int(tid), 0) + 1
                    confirmed = self._hits[int(tid)] >= self._min_hits
                if key == "box":
                    n += 1
                    if confirmed:
                        self._seen_ids.add(int(tid))
                color = ccolor if c >= self._label_min_conf else (120, 120, 120)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2 if c >= self._label_min_conf else 1)
                if c >= self._label_min_conf:
                    label = ("#%d " % tid if tid >= 0 else "") + ("%.2f" % c if key == "box" else "%s %.2f" % (key, c))
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                    cv2.rectangle(frame, (int(x1), int(y1) - th - 6), (int(x1) + tw + 4, int(y1)), color, -1)
                    cv2.putText(frame, label, (int(x1) + 2, int(y1) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
                d = Detection2D()
                d.header = msg.header
                d.id = str(int(tid)) if confirmed else ""  # unconfirmed tracks publish no id (not counted)
                bb = BoundingBox2D()
                bb.center.position.x = float((x1 + x2) / 2.0)
                bb.center.position.y = float((y1 + y2) / 2.0)
                bb.size_x = float(x2 - x1)
                bb.size_y = float(y2 - y1)
                d.bbox = bb
                h = ObjectHypothesisWithPose()
                h.hypothesis.class_id = key  # box / cone / forklift
                h.hypothesis.score = float(c)
                d.results.append(h)
                det_msg.detections.append(d)

        # forget tracks that vanished (keeps the hit table small)
        for tid in [t for t in self._hits if t not in present]:
            del self._hits[tid]
        # top left: boxes in this frame; top right: unique boxes tracked so far; bottom: inventory by zone
        cv2.putText(frame, "boxes: %d" % n, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 200, 255), 2, cv2.LINE_AA)
        total = "total: %d" % len(self._seen_ids)
        (tw, _), _ = cv2.getTextSize(total, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.putText(frame, total, (frame.shape[1] - tw - 8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 200, 255), 2, cv2.LINE_AA)
        if self._inventory:
            y = frame.shape[0] - 8
            for line in reversed(self._inventory.split("\n")):
                (lw, lh), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(frame, (6, y - lh - 4), (10 + lw, y + 2), (0, 0, 0), -1)
                cv2.putText(frame, line, (8, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                y -= lh + 8
        out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = msg.header
        self._pub_image.publish(out)
        self._pub_det.publish(det_msg)

        if time.time() - self._t_report >= 5.0:
            self.get_logger().info(
                "box_tracker: %.1f fps, %.0f ms/frame, %d boxes in the last frame"
                % (self._frames / (time.time() - self._t_report), 1000.0 * self._infer_s / max(self._frames, 1), n))
            self._frames = 0
            self._infer_s = 0.0
            self._t_report = time.time()


def main():
    rclpy.init()
    node = BoxTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
