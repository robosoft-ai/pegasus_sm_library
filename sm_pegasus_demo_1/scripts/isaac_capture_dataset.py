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

"""Capture a labelled detector dataset from the drone camera inside Isaac Sim.

NOT a ROS node: runs in Isaac Sim's Python (Window > Script Editor):

    exec(open("<this file>").read())          # labels the scene, starts capturing
    capture.stop()                            # when the flight is over

It labels the warehouse prims by name (cardboard boxes, traffic cones, the
forklift), attaches Replicator's rgb and tight 2D bounding-box annotators to the
Pegasus vehicle camera's render product, and every `every` app updates saves
    <out>/images/<n>.png            the camera frame (what the tracker sees)
    <out>/labels_raw/<n>.json       boxes [x0, y0, x1, y1], class name, occlusion
train_box_detector.py (in the tracker venv) turns that into a YOLO dataset and
trains the fine-tuned model. Fly the mission while it runs: aisles, the head-on
rack passes, the spill and the overview give the viewpoints the tracker meets.
"""

import json
import os
import time

import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from PIL import Image

CLASS_PREFIXES = {
    "cardboard_box": ("Box_", "SM_CardBoxA", "SM_CardBoxB", "SM_CardBoxD"),
    "traffic_cone": ("S_TrafficCone",),
    "forklift": ("forklift",),
}
DEFAULTS = dict(
    vehicle="/World/quadrotor",
    out=os.path.expanduser("~/datasets/warehouse_boxes"),
    every=15,             # app updates between saved frames (~0.1 s simulated at 250 updates/s)
    max_occlusion=0.75,   # drop boxes more occluded than this
    min_side_px=6,        # drop boxes smaller than this
)


def _class_of(name):
    for cls, prefixes in CLASS_PREFIXES.items():
        if any(name.startswith(p) for p in prefixes):
            return cls
    return None


def label_scene():
    # Walk the whole stage (Pegasus references the warehouse under /World, the
    # dump of the raw USD had it under /Root) and label the OUTERMOST prim of each
    # matching name - the nested mesh prims carry the same names and are covered
    # by their parent's label.
    from isaacsim.core.utils.semantics import add_labels
    stage = omni.usd.get_context().get_stage()
    counts = {k: 0 for k in CLASS_PREFIXES}
    for prim in stage.Traverse():
        cls = _class_of(prim.GetName())
        if cls is None:
            continue
        parent = prim.GetParent()
        if parent and parent.IsValid() and _class_of(parent.GetName()) is not None:
            continue
        add_labels(prim, labels=[cls], instance_name="class")
        counts[cls] += 1
    print("labelled:", counts)
    return counts


class DatasetCapture:
    def __init__(self, **overrides):
        self.cfg = dict(DEFAULTS)
        self.cfg.update(overrides)
        cfg = self.cfg
        os.makedirs(os.path.join(cfg["out"], "images"), exist_ok=True)
        os.makedirs(os.path.join(cfg["out"], "labels_raw"), exist_ok=True)

        # The camera's render product only exists once the simulation is playing
        # (Pegasus initializes the camera in start()), so the annotators are
        # attached lazily from the update callback.
        self._rp = None
        self._rgb = None
        self._bb = None

        existing = [int(f[:-4]) for f in os.listdir(os.path.join(cfg["out"], "images")) if f.endswith(".png")]
        self._n = (max(existing) + 1) if existing else 0
        self._tick = 0
        self._saved = 0
        self._t0 = time.time()
        self._sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="sm_pegasus_demo_1_dataset_capture")
        print("capture: armed -> %s, every %d updates, starting at frame %d (attaches to the camera on Play)"
              % (cfg["out"], cfg["every"], self._n))

    def stop(self):
        self._sub = None
        try:
            if self._rp is not None:
                self._rgb.detach([self._rp])
                self._bb.detach([self._rp])
        except Exception:
            pass
        print("capture: stopped, %d frames saved in %.0f s" % (self._saved, time.time() - self._t0))

    def _attach(self):
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
        try:
            vehicle = PegasusInterface().get_vehicle(self.cfg["vehicle"])
            cam = vehicle._graphical_sensors[0]._camera
            rp = getattr(cam, "_render_product_path", None)
        except Exception:
            return False
        if not rp:
            return False
        self._rp = rp
        self._rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        self._bb = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
        self._rgb.attach([rp])
        self._bb.attach([rp])
        print("capture: attached to render product %s" % rp)
        return True

    def _on_update(self, _e):
        self._tick += 1
        if self._rp is None:
            if self._tick % 60 == 0:  # ~4 times a second: wait for Play
                try:
                    self._attach()
                except Exception as exc:
                    print("capture: attach failed (%r)" % (exc,))
            return
        if self._tick % int(self.cfg["every"]):
            return
        try:
            rgb = self._rgb.get_data()
            bb = self._bb.get_data()
        except Exception as exc:
            self._errors = getattr(self, "_errors", 0) + 1
            if self._errors > 50:  # render product gone (stage reloaded): stop quietly
                print("capture: stopping (%r)" % (exc,))
                self.stop()
            return
        if rgb is None or bb is None or getattr(rgb, "size", 0) == 0:
            return
        data = bb["data"] if isinstance(bb, dict) else bb
        info = bb.get("info", {}) if isinstance(bb, dict) else {}
        id_to_labels = info.get("idToLabels", {})
        h, w = rgb.shape[0], rgb.shape[1]
        boxes = []
        for row in data:
            sid = int(row["semanticId"])
            x0, y0, x1, y1 = int(row["x_min"]), int(row["y_min"]), int(row["x_max"]), int(row["y_max"])
            occ = float(row["occlusionRatio"]) if "occlusionRatio" in row.dtype.names else 0.0
            lab = id_to_labels.get(sid, id_to_labels.get(str(sid), {}))
            cls = lab.get("class") if isinstance(lab, dict) else None
            if cls not in CLASS_PREFIXES:
                continue
            if occ > float(self.cfg["max_occlusion"]):
                continue
            if (x1 - x0) < self.cfg["min_side_px"] or (y1 - y0) < self.cfg["min_side_px"]:
                continue
            boxes.append({"cls": cls, "box": [max(0, x0), max(0, y0), min(w, x1), min(h, y1)], "occlusion": occ})
        name = "%06d" % self._n
        Image.fromarray(np.asarray(rgb)[:, :, :3]).save(os.path.join(self.cfg["out"], "images", name + ".png"))
        with open(os.path.join(self.cfg["out"], "labels_raw", name + ".json"), "w") as f:
            json.dump({"width": w, "height": h, "boxes": boxes}, f)
        self._n += 1
        self._saved += 1
        if self._saved % 50 == 0:
            print("capture: %d frames (%d boxes in the last one)" % (self._saved, len(boxes)))


def start(**overrides):
    label_scene()
    return DatasetCapture(**overrides)


if globals().get("CAPTURE_AUTOSTART", True):
    capture = start()
