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

"""Turn an isaac_capture_dataset.py capture into a YOLO dataset and fine-tune.

Run with the tracker venv:
    ~/venvs/box_tracker/bin/python train_box_detector.py --data ~/datasets/warehouse_boxes \
        --epochs 40 --model yolov8s.pt --out ~/venvs/box_tracker/box_detector.pt
Classes (in this order): cardboard_box, traffic_cone, forklift.
"""

import argparse
import json
import os
import random
import shutil

CLASSES = ["cardboard_box", "traffic_cone", "forklift"]


def convert(src, dst, val_fraction=0.1, seed=0):
    images = sorted(f for f in os.listdir(os.path.join(src, "images")) if f.endswith(".png"))
    random.Random(seed).shuffle(images)
    n_val = max(1, int(len(images) * val_fraction))
    split = {"val": images[:n_val], "train": images[n_val:]}
    stats = {c: 0 for c in CLASSES}
    for part, names in split.items():
        os.makedirs(os.path.join(dst, "images", part), exist_ok=True)
        os.makedirs(os.path.join(dst, "labels", part), exist_ok=True)
        for name in names:
            stem = name[:-4]
            with open(os.path.join(src, "labels_raw", stem + ".json")) as f:
                raw = json.load(f)
            w, h = raw["width"], raw["height"]
            lines = []
            for b in raw["boxes"]:
                x0, y0, x1, y1 = b["box"]
                cx, cy, bw, bh = (x0 + x1) / 2.0 / w, (y0 + y1) / 2.0 / h, (x1 - x0) / w, (y1 - y0) / h
                lines.append("%d %.6f %.6f %.6f %.6f" % (CLASSES.index(b["cls"]), cx, cy, bw, bh))
                stats[b["cls"]] += 1
            shutil.copy(os.path.join(src, "images", name), os.path.join(dst, "images", part, name))
            with open(os.path.join(dst, "labels", part, stem + ".txt"), "w") as f:
                f.write("\n".join(lines) + ("\n" if lines else ""))
    with open(os.path.join(dst, "data.yaml"), "w") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n" % os.path.abspath(dst))
        for i, c in enumerate(CLASSES):
            f.write("  %d: %s\n" % (i, c))
    print("dataset: %d train, %d val images, boxes per class %s" % (len(split["train"]), len(split["val"]), stats))
    return os.path.join(dst, "data.yaml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/datasets/warehouse_boxes"))
    ap.add_argument("--yolo", default=None, help="converted dataset dir (default <data>_yolo)")
    ap.add_argument("--model", default="yolov8s.pt")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--out", default=os.path.expanduser("~/venvs/box_tracker/box_detector.pt"))
    ap.add_argument("--convert-only", action="store_true")
    a = ap.parse_args()
    yolo_dir = a.yolo or (a.data.rstrip("/") + "_yolo")
    data_yaml = convert(a.data, yolo_dir)
    if a.convert_only:
        return
    from ultralytics import YOLO
    model = YOLO(a.model)
    results = model.train(data=data_yaml, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, device=0,
                          project=os.path.join(yolo_dir, "runs"), name="box_detector", exist_ok=True,
                          patience=15, verbose=False)
    best = os.path.join(results.save_dir, "weights", "best.pt")
    shutil.copy(best, a.out)
    print("fine-tuned weights:", a.out)


if __name__ == "__main__":
    main()
