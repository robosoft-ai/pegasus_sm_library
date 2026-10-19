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

"""Chase camera for filming the Pegasus vehicle in Isaac Sim.

NOT a ROS node: this runs inside Isaac Sim's Python. It creates a camera prim
that is re-posed every frame from the Pegasus vehicle state - a few metres
behind and above the vehicle, levelled (no roll or pitch from the airframe),
yaw following the vehicle with a little smoothing, always looking at it - and
opens it in its own viewport window to record from.

Use it in one of two ways:

  1. Isaac Sim > Window > Script Editor, with the vehicle loaded, run:
       exec(open("<this file>").read())
     A "Chase Camera" window appears. `chase.stop()` removes it,
     `chase.set(back=4.0, up=1.5)` retunes it live.

  2. Automatically: the sm_pegasus_demo_1 patch of the Pegasus GUI delegate
     loads this file at "Load Vehicle" (path in SM_PEGASUS_DEMO_1_CHASE_CAMERA_SCRIPT or the
     default workspace location) and calls start().

Tunables (all in metres / degrees / per-frame smoothing factors): see DEFAULTS.
"""

import math

import numpy as np
import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import create_viewport_window
from pxr import Gf, Sdf, UsdGeom

DEFAULTS = dict(
    vehicle="/World/quadrotor",      # Pegasus stage prefix of the vehicle
    camera_path="/World/ChaseCamera",
    back=3.0,                        # metres behind the vehicle (along its heading)
    up=1.2,                          # metres above it
    side=0.0,                        # metres to the left (negative = right)
    look_up=0.2,                     # aim this much above the vehicle centre
    lead=0.0,                        # aim this far ahead of the vehicle (along its heading)
    pos_smoothing=0.12,              # per-frame low-pass on the camera position (1 = rigid)
    yaw_smoothing=0.08,              # per-frame low-pass on the follow heading
    focal_length=18.0,               # mm on a 20.955 mm aperture: ~60 deg horizontal
    window="Chase Camera",
    width=1920,
    height=1080,
)


def _yaw_from_quat_xyzw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ChaseCamera:
    _instances = {}

    def __init__(self, **overrides):
        self.cfg = dict(DEFAULTS)
        self.cfg.update(overrides)
        cfg = self.cfg

        stage = omni.usd.get_context().get_stage()
        cam = UsdGeom.Camera.Define(stage, Sdf.Path(cfg["camera_path"]))
        cam.GetFocalLengthAttr().Set(float(cfg["focal_length"]))
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
        xf = UsdGeom.Xformable(cam.GetPrim())
        xf.ClearXformOpOrder()
        self._op = xf.AddTransformOp()

        self._pos = None
        self._yaw = None
        self._vehicle = None
        self._warned = False

        self._window = create_viewport_window(
            cfg["window"], width=int(cfg["width"]), height=int(cfg["height"]),
            camera_path=Sdf.Path(cfg["camera_path"]))

        self._sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="sm_pegasus_demo_1_chase_camera")
        ChaseCamera._instances[cfg["camera_path"]] = self

    # live retune: chase.set(back=4.0, up=1.5, focal_length=24.0)
    def set(self, **overrides):
        self.cfg.update(overrides)
        if "focal_length" in overrides:
            stage = omni.usd.get_context().get_stage()
            UsdGeom.Camera(stage.GetPrimAtPath(self.cfg["camera_path"])).GetFocalLengthAttr().Set(
                float(overrides["focal_length"]))

    def stop(self):
        self._sub = None
        ChaseCamera._instances.pop(self.cfg["camera_path"], None)
        try:
            if self._window is not None:
                self._window.destroy()
        except Exception:
            pass
        self._window = None

    def _get_vehicle(self):
        if self._vehicle is not None:
            return self._vehicle
        try:
            from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
            self._vehicle = PegasusInterface().get_vehicle(self.cfg["vehicle"])
        except Exception:
            self._vehicle = None
        return self._vehicle

    def _on_update(self, _event):
        try:
            self._update()
        except Exception as exc:
            # the stage was cleared or reloaded under us (the camera prim is gone):
            # stop instead of raising on every frame and flooding the log
            print("chase camera: stopping (%r)" % (exc,))
            self.stop()

    def _update(self):
        v = self._get_vehicle()
        if v is None:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None or not stage.GetPrimAtPath(self.cfg["camera_path"]).IsValid():
            raise RuntimeError("camera prim %s no longer exists" % self.cfg["camera_path"])
        try:
            p = np.array(v.state.position, dtype=float)      # ENU, metres
            yaw = _yaw_from_quat_xyzw(v.state.attitude)      # FLU body yaw about ENU z
        except Exception:
            return
        if not np.all(np.isfinite(p)):
            return

        cfg = self.cfg
        if self._pos is None:
            self._pos = p.copy()
            self._yaw = yaw
        else:
            a = float(cfg["pos_smoothing"])
            self._pos += a * (p - self._pos)
            b = float(cfg["yaw_smoothing"])
            self._yaw = _wrap(self._yaw + b * _wrap(yaw - self._yaw))

        c, s = math.cos(self._yaw), math.sin(self._yaw)
        fwd = np.array([c, s, 0.0])
        left = np.array([-s, c, 0.0])
        eye = self._pos - float(cfg["back"]) * fwd + float(cfg["side"]) * left + np.array([0.0, 0.0, float(cfg["up"])])
        target = p + float(cfg["lead"]) * fwd + np.array([0.0, 0.0, float(cfg["look_up"])])
        if np.linalg.norm(target - eye) < 0.05:
            return
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0.0, 0.0, 1.0))
        self._op.Set(view.GetInverse())


def start(**overrides):
    """Create (or replace) the chase camera and return it."""
    path = overrides.get("camera_path", DEFAULTS["camera_path"])
    old = ChaseCamera._instances.get(path)
    if old is not None:
        old.stop()
    return ChaseCamera(**overrides)


# Script Editor convenience: `exec(open(path).read())` starts it as `chase`
if globals().get("CHASE_AUTOSTART", True):
    chase = start()
