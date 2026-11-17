"""EGL MuJoCo renderer with chase / tactical / orbit camera modes."""
from __future__ import annotations

import io
import math
import threading
import time

import mujoco
import numpy as np
from PIL import Image

from .robot import quat_to_yaw

CAMERAS = {
    "chase": (180.0, -19.0, 3.9),
    "top": (180.0, -89.5, 15.2),
    "wide": (142.0, -34.0, 9.5),
}


class RenderThread(threading.Thread):
    def __init__(self, model, state, width=1280, height=720, quality=86):
        super().__init__(name="mujoco-render", daemon=True)
        self.model = model
        self.state = state
        self.width = width
        self.height = height
        self.quality = quality
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.lookat = np.array([3.5, 0.0, 0.5])
        self.azimuth = 160.0
        self.elevation = -24.0
        self.distance = 10.0
        self.initialized = False

    def run(self):
        renderer = None
        try:
            renderer = mujoco.Renderer(self.model, height=self.height, width=self.width, max_geom=30000)
            data = mujoco.MjData(self.model)
            while not self.state.stop.is_set():
                if not self.state.render_event.wait(timeout=0.5):
                    continue
                self.state.render_event.clear()
                if self.state.paused and not self.state.render_dirty:
                    continue
                with self.state.snapshot_lock:
                    snap = self.state.snapshot
                if snap is None:
                    continue
                qpos, qvel, act, ctrl, mocap_pos, mocap_quat, sim_time = snap
                data.qpos[:] = qpos
                data.qvel[:] = qvel
                data.act[:] = act
                data.ctrl[:] = ctrl
                data.mocap_pos[:] = mocap_pos
                data.mocap_quat[:] = mocap_quat
                data.time = sim_time
                mujoco.mj_forward(self.model, data)
                self._camera(data)
                renderer.update_scene(data, camera=self.camera)
                rgb = renderer.render()
                buf = io.BytesIO()
                Image.fromarray(rgb).save(buf, format="JPEG", quality=self.quality, optimize=False)
                with self.state.lock:
                    self.state.frame_jpeg = buf.getvalue()
                    self.state.frame_id += 1
                    self.state.render_error = None
                    self.state.render_dirty = False
        except Exception as exc:  # noqa: BLE001
            with self.state.lock:
                self.state.render_error = f"{type(exc).__name__}: {exc}"
            print(f"[render] stopped: {type(exc).__name__}: {exc}", flush=True)
        finally:
            if renderer is not None:
                renderer.close()

    def _camera(self, data: mujoco.MjData):
        st = self.state
        mode = st.camera_mode
        root = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        base = data.qpos[self.model.jnt_qposadr[root]:self.model.jnt_qposadr[root] + 3]
        yaw = quat_to_yaw(data.qpos[self.model.jnt_qposadr[root] + 3:self.model.jnt_qposadr[root] + 7])
        if mode == "top":
            target = np.array([3.5, 0.0, 0.0])
            az, el, dist = CAMERAS["top"]
        elif mode == "wide":
            target = np.array([3.5, 0.0, 0.55])
            az, el, dist = CAMERAS["wide"]
        elif mode == "orbit":
            target = np.array([base[0], base[1], max(0.18, base[2])])
            az, el, dist = st.orbit_azimuth, st.orbit_elevation, st.orbit_distance
        else:
            fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            target = np.array([base[0] + fwd[0] * 0.85, base[1] + fwd[1] * 0.85, max(0.18, base[2] + 0.12)])
            az, el, dist = CAMERAS["chase"]
            az = math.degrees(yaw)
        if not self.initialized:
            self.lookat[:] = target
            self.azimuth, self.elevation, self.distance = az, el, dist
            self.initialized = True
        else:
            self.lookat = self.lookat * 0.82 + target * 0.18
            daz = (az - self.azimuth + 180.0) % 360.0 - 180.0
            self.azimuth = (self.azimuth + daz * 0.18) % 360.0
            self.elevation += (el - self.elevation) * 0.18
            self.distance += (dist - self.distance) * 0.18
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = self.lookat
        self.camera.azimuth = self.azimuth
        self.camera.elevation = self.elevation
        self.camera.distance = self.distance
