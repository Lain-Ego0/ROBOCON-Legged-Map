"""ROBOCON 2027 S-Dog obstacle / logistics simulator web application.

Run:
    python -m sim.app --port 8731
"""
from __future__ import annotations

import argparse
import io
import json
import math
import mimetypes
import os
import pathlib
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from . import scene as scene_builder
from .cargo import CargoManager
from .config import (
    ASSETS, CAMERA_PRESETS, CATEGORIES, CATEGORY_COLORS, CATEGORY_ZH,
    CMD_LIMITS, HOST, HOME, HOME_ORDER, LIDAR3D_CHANNELS, LIDAR3D_MAX_RANGE,
    LIDAR3D_NOISE_STD, LIDAR3D_PROFILE, LIDAR3D_RAYS_PER_SCAN, LIDAR3D_SCAN_HZ,
    MISSION_SECONDS, OBSTACLE_CHECKPOINT_RADIUS, OBSTACLE_ROUTE, POLICY, PORT,
    SCENE, START_LOGISTICS, START_MANUAL, START_OBSTACLE, STORAGE, TREE, WEB,
)
from .glim_bridge import GLIMBridge
from .localization import GLIMLocalizer, LidarScanner, LidarScanner3D
from .minimap import render as render_map
from .mission import MissionController, RuleScoring
from .planning import OccupancyGrid, PathFollower, PathPlan
from .render import RenderThread
from .robot import SdogPolicy, quat_to_yaw

MODES = {
    "logistics": {
        "label": "物资归位 · 决策树",
        "short": "任务赛",
        "description": "8 个物资箱从存放区搬运至对应归位区，最多携带 2 箱，优先同类叠放。",
        "spawn": START_LOGISTICS,
    },
    "obstacle": {
        "label": "障碍赛 · 越障航线",
        "short": "障碍赛",
        "description": "沿 8 个障碍的检查点自主行进，展示 S-Dog 策略与 GLIM 定位的越障表现。",
        "spawn": START_OBSTACLE,
    },
    "manual": {
        "label": "手动驾驶 · 自由探索",
        "short": "手动",
        "description": "W/S 前后、A/D 横移、Q/E 转向，Shift 加速，空格停车。",
        "spawn": START_MANUAL,
    },
}


class SimState:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.paused = False
        self.render_event = threading.Event()
        self.snapshot_lock = threading.Lock()
        self.snapshot = None
        self.frame_jpeg: bytes | None = None
        self.frame_id = 0
        self.render_error: str | None = None
        self.render_dirty = True
        self.camera_mode = "chase"
        self.orbit_azimuth = 135.0
        self.orbit_elevation = -28.0
        self.orbit_distance = 6.0
        self.reset_requested = False
        self.layout_requested: list[tuple[float, float]] | None = None
        self.layout_confirmed = False
        self.layout_applied = threading.Event()
        self.relocalize_requested = False
        self.pending_mode: str | None = None
        self.keys: set[str] = set()
        self.telemetry: dict = {"ok": False}
        self.map_png = b""
        self.config: dict = {}
        self.error: str | None = None


class ObstacleTour:
    def __init__(self, grid: OccupancyGrid, start):
        self.grid = grid
        self.index = 1
        self.points = [(p[0], p[1]) for p in OBSTACLE_ROUTE]
        self.labels = [p[2] for p in OBSTACLE_ROUTE]
        self.scores = [p[3] for p in OBSTACLE_ROUTE]
        self.reached = [True] + [False] * (len(self.points) - 1)
        self.follower = PathFollower()
        self.score = 0
        self.logs = []
        self.replans = 0
        self.plan_failed = False
        self.started = False
        self.finished = False
        self.elapsed = 0.0
        self.start = tuple(start)
        self._plan_next((start[0], start[1]))

    def _plan_next(self, pose, extra_clearance=0):
        if self.index >= len(self.points):
            self.finished = True
            self.follower.clear()
            return
        goal = self.points[self.index]
        plan = self.grid.plan(pose, goal, extra_clearance=extra_clearance)
        self.plan_failed = not plan.ok
        self.follower.set(plan)

    def start_tour(self):
        self.started = True
        self.elapsed = 0.0
        self.log("越障航线启动", "start")

    def log(self, text, kind="info"):
        self.logs.append({"t": round(self.elapsed, 1), "kind": kind, "text": text})
        self.logs = self.logs[-30:]

    def step(self, pose, dt):
        if not self.started or self.finished:
            return np.zeros(3, dtype=np.float32)
        self.elapsed += dt
        if self.elapsed >= MISSION_SECONDS:
            self.finished = True
            self.follower.clear()
            self.log("越障航线结束 · 180 s", "end")
            return np.zeros(3, dtype=np.float32)
        if self.plan_failed:
            self.finished = True
            self.log(f"無可行路徑 · {self.labels[self.index]}，停止航線", "warn")
            return np.zeros(3, dtype=np.float32)
        cmd, target, remain = self.follower.command(
            pose, lookahead=0.58, speed=0.43, fine_distance=0.4, dt=dt)
        if self.follower.stuck:
            if self.replans >= 2:
                self.finished = True
                self.log(f"路徑停滯且重規劃失敗 · {self.labels[self.index]}", "warn")
                return np.zeros(3, dtype=np.float32)
            self.replans += 1
            self._plan_next(pose[:2], extra_clearance=self.replans)
            self.log(f"路徑停滯 · 擴大障礙間距並重規劃 {self.replans}/2", "warn")
            return np.zeros(3, dtype=np.float32)
        goal = self.points[self.index]
        if remain < OBSTACLE_CHECKPOINT_RADIUS:
            self.reached[self.index] = True
            if self.scores[self.index]:
                self.score += self.scores[self.index]
            self.log(f"通过检查点 · {self.labels[self.index]} +{self.scores[self.index]}", "score")
            self.index += 1
            self.replans = 0
            if self.index >= len(self.points):
                self.finished = True
                self.follower.clear()
                return np.zeros(3, dtype=np.float32)
            self._plan_next((pose[0], pose[1]))
        return cmd

    def telemetry(self):
        task = {
            "kind": "tour", "state": "done" if self.finished else "run",
            "note": self.labels[self.index] if self.index < len(self.labels) else "完成",
            "target": self.index, "elapsed": 0.0, "timeout": 0.0,
        }
        return {
            "started": self.started, "finished": self.finished,
            "time": round(self.elapsed, 2), "time_left": round(max(0.0, MISSION_SECONDS - self.elapsed), 1),
            "decision_path": ["root", "empty", "pick_new"],
            "node_states": {}, "reason": f"航线检查点 {self.index}/{len(self.points)}",
            "action": "tour", "status": task["note"], "task": task,
            "logs": self.logs, "bar_crossed": False, "carried": 0, "carry_limit": 2,
            "score": self.score,
        }


class Simulator(threading.Thread):
    def __init__(self, state: SimState, args):
        super().__init__(name="robocon-sim", daemon=True)
        self.state = state
        self.args = args
        self.mode = "logistics"
        self.started = False
        self.auto_start_at = None
        self.map_png = b""
        self.last_telemetry = 0.0
        self.last_scan = -1.0
        self.last_score = 0.0
        self.last_snapshot = 0.0
        self.scan_clock = 0.0
        self._last_command = np.zeros(3, dtype=np.float32)
        self.last_reloc_attempt = -10.0
        self.lost_handled = False
        self.distance = 0.0
        self.last_gt = None
        self.wall_start = time.monotonic()
        self.tour: ObstacleTour | None = None

    # ------------------------------------------------------------------ setup
    def load(self):
        if not SCENE.exists():
            scene_builder.build()
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        model = self.model
        model.opt.timestep = 0.002
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON
        model.opt.iterations = 100
        if hasattr(model.opt, "ls_iterations"):
            model.opt.ls_iterations = 50
        self.data = mujoco.MjData(model)
        self.robot = SdogPolicy(model, self.data, POLICY)
        self.cargo = CargoManager(model, self.data)
        self.grid = OccupancyGrid(model, self.data)
        self.lidar = LidarScanner(model, self.data)
        self.lidar3d = LidarScanner3D(model, self.data)
        self.imu_accel_adr, self.imu_gyro_adr = self._sensor_addresses(model)
        self.scoring = RuleScoring()
        self.mission = MissionController(self.cargo, self.grid, self.scoring, START_LOGISTICS[:2])
        self.localizer = GLIMLocalizer(START_LOGISTICS[:2], 0.0)
        root = pathlib.Path(__file__).resolve().parents[1]
        self.glim_bridge = GLIMBridge(
            root / "build/glim-cpu-runtime/mujoco_glim_bridge",
            root / "slam3d/config",
            root / "build/glim-cpu-runtime",
        )
        self.glim_pose = None
        self.glim_stamp = None
        self.glim_alignment = None
        self.glim_generation = self.glim_bridge.generation
        self.imu_pending = []
        self.state.config = self._config()
        self.state.map_png = render_map(model, self.data)
        self.spawn = START_LOGISTICS
        self._reset_runtime(first=True)
        self.renderer = RenderThread(model, self.state, width=self.args.width, height=self.args.height)
        self.renderer.start()

    def _config(self):
        return {
            "categories": [{"id": c, "zh": CATEGORY_ZH[c], "color": CATEGORY_COLORS[c]} for c in CATEGORIES],
            "storage": [{"x": cargo.initial[0], "y": cargo.initial[1], "category": cargo.category}
                        for cargo in self.cargo.cargo],
            "storage_slots": [{"x": x, "y": y} for x, y, _ in STORAGE],
            "home": {c: list(HOME[c]) for c in HOME_ORDER},
            "tree": TREE,
            "modes": {k: {"label": v["label"], "short": v["short"], "description": v["description"]} for k, v in MODES.items()},
            "camera": CAMERA_PRESETS,
            "bounds": [-7.35, 14.35, -3.15, 3.15],
            "mission_seconds": MISSION_SECONDS,
            "policy": "sdog_mixed_moe_cts_100k.pt · 45 obs · 10-frame history",
            "lidar3d": {
                "profile": LIDAR3D_PROFILE,
                "channels": LIDAR3D_CHANNELS,
                "scan_hz": LIDAR3D_SCAN_HZ,
                "rays_per_scan": LIDAR3D_RAYS_PER_SCAN,
                "max_range_m": LIDAR3D_MAX_RANGE,
                "noise_std_m": LIDAR3D_NOISE_STD,
                "imu_hz": 500.0,
                "hardware_target_point_rate_hz": 200000.0,
            },
            "map_url": "/map.png",
        }

    def _reset_runtime(self, first=False):
        spawn = self.spawn
        self.robot.reset(spawn, 0.0)
        for _ in range(30):
            mujoco.mj_step(self.model, self.data)
        self.cargo.reset()
        self.localizer.reset(spawn[:2], 0.0)
        self.mission.reset(spawn[:2])
        self.scoring.reset()
        self.started = False
        self.distance = 0.0
        self.last_gt = None
        self.last_scan = -1.0
        self.lidar3d.reset()
        self.imu_pending.clear()
        self.glim_pose = None
        self.glim_stamp = None
        self.glim_alignment = None
        if not first and hasattr(self, "glim_bridge"):
            self.glim_generation = self.glim_bridge.reset()
        self.scan_clock = 0.0
        self._last_command = np.zeros(3, dtype=np.float32)
        self.last_reloc_attempt = -10.0
        self.last_score = 0.0
        self.tour = None
        if self.mode == "obstacle":
            self.tour = ObstacleTour(self.grid, spawn[:2])
        self.auto_start_at = None
        with self.state.lock:
            self.state.error = None
            self.state.render_dirty = True
        self._publish_snapshot()

    @staticmethod
    def _sensor_addresses(model):
        acc_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")
        gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")
        if acc_id < 0 or gyro_id < 0:
            raise ValueError("MuJoCo scene must define imu_accel and imu_gyro sensors")
        return int(model.sensor_adr[acc_id]), int(model.sensor_adr[gyro_id])

    def _capture_slam_sensors(self):
        stamp = float(self.data.time)
        accel = self.data.sensordata[self.imu_accel_adr:self.imu_accel_adr + 3].copy()
        gyro = self.data.sensordata[self.imu_gyro_adr:self.imu_gyro_adr + 3].copy()
        self.imu_pending.append((stamp, accel, gyro))
        scan = self.lidar3d.capture_step(stamp)
        if scan is not None and self.glim_bridge.submit(self.imu_pending, scan):
            self.imu_pending = []

    def _publish_snapshot(self):
        d = self.data
        snap = (d.qpos.copy(), d.qvel.copy(), d.act.copy(), d.ctrl.copy(),
                d.mocap_pos.copy(), d.mocap_quat.copy(), float(d.time))
        with self.state.snapshot_lock:
            self.state.snapshot = snap
        self.state.render_event.set()

    def _switch_mode(self, mode: str):
        if mode not in MODES:
            return
        self.mode = mode
        self.spawn = MODES[mode]["spawn"]
        self._reset_runtime()
        with self.state.lock:
            self.state.paused = False

    # ------------------------------------------------------------------- loop
    def run(self):
        try:
            if not hasattr(self, "model"):
                self.load()
        except Exception as exc:  # noqa: BLE001
            with self.state.lock:
                self.state.error = f"init: {type(exc).__name__}: {exc}"
            print(f"[sim] init failed: {type(exc).__name__}: {exc}", flush=True)
            return
        dt = 0.02
        next_tick = time.perf_counter()
        try:
            while not self.state.stop.is_set():
                now = time.perf_counter()
                if self.state.layout_requested is not None:
                    with self.state.lock:
                        layout = self.state.layout_requested
                        self.state.layout_requested = None
                    self.cargo.set_layout(layout)
                    self.state.config = self._config()
                    self.state.map_png = render_map(self.model, self.data)
                    with self.state.lock:
                        self.state.layout_confirmed = True
                        self.state.layout_applied.set()
                    self._publish_snapshot()
                if self.state.reset_requested:
                    with self.state.lock:
                        self.state.reset_requested = False
                    self._reset_runtime()
                if self.state.pending_mode is not None:
                    mode = self.state.pending_mode
                    with self.state.lock:
                        self.state.pending_mode = None
                    self._switch_mode(mode)
                if self.state.paused:
                    if self.state.render_dirty:
                        self._publish_snapshot()
                    time.sleep(0.02)
                    next_tick = time.perf_counter()
                    continue
                if now < next_tick:
                    time.sleep(min(next_tick - now, 0.004))
                    continue
                if now - next_tick > 0.12:
                    next_tick = now
                self._tick(dt)
                next_tick += dt
        except Exception as exc:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            with self.state.lock:
                self.state.error = f"runtime: {type(exc).__name__}: {exc}"
        finally:
            self.state.stop.set()
            self.state.render_event.set()
            if hasattr(self, "glim_bridge"):
                self.glim_bridge.close()

    def _tick(self, dt):
        st = self.state
        glim_age = math.inf if self.glim_stamp is None else max(0.0, float(self.data.time) - self.glim_stamp)
        glim_valid = self.glim_pose is not None and glim_age <= 0.75
        navigation_pose = self.glim_pose if glim_valid else self.localizer.pose
        # Automatic mission start only for the logistics demo.
        if self.auto_start_at is not None and time.monotonic() >= self.auto_start_at:
            self.auto_start_at = None
            if self.mode == "logistics":
                self.started = True
                self.mission.start()
            elif self.mode == "obstacle" and self.tour is not None:
                self.started = True
                self.tour.start_tour()
        if self.mode == "manual":
            cmd = self._manual_command()
        elif not glim_valid:
            cmd = np.zeros(3, dtype=np.float32)
        elif self.mode == "obstacle" and self.tour is not None:
            cmd = self.tour.step(navigation_pose, dt) if self.started else np.zeros(3, dtype=np.float32)
        else:
            cmd = self.mission.step(navigation_pose, self.robot, dt) if self.started else np.zeros(3, dtype=np.float32)
        cmd = np.asarray(cmd, dtype=np.float32)
        self._last_command = cmd.copy()
        cmd[0] = float(np.clip(cmd[0], -CMD_LIMITS[0], CMD_LIMITS[0]))
        cmd[1] = float(np.clip(cmd[1], -CMD_LIMITS[1], CMD_LIMITS[1]))
        cmd[2] = float(np.clip(cmd[2], -CMD_LIMITS[2], CMD_LIMITS[2]))
        self.robot.infer_step(
            cmd,
            pre_step=self.cargo.sync_carried,
            post_step=self._capture_slam_sensors,
        )
        for record in self.glim_bridge.drain_poses():
            if record.get("generation") != self.glim_generation:
                continue
            raw = np.asarray(record["pose"], dtype=np.float64)
            self.glim_stamp = float(record["stamp"])
            yaw_est = math.atan2(
                2.0 * (raw[6] * raw[5] + raw[3] * raw[4]),
                1.0 - 2.0 * (raw[4] ** 2 + raw[5] ** 2),
            )
            if self.glim_alignment is None:
                angle = -yaw_est
                c, s = math.cos(angle), math.sin(angle)
                aligned_xy = np.array([c * raw[0] - s * raw[1], s * raw[0] + c * raw[1]])
                self.glim_alignment = (angle, self.spawn[0] - aligned_xy[0], self.spawn[1] - aligned_xy[1])
            angle, tx, ty = self.glim_alignment
            c, s = math.cos(angle), math.sin(angle)
            self.glim_pose = np.array([
                c * raw[0] - s * raw[1] + tx,
                s * raw[0] + c * raw[1] + ty,
                (yaw_est + angle + math.pi) % (2.0 * math.pi) - math.pi,
            ])
        pos, quat, yaw = self.robot.pose()
        gt = np.array([pos[0], pos[1], yaw], dtype=np.float64)
        # Side-contact pickup.  During a pick task only the selected box can attach.
        target = None
        allow_pickup = self.mode == "manual"
        if self.mode == "logistics" and self.mission.task is not None and self.mission.task.t.name == "pick":
            target = self.mission.task.t.target
            allow_pickup = True
        cargo = self.cargo.try_pickup(pos, yaw, target_index=target) if allow_pickup else None
        if cargo is not None:
            self.mission.log(f"侧面接触 · 取物成功 #{cargo.index} {CATEGORY_ZH[cargo.category]}", "pickup")
        # Range-inertial localization.
        self.localizer.predict(dt, self.robot.body_velocity(), self.robot.yaw_rate_world())
        self.scan_clock += dt
        if self.scan_clock - self.last_scan >= 0.1 - 1e-9:
            scan = self.lidar.scan(pos, yaw)
            if st.relocalize_requested:
                st.relocalize_requested = False
                self.localizer.relocalize(scan)
            self.localizer.update(scan, gt)
            self.last_scan = self.scan_clock
        if self.last_gt is not None:
            self.distance += math.hypot(pos[0] - self.last_gt[0], pos[1] - self.last_gt[1])
        self.last_gt = (pos[0], pos[1])
        self._publish_snapshot()
        now = time.monotonic()
        if now - self.last_telemetry >= 0.10:
            self.last_telemetry = now
            self._publish_telemetry()
        if now - self.last_snapshot >= 0.20:
            self.last_snapshot = now

    def _manual_command(self):
        keys = set(self.state.keys)
        turbo = 1.6 if "shift" in keys else 1.0
        if "space" in keys:
            return np.zeros(3, dtype=np.float32)
        return np.array([
            (int("w" in keys) - int("s" in keys)) * 0.85 * turbo,
            (int("a" in keys) - int("d" in keys)) * 0.45 * turbo,
            (int("q" in keys) - int("e" in keys)) * 1.0 * turbo,
        ], dtype=np.float32)

    def _publish_telemetry(self):
        st = self.state
        pos, quat, yaw = self.robot.pose()
        gt = np.array([pos[0], pos[1], yaw], dtype=np.float64)
        glim_valid = self.glim_pose is not None and max(0.0, float(self.data.time) - self.glim_stamp) <= 0.75
        estimate_pose = self.glim_pose if glim_valid else self.localizer.pose
        scan = self.localizer.last_scan
        step = max(1, len(scan) // 140)
        scan_small = [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in scan[::step]]
        score = self.scoring.evaluate(self.cargo)
        mission_t = self.mission.telemetry()
        if self.mode == "obstacle" and self.tour is not None:
            tour = self.tour.telemetry()
            mission_t.update(tour)
            score = {"total": self.tour.score, "bar": 0, "items": [], "max": sum(p[3] for p in OBSTACLE_ROUTE), "battle": self.tour.score}
        path = []
        target = None
        task = None
        if self.mode == "logistics" and self.mission.task is not None:
            task = mission_t.get("task")
            follower = getattr(self.mission.task, "follower", None)
            if follower is not None and follower.plan is not None:
                path = [[round(x, 2), round(y, 2)] for x, y in follower.plan.points[::2]]
                if follower.plan.points:
                    target = list(follower.plan.points[-1])
        elif self.mode == "obstacle" and self.tour is not None:
            task = mission_t.get("task")
            follower = self.tour.follower
            if follower.plan is not None:
                path = [[round(x, 2), round(y, 2)] for x, y in follower.plan.points[::2]]
                if follower.plan.points:
                    target = list(follower.plan.points[-1])
        telemetry = {
            "ok": True,
            "server_time": time.time(),
            "mode": self.mode,
            "paused": st.paused,
            "started": self.started,
            "layout_confirmed": st.layout_confirmed,
            "sim_time": round(float(self.data.time), 2),
            "wall_time": round(time.monotonic() - self.wall_start, 1),
            "error": st.error,
            "render_error": st.render_error,
            "robot": {
                "gt": [round(float(gt[0]), 3), round(float(gt[1]), 3), round(float((gt[2] + math.pi) % (2 * math.pi) - math.pi), 3)],
                "est": [round(float(estimate_pose[0]), 3), round(float(estimate_pose[1]), 3), round(float((estimate_pose[2] + math.pi) % (2 * math.pi) - math.pi), 3)],
                "z": round(float(pos[2]), 3),
                "tilt": round(self.robot.tilt(), 1),
                "fallen": self.robot.fallen(),
                "velocity": [round(float(v), 2) for v in self.robot.body_velocity()[:2]],
                "command": [round(float(v), 2) for v in self._last_command] if hasattr(self, "_last_command") else [0, 0, 0],
                "yaw_rate": round(self.robot.yaw_rate(), 2),
            },
            "loc": {
                "status": self.localizer.status,
                "inliers": int(self.localizer.inliers),
                "rmse": round(float(self.localizer.rmse), 4),
                "correction": round(float(self.localizer.correction), 3),
                "latency_ms": round(float(self.localizer.latency_ms), 2),
                "error": round(float(math.hypot(self.localizer.pose[0] - gt[0], self.localizer.pose[1] - gt[1])), 4),
                "yaw_error": round(float(abs((self.localizer.pose[2] - gt[2] + math.pi) % (2 * math.pi) - math.pi)), 4),
                "keyframes": len(self.localizer.mapper.poses),
                "loop_closures": self.localizer.mapper.loop_closures,
                "mapped_points": sum(len(scan) for scan in self.localizer.mapper.scans),
                "scan": scan_small,
                "trajectory": [[round(float(x), 2), round(float(y), 2)] for x, y in self.localizer.trajectory[-240:]],
                "glim_status": self.glim_bridge.status(),
                "glim_pose": None if self.glim_pose is None else [round(float(v), 4) for v in self.glim_pose],
                "glim_stamp": self.glim_stamp,
                "glim_valid": glim_valid,
                "control_source": "manual" if self.mode == "manual" else "glim" if glim_valid else "stopped",
                "glim_latency_ms": None if self.glim_stamp is None else round(max(0.0, self.data.time - self.glim_stamp) * 1000.0, 1),
                "glim_error": None if self.glim_pose is None else round(float(math.hypot(self.glim_pose[0] - gt[0], self.glim_pose[1] - gt[1])), 4),
                "glim_yaw_error": None if self.glim_pose is None else round(float(abs((self.glim_pose[2] - gt[2] + math.pi) % (2 * math.pi) - math.pi)), 4),
            },
            "cargo": self.cargo.snapshot(),
            "decision": mission_t,
            "score": score,
            "path": path,
            "target": target,
            "task": task,
            "stats": {
                "distance": round(self.distance, 2),
                "carried": self.cargo.carried_count,
                "delivered": sum(1 for c in self.cargo.cargo if c.deposited_zone),
            },
            "camera": {"mode": st.camera_mode},
            "config": {"mission_seconds": MISSION_SECONDS},
        }
        with st.lock:
            st.telemetry = telemetry

    def _set_command_for_telemetry(self, cmd):
        self._last_command = cmd


class Handler(BaseHTTPRequestHandler):
    server_version = "Robocon2027Sim/1.0"

    def log_message(self, fmt, *args):
        return

    @property
    def state(self) -> SimState:
        return self.server.state

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _file(self, path: pathlib.Path, content_type: str):
        try:
            body = path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            return self._file(WEB / "index.html", "text/html; charset=utf-8")
        if path == "/static/style.css":
            return self._file(WEB / "style.css", "text/css; charset=utf-8")
        if path == "/static/app.js":
            return self._file(WEB / "app.js", "application/javascript; charset=utf-8")
        if path == "/map.png":
            body = self.state.map_png
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/config":
            return self._json(self.state.config)
        if path == "/api/state":
            with self.state.lock:
                data = self.state.telemetry
            return self._json(data)
        if path == "/frame.jpg":
            with self.state.lock:
                frame, fid = self.state.frame_jpeg, self.state.frame_id
            if frame is None:
                self.send_error(503, "waiting for first frame")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Id", str(fid))
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
            return
        if path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=sdogframe")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            last = -1
            try:
                while not self.state.stop.is_set():
                    with self.state.lock:
                        frame, fid = self.state.frame_jpeg, self.state.frame_id
                    if frame is not None and fid != last:
                        self.wfile.write(
                            b"--sdogframe\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                        self.wfile.flush()
                        last = fid
                    time.sleep(0.015)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            return
        self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            length = min(int(self.headers.get("Content-Length", "0")), 65536)
            payload = json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return self._json({"error": "invalid json"}, 400)
        if path == "/api/mission":
            action = payload.get("action")
            with self.state.lock:
                if action == "start":
                    if self.server.sim.started:
                        return self._json({"ok": True, "already": True})
                    if self.server.sim.mode == "logistics" and not self.state.layout_confirmed:
                        return self._json({"error": "请先确认物资布置"}, 409)
                    self.server.sim.auto_start_at = None
                    self.server.sim.started = True
                    self.state.paused = False
                    if self.server.sim.mode == "logistics":
                        self.server.sim.mission.start()
                    elif self.server.sim.tour is not None:
                        self.server.sim.tour.start_tour()
                elif action == "pause":
                    self.state.paused = True
                elif action == "resume":
                    self.state.paused = False
                    self.state.render_dirty = True
                elif action == "reset":
                    self.state.reset_requested = True
                    self.state.paused = False
                elif action == "relocalize":
                    self.state.relocalize_requested = True
            return self._json({"ok": True})
        if path == "/api/storage-layout":
            raw = payload.get("storage")
            if not isinstance(raw, list) or len(raw) != len(STORAGE):
                return self._json({"error": "需要提供全部 8 个物块位置"}, 400)
            positions = []
            try:
                entries = sorted(raw, key=lambda item: int(item["id"]))
                for index, item in enumerate(entries):
                    if int(item["id"]) != index:
                        raise ValueError("物块编号不完整")
                    x, y = float(item["x"]), float(item["y"])
                    if not (8.1 <= x <= 13.9 and -1.9 <= y <= 1.9):
                        raise ValueError("物块必须位于任务场地内")
                    positions.append((x, y))
                for i, (x, y) in enumerate(positions):
                    for ox, oy in positions[:i]:
                        if abs(x - ox) < 0.2 and abs(y - oy) < 0.2:
                            raise ValueError("物块之间不能重叠")
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                return self._json({"error": str(exc) or "物块坐标无效"}, 400)
            with self.state.lock:
                if self.server.sim.started or self.server.sim.mode != "logistics":
                    return self._json({"error": "仅可在任务赛启动前编辑物资布置"}, 409)
                self.state.layout_confirmed = False
                self.state.layout_applied.clear()
                self.state.layout_requested = positions
            if not self.state.layout_applied.wait(timeout=3.0):
                return self._json({"error": "仿真未能应用物资布置"}, 503)
            return self._json({"ok": True})
        if path == "/api/mode":
            mode = payload.get("mode")
            if mode not in MODES:
                return self._json({"error": "unknown mode"}, 400)
            with self.state.lock:
                self.state.pending_mode = mode
                self.state.paused = False
            return self._json({"ok": True})
        if path == "/api/camera":
            mode = payload.get("mode")
            with self.state.lock:
                if mode in ("chase", "top", "wide", "orbit"):
                    self.state.camera_mode = mode
                    if mode == "orbit":
                        self.state.orbit_azimuth = float(payload.get("azimuth", self.state.orbit_azimuth))
                        self.state.orbit_elevation = float(payload.get("elevation", self.state.orbit_elevation))
                        self.state.orbit_distance = float(payload.get("distance", self.state.orbit_distance))
                elif mode == "orbit_delta":
                    self.state.camera_mode = "orbit"
                    self.state.orbit_azimuth = (self.state.orbit_azimuth + float(payload.get("dazimuth", 0.0))) % 360.0
                    self.state.orbit_elevation = float(np.clip(self.state.orbit_elevation + float(payload.get("delevation", 0.0)), -88.0, 20.0))
                    self.state.orbit_distance = float(np.clip(self.state.orbit_distance + float(payload.get("ddistance", 0.0)), 1.8, 18.0))
            return self._json({"ok": True})
        if path == "/api/keys":
            keys = payload.get("keys", [])
            valid = {"w", "a", "s", "d", "q", "e", "space", "shift"}
            with self.state.lock:
                self.state.keys = {str(k).lower() for k in keys if str(k).lower() in valid}
            return self._json({"ok": True})
        self.send_error(404)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--no-autostart", action="store_true")
    parser.add_argument("--no-open", action="store_true")
    args = parser.parse_args()
    sources = (scene_builder.FIELD, scene_builder.ROBOT, pathlib.Path(scene_builder.__file__))
    if not SCENE.exists() or SCENE.stat().st_mtime < max(p.stat().st_mtime for p in sources):
        scene_builder.build()
    state = SimState()
    sim = Simulator(state, args)
    sim.load()
    state.config = sim.state.config
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.daemon_threads = True
    httpd.state = state
    httpd.sim = sim
    sim.start()
    url = f"http://{args.host}:{args.port}/"
    print(f"[sim] ROBOCON 2027 simulator ready: {url}", flush=True)
    print(f"[sim] policy: {POLICY}", flush=True)
    if not args.no_open:
        try:
            import webbrowser
            webbrowser.open(url)
        except Exception:
            pass
    try:
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        state.stop.set()
        httpd.shutdown()
        sim.join(timeout=3.0)
        httpd.server_close()


if __name__ == "__main__":
    main()
