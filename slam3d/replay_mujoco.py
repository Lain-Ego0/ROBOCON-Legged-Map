#!/usr/bin/env python3
"""Replay MuJoCo-generated 3D LiDAR/IMU into the native GLIM estimator."""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.config import POLICY, SCENE, START_LOGISTICS
from sim.glim_bridge import GLIMBridge
from sim.localization import LidarScanner3D
from sim.robot import SdogPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=8.0)
    args = parser.parse_args()
    root = ROOT
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.opt.timestep = 0.002
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON
    data = mujoco.MjData(model)
    robot = SdogPolicy(model, data, POLICY)
    robot.reset(START_LOGISTICS, 0.0)
    scanner = LidarScanner3D(model, data)
    sensor_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name) for name in ("imu_accel", "imu_gyro")]
    sensor_adrs = [int(model.sensor_adr[sid]) for sid in sensor_ids]
    bridge = GLIMBridge(
        root / "build/glim-runtime/mujoco_glim_bridge",
        root / "slam3d/config",
        root / "build/glim-runtime",
    )
    imu_batch = []
    scans = 0
    point_counts = []
    results = []
    truth_by_stamp = {}
    origin_alignment = None
    scan_time = time.monotonic()

    def capture():
        nonlocal imu_batch, scans
        stamp = float(data.time)
        a, g = sensor_adrs
        accel = data.sensordata[a:a + 3].copy()
        gyro = data.sensordata[g:g + 3].copy()
        imu_batch.append((stamp, accel, gyro))
        cloud = scanner.capture_step(stamp)
        if cloud is not None:
            point_counts.append(len(cloud[1]))
            if bridge.submit(imu_batch, cloud):
                scans += 1
                pos, _, yaw = robot.pose()
                truth_by_stamp[cloud[0]] = (pos.copy(), yaw)
                imu_batch = []

    steps = int(args.duration / 0.02)
    for _ in range(steps):
        t = float(data.time)
        command = np.zeros(3, dtype=np.float32)
        if 1.0 <= t < 6.0 or 9.0 <= t < 14.0 or 17.0 <= t < 22.0:
            command[0] = 0.28
        elif 6.0 <= t < 9.0 or 14.0 <= t < 17.0:
            command[2] = 0.45
        robot.infer_step(command, post_step=capture)
        for record in bridge.drain_poses():
            raw = np.asarray(record["pose"], dtype=np.float64)
            qx, qy, qz, qw = raw[3:7]
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            if origin_alignment is None:
                angle = -yaw
                c, s = math.cos(angle), math.sin(angle)
                rotated = np.array([c * raw[0] - s * raw[1], s * raw[0] + c * raw[1]])
                origin_alignment = (angle, START_LOGISTICS[0] - rotated[0], START_LOGISTICS[1] - rotated[1])
            angle, tx, ty = origin_alignment
            c, s = math.cos(angle), math.sin(angle)
            estimate = np.array([
                c * raw[0] - s * raw[1] + tx,
                s * raw[0] + c * raw[1] + ty,
                (yaw + angle + math.pi) % (2 * math.pi) - math.pi,
            ])
            pos, gt_yaw = truth_by_stamp.get(record["stamp"], (robot.pose()[0], robot.pose()[2]))
            error = float(np.linalg.norm(estimate[:2] - pos[:2]))
            yaw_error = abs((estimate[2] - gt_yaw + math.pi) % (2 * math.pi) - math.pi)
            results.append((record["stamp"], error, yaw_error, estimate, pos.copy()))

    deadline = time.monotonic() + 15.0
    while bridge.status()["processed_scans"] < scans and time.monotonic() < deadline:
        for record in bridge.drain_poses():
            raw = np.asarray(record["pose"], dtype=np.float64)
            qx, qy, qz, qw = raw[3:7]
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            if origin_alignment is None:
                angle = -yaw
                c, s = math.cos(angle), math.sin(angle)
                rotated = np.array([c * raw[0] - s * raw[1], s * raw[0] + c * raw[1]])
                origin_alignment = (angle, START_LOGISTICS[0] - rotated[0], START_LOGISTICS[1] - rotated[1])
            angle, tx, ty = origin_alignment
            c, s = math.cos(angle), math.sin(angle)
            estimate = np.array([c * raw[0] - s * raw[1] + tx, s * raw[0] + c * raw[1] + ty,
                                 (yaw + angle + math.pi) % (2 * math.pi) - math.pi])
            truth = truth_by_stamp.get(record["stamp"])
            if truth is not None:
                pos, gt_yaw = truth
                error = float(np.linalg.norm(estimate[:2] - pos[:2]))
                yaw_error = abs((estimate[2] - gt_yaw + math.pi) % (2 * math.pi) - math.pi)
                results.append((record["stamp"], error, yaw_error, estimate, pos))
        time.sleep(0.05)
    bridge.close()
    status = bridge.status()
    elapsed = time.monotonic() - scan_time
    print(f"simulated={data.time:.2f}s wall={elapsed:.2f}s scans={scans} points/scan={point_counts[:5]}.. dropped={status['dropped_scans']}")
    print(f"GLIM status: {status['error'] or 'process exited cleanly'}; valid estimates={len(results)}")
    if results:
        errors = np.asarray([r[1] for r in results if r[1] > 0.0])
        if len(errors):
            print(f"aligned xy error: rmse={np.sqrt(np.mean(errors ** 2)):.3f}m max={np.max(errors):.3f}m")
        print(f"last estimate record: t={results[-1][0]:.3f}s pose={results[-1][3]}")
    if scans == 0 or not results:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
