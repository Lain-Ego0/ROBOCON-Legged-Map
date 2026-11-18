"""2D range-inertial SLAM inspired by GLIM's keyframe/submap architecture."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.sparse import lil_matrix


@dataclass
class LocalizationResult:
    pose: np.ndarray
    gt: np.ndarray
    error: float
    yaw_error: float
    status: str
    inliers: int
    rmse: float
    correction: float
    latency_ms: float
    scan_local: np.ndarray
    trajectory: list


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, _wrap(b[2] - a[2])])


class OnlineSubmapMapper:
    """2D range-inertial SLAM: keyframe submaps, robust registration and loop factors."""

    def __init__(self):
        self.poses: list[np.ndarray] = []
        self.scans: list[np.ndarray] = []
        self.edges: list[tuple[int, int, np.ndarray, float, bool]] = []
        self.last_insert_time = -1.0
        self.loop_closures = 0
        self.last_loop_keyframe = -20
        self.loop_pairs: set[tuple[int, int]] = set()
        self._target_cache = None
        self._target_pose_count = -1

    def reset(self):
        self.poses.clear()
        self.scans.clear()
        self.edges.clear()
        self.last_insert_time = -1.0
        self.loop_closures = 0
        self.last_loop_keyframe = -20
        self.loop_pairs.clear()
        self._target_cache = None
        self._target_pose_count = -1

    @staticmethod
    def _world(scan: np.ndarray, pose: np.ndarray) -> np.ndarray:
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return scan @ np.array([[c, s], [-s, c]]) + pose[:2]

    @staticmethod
    def _voxel(points: np.ndarray, size: float = 0.055) -> np.ndarray:
        if len(points) == 0:
            return points.reshape(-1, 2)
        keys = np.floor(points / size).astype(np.int32)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(idx)]

    def _target(self, pose: np.ndarray, radius: float = 4.5):
        if not self.poses:
            return None
        if self._target_cache is not None and self._target_pose_count == len(self.poses):
            return self._target_cache
        ids = list(range(max(0, len(self.poses) - 14), len(self.poses)))
        chunks = [self._world(self.scans[i], self.poses[i]) for i in ids]
        self._target_cache = self._make_target(np.concatenate(chunks), 0.07)
        self._target_pose_count = len(self.poses)
        return self._target_cache

    @staticmethod
    def _make_target(cloud: np.ndarray, voxel: float = 0.07):
        points = OnlineSubmapMapper._voxel(cloud, voxel)
        if len(points) < 12:
            return None
        tree = cKDTree(points)
        _, ids = tree.query(points, k=min(9, len(points)))
        neighbors = points[ids]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        cov = np.einsum("nki,nkj->nij", centered, centered)
        _, vecs = np.linalg.eigh(cov)
        normals = vecs[:, :, 0]
        return points, normals, cKDTree(points)

    @staticmethod
    def _align(scan: np.ndarray, init: np.ndarray, target, iterations: int = 7):
        pose = init.copy()
        predicted = init.copy()
        if target is None or len(scan) < 20:
            return pose, 0, 1.0
        points, normals, tree = target
        source = scan[::max(1, len(scan) // 220)]
        best = (0, 1.0)
        for _ in range(iterations):
            world = OnlineSubmapMapper._world(source, pose)
            dist, ids = tree.query(world, k=1, workers=1)
            mask = dist < 0.48
            if mask.sum() < 18:
                break
            p, q, n = world[mask], points[ids[mask]], normals[ids[mask]]
            r = np.einsum("ij,ij->i", n, p - q)
            c, s = math.cos(pose[2]), math.sin(pose[2])
            sx, sy = source[mask, 0], source[mask, 1]
            J = np.stack((n[:, 0], n[:, 1],
                n[:, 0] * (-s * sx - c * sy) + n[:, 1] * (c * sx - s * sy)), axis=1)
            scale = 0.08
            w = 1.0 / (1.0 + (r / scale) ** 2)
            # Keep degenerate wall/corridor scans anchored to the inertial
            # prediction instead of allowing a low-residual lateral slide.
            prior_info = np.diag([1.0 / 0.045 ** 2, 1.0 / 0.045 ** 2, 1.0 / 0.035 ** 2])
            prior_delta = pose - predicted
            prior_delta[2] = _wrap(prior_delta[2])
            H = J.T @ (J * w[:, None]) + prior_info + np.diag([1e-3, 1e-3, 3e-3])
            g = J.T @ (r * w) + prior_info @ prior_delta
            try:
                delta = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                break
            if not np.all(np.isfinite(delta)):
                break
            delta[:2] = np.clip(delta[:2], -0.045, 0.045)
            delta[2] = np.clip(delta[2], -0.035, 0.035)
            pose += delta
            pose[2] = _wrap(pose[2])
            rmse = float(np.sqrt(np.mean(r ** 2)))
            if mask.sum() > best[0] or (mask.sum() == best[0] and rmse < best[1]):
                best = (int(mask.sum()), rmse)
            if np.linalg.norm(delta) < 1e-4:
                break
        return pose, best[0], best[1]

    def track(self, scan: np.ndarray, predicted: np.ndarray, sim_time: float):
        if not self.poses:
            self._insert(scan, predicted, sim_time)
            return predicted.copy(), len(scan), 0.0, False
        result, inliers, rmse = self._align(scan, predicted, self._target(predicted))
        accepted = inliers >= 22 and rmse < 0.20 and np.linalg.norm(result[:2] - predicted[:2]) < 0.10
        pose = result if accepted else predicted.copy()
        last = self.poses[-1]
        trans = np.linalg.norm(pose[:2] - last[:2])
        rot = abs(_wrap(pose[2] - last[2]))
        if accepted and sim_time - self.last_insert_time >= 0.25 and (trans >= 0.22 or rot >= 0.18):
            self._insert(scan, pose, sim_time)
            pose = self.poses[-1].copy()
            loop = self._try_loop(scan, pose)
            if loop:
                pose = self.poses[-1].copy()
        return pose, inliers, rmse, accepted

    def _insert(self, scan, pose, sim_time):
        idx = len(self.poses)
        pose = pose.copy()
        if idx:
            self.edges.append((idx - 1, idx, _between(self.poses[-1], pose), 1.0, False))
        self.poses.append(pose)
        self.scans.append(self._voxel(scan, 0.04))
        self.last_insert_time = sim_time
        self._target_cache = None
        self._add_local_factors(idx)
        self._optimize_graph()

    def _add_local_factors(self, current: int):
        if current < 4:
            return
        scan = self.scans[current]
        pose = self.poses[current]
        start = max(0, current - 10)
        for ref in range(start, current - 2):
            if np.linalg.norm(self.poses[ref][:2] - pose[:2]) > 3.0:
                continue
            target = self._make_target(self._world(self.scans[ref], self.poses[ref]))
            solved, inliers, rmse = self._align(scan, pose, target, iterations=5)
            correction = np.linalg.norm(solved[:2] - pose[:2])
            if inliers >= 24 and rmse < 0.12 and correction < 0.20:
                weight = float(np.clip(1.15 - 5.0 * rmse, 0.55, 1.15))
                self.edges.append((ref, current, _between(self.poses[ref], solved), weight, False))

    def _try_loop(self, scan, pose):
        current = len(self.poses) - 1
        if current < 20 or current - self.last_loop_keyframe < 18:
            return False
        candidates = [i for i in range(current - 18)
                      if 0.35 < np.linalg.norm(self.poses[i][:2] - pose[:2]) < 4.5]
        if not candidates:
            return False
        ranked = []
        candidates.sort(key=lambda i: np.linalg.norm(self.poses[i][:2] - pose[:2]))
        for i in candidates[:32]:
            ids = [j for j in range(max(0, i - 2), min(current, i + 3))]
            chunks = [self._world(self.scans[j], self.poses[j]) for j in ids]
            cloud = self._voxel(np.concatenate(chunks), 0.07)
            if len(cloud) < 30:
                continue
            tree = cKDTree(cloud)
            _, near = tree.query(cloud, k=min(9, len(cloud)))
            neighbors = cloud[near]
            centered = neighbors - neighbors.mean(axis=1, keepdims=True)
            _, vecs = np.linalg.eigh(np.einsum("nki,nkj->nij", centered, centered))
            target = (cloud, vecs[:, :, 0], tree)
            solved, inliers, rmse = self._align(scan, pose, target, iterations=12)
            delta = np.linalg.norm(solved[:2] - pose[:2])
            if inliers >= 45 and rmse < 0.055 and delta < 0.90:
                rank = inliers - 100 * rmse - delta
                ranked.append((rank, i, solved, rmse))
        if not ranked:
            return False
        ranked.sort(key=lambda item: item[0], reverse=True)
        best = ranked[0]
        for alternative in ranked[1:]:
            same_place = np.linalg.norm(self.poses[best[1]][:2] - self.poses[alternative[1]][:2]) < 0.55
            if not same_place and best[0] - alternative[0] < 3.0:
                return False
        _, i, solved, rmse = best
        pair = (i, current)
        if pair in self.loop_pairs:
            return False
        self.edges.append((i, current, _between(self.poses[i], solved), max(0.5, 1.0 - rmse * 5), True))
        self.loop_pairs.add(pair)
        self._optimize_graph(fixed_lag=False)
        self.loop_closures += 1
        self.last_loop_keyframe = current
        self._target_cache = None
        return True

    def _optimize_graph(self, fixed_lag: bool = True):
        n = len(self.poses)
        if n < 3:
            return
        initial = np.asarray(self.poses)
        start = max(1, n - 12) if fixed_lag else 1
        active_edges = [(a, b, z, w, loop) for a, b, z, w, loop in self.edges if b >= start]
        x0 = initial[start:].reshape(-1)
        sparsity = lil_matrix((3 * len(active_edges), 3 * (n - start)), dtype=np.int8)
        for e, (a, b, *_rest) in enumerate(active_edges):
            for node in (a, b):
                if node >= start:
                    col = node - start
                    sparsity[3 * e:3 * e + 3, 3 * col:3 * (col + 1)] = 1

        def residual(x):
            poses = initial.copy()
            poses[start:] = x.reshape(-1, 3)
            out = []
            for a, b, measurement, weight, loop in active_edges:
                error = _between(measurement, _between(poses[a], poses[b]))
                robust = 1.0 if not loop else 1.0 / math.sqrt(1.0 + (np.linalg.norm(error[:2]) / 0.18) ** 2)
                out.extend(error * np.array([12.0, 12.0, 8.0]) * math.sqrt(weight * robust))
            return np.asarray(out)

        result = least_squares(residual, x0, jac_sparsity=sparsity.tocsr(),
                               loss="soft_l1", f_scale=1.0, max_nfev=5, verbose=0)
        optimized = initial.copy()
        optimized[start:] = result.x.reshape(-1, 3)
        self.poses = [p.copy() for p in optimized]


class PriorMap:
    """Static 2D point/normal map sampled from the obstacle and task arenas."""

    def __init__(self, model, data):
        import mujoco

        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.robot_bodies = set()
        for name in ("base_link", "FL_hip", "FL_thigh", "FL_calf", "FL_foot",
                     "FR_hip", "FR_thigh", "FR_calf", "FR_foot",
                     "RL_hip", "RL_thigh", "RL_calf", "RL_foot",
                     "RR_hip", "RR_thigh", "RR_calf", "RR_foot"):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid >= 0:
                self.robot_bodies.add(int(bid))
        self.points, self.normals = self._sample()
        self.tree = cKDTree(self.points)
        self.bounds = (-7.05, 14.05, -2.95, 2.95)
        self.grid_res = 0.10
        self.distance_field = self._distance_field()

    def _sample(self) -> tuple[np.ndarray, np.ndarray]:
        import mujoco

        pts: list[np.ndarray] = []
        for gid in range(self.model.ngeom):
            if self.model.geom_group[gid] != 0:
                continue
            if int(self.model.geom_bodyid[gid]) in self.robot_bodies:
                continue
            name = self.model.geom(gid).name or ""
            if name.startswith("block_"):
                continue
            gtype = self.model.geom_type[gid]
            pos = self.data.geom_xpos[gid].copy()
            mat = self.data.geom_xmat[gid].reshape(3, 3)
            size = self.model.geom_size[gid]
            if gtype == mujoco.mjtGeom.mjGEOM_BOX:
                top = float(pos[2] + np.abs(mat[2, :]) @ size)
            elif gtype == mujoco.mjtGeom.mjGEOM_CYLINDER:
                top = float(pos[2] + np.abs(mat[2, :]) @ np.array([size[0], size[0], size[1]]))
            else:
                top = float(pos[2] + 0.5)
            if top < 0.08:
                continue
            if gtype == mujoco.mjtGeom.mjGEOM_BOX:
                pts.extend(self._box_faces(pos, mat, size))
            elif gtype == mujoco.mjtGeom.mjGEOM_CYLINDER:
                pts.extend(self._cylinder(pos, mat, size))
            elif gtype == mujoco.mjtGeom.mjGEOM_MESH:
                pts.extend(self._mesh(pos, mat, gid))
            elif gtype in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CAPSULE):
                pts.extend(self._cylinder(pos, mat, size))
        points = np.concatenate([np.asarray(a, dtype=np.float64).reshape(-1, 2) for a in pts], axis=0) if pts else np.zeros((0, 2))
        # Keep only field-scale points and de-duplicate at 3 cm.
        mask = (points[:, 0] > -7.2) & (points[:, 0] < 14.2) & (points[:, 1] > -3.1) & (points[:, 1] < 3.1)
        points = points[mask]
        if len(points) == 0:
            return np.zeros((0, 2)), np.zeros((0, 2))
        keys = np.round(points / 0.03).astype(np.int64)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(unique_idx)]
        normals = self._estimate_normals(points)
        return points, normals

    @staticmethod
    def _box_faces(pos, mat, size):
        hx, hy, hz = size
        # Only vertical faces matter for 2D range matching.
        faces = []
        for axis in (0, 1):
            for sign in (-1.0, 1.0):
                u = np.linspace(-hx if axis else -hy, hx if axis else hy, max(2, int(2 * (hx if axis else hy) / 0.12) + 1))
                for z in np.linspace(-hz, hz, max(2, int(2 * hz / 0.12) + 1)):
                    if axis == 0:
                        local = np.stack([np.full_like(u, sign * hx), u, np.full_like(u, z)], axis=1)
                    else:
                        local = np.stack([u, np.full_like(u, sign * hy), np.full_like(u, z)], axis=1)
                    world = local @ mat.T + pos
                    faces.append(world[:, :2])
        return faces

    @staticmethod
    def _cylinder(pos, mat, size):
        r, h = size[0], size[1]
        angles = np.linspace(0, 2 * np.pi, 18, endpoint=False)
        rings = []
        for z in (-h, 0.0, h):
            local = np.stack([r * np.cos(angles), r * np.sin(angles), np.full_like(angles, z)], axis=1)
            rings.append((local @ mat.T + pos)[:, :2])
        return rings

    def _mesh(self, pos, mat, gid):
        import mujoco

        mesh_id = int(self.model.geom_dataid[gid])
        if mesh_id < 0:
            return []
        adr = int(self.model.mesh_vertadr[mesh_id])
        count = int(self.model.mesh_vertnum[mesh_id])
        verts = self.model.mesh_vert[adr:adr + count]
        world = verts @ mat.T + pos
        return [world[:, :2]]

    @staticmethod
    def _estimate_normals(points: np.ndarray) -> np.ndarray:
        if len(points) < 4:
            return np.zeros_like(points)
        tree = cKDTree(points)
        _, idx = tree.query(points, k=min(10, len(points)))
        neigh = points[idx]
        centered = neigh - neigh.mean(axis=1, keepdims=True)
        cov = np.einsum("nki,nkj->nij", centered, centered)
        vals, vecs = np.linalg.eigh(cov)
        normals = vecs[:, :, 0]
        # Orient consistently (sign is irrelevant for point-to-line residuals).
        flip = (normals[:, 0] * points[:, 0] + normals[:, 1] * points[:, 1]) < 0
        normals[flip] *= -1.0
        return normals

    def _distance_field(self):
        x0, x1, y0, y1 = self.bounds
        nx = int((x1 - x0) / self.grid_res) + 1
        ny = int((y1 - y0) / self.grid_res) + 1
        occ = np.ones((nx, ny), dtype=bool)
        ij = np.round((self.points - np.array([x0, y0])) / self.grid_res).astype(int)
        ij[:, 0] = np.clip(ij[:, 0], 0, nx - 1)
        ij[:, 1] = np.clip(ij[:, 1], 0, ny - 1)
        occ[ij[:, 0], ij[:, 1]] = False
        return distance_transform_edt(occ) * self.grid_res


class LidarScanner:
    """MuJoCo ray-cast LiDAR in the robot body frame."""

    def __init__(self, model, data, rays_per_layer: int = 90):
        import mujoco

        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.rays = rays_per_layer
        self.angles = np.linspace(-np.pi, np.pi, rays_per_layer, endpoint=False)
        self.geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.max_range = 9.0
        self.rng = np.random.default_rng(7)
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.robot_bodies = set()
        if base_id >= 0:
            for body_id in range(model.nbody):
                parent = body_id
                while parent > 0 and parent != base_id:
                    parent = int(model.body_parentid[parent])
                if parent == base_id:
                    self.robot_bodies.add(body_id)

    def scan(self, base_pos: np.ndarray, yaw: float, rng: np.random.Generator | None = None) -> np.ndarray:
        rng = rng or self.rng
        heights = np.array([max(0.09, base_pos[2] - 0.15), base_pos[2], base_pos[2] + 0.17, base_pos[2] + 0.33])
        out = []
        gid = np.zeros(1, dtype=np.int32)
        c, s = math.cos(yaw), math.sin(yaw)
        for h in heights:
            origin = np.array([base_pos[0], base_pos[1], h], dtype=np.float64)
            for a in self.angles:
                world_a = yaw + a + rng.normal(0.0, 0.0022)
                direction = np.array([math.cos(world_a), math.sin(world_a), 0.0])
                travelled = 0.0
                while travelled < self.max_range:
                    dist = self.mujoco.mj_ray(
                        self.model, self.data, origin, direction,
                        self.geomgroup, 1, -1, gid)
                    if dist < 0.0:
                        break
                    travelled += dist
                    geom_id = int(gid[0])
                    body_id = int(self.model.geom_bodyid[geom_id])
                    geom_name = self.model.geom(geom_id).name or ""
                    if body_id not in self.robot_bodies and not geom_name.startswith("block_"):
                        break
                    advance = dist + 0.015
                    origin = origin + direction * advance
                    travelled += 0.015
                if dist < 0.12 or travelled > self.max_range:
                    continue
                dist = travelled + rng.normal(0.0, 0.004)
                out.append((dist * math.cos(a), dist * math.sin(a)))
        return np.asarray(out, dtype=np.float64).reshape(-1, 2)


class LidarScanner3D:
    """Spinning multi-beam LiDAR synthesized from MuJoCo ray casts."""

    def __init__(
        self,
        model,
        data,
        channels: int | None = None,
        scan_hz: float | None = None,
        rays_per_scan: int | None = None,
    ):
        import mujoco

        from .config import (
            LIDAR3D_AZIMUTH_GOLDEN_RATIO,
            LIDAR3D_CHANNELS,
            LIDAR3D_MAX_RANGE,
            LIDAR3D_NOISE_STD,
            LIDAR3D_RAYS_PER_SCAN,
            LIDAR3D_SCAN_HZ,
        )

        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_lidar_site")
        if self.site_id < 0:
            raise ValueError("MuJoCo model is missing imu_lidar_site")
        self.channels = channels or LIDAR3D_CHANNELS
        self.scan_hz = scan_hz or LIDAR3D_SCAN_HZ
        self.rays_per_scan = rays_per_scan or LIDAR3D_RAYS_PER_SCAN
        if self.rays_per_scan % self.channels:
            raise ValueError("rays_per_scan must be divisible by channels")
        self.azimuth_slices = self.rays_per_scan // self.channels
        scan_steps = max(1, round((1.0 / self.scan_hz) / 0.002))
        self.azimuths_per_step = max(1, self.azimuth_slices // scan_steps)
        self.azimuth_slices = self.azimuths_per_step * scan_steps
        self.elevations = np.deg2rad([-25, -18, -12, -8, -5, -2, 1, 4, 7, 10, 14, 19, 25, 32, 40, 50])[:self.channels]
        self.geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.max_range = LIDAR3D_MAX_RANGE
        self.noise_std = LIDAR3D_NOISE_STD
        self.azimuth_ratio = LIDAR3D_AZIMUTH_GOLDEN_RATIO
        self.rng = np.random.default_rng(19)
        self.slice_index = 0
        self.sample_index = 0
        self.scan_start = None
        self.points: list[tuple[float, float, float, float]] = []
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.robot_bodies = set()
        for body_id in range(model.nbody):
            parent = body_id
            while parent > 0 and parent != base_id:
                parent = int(model.body_parentid[parent])
            if parent == base_id:
                self.robot_bodies.add(body_id)

    def reset(self):
        self.slice_index = 0
        self.sample_index = 0
        self.scan_start = None
        self.points.clear()

    def capture_step(self, stamp: float):
        if self.scan_start is None:
            self.scan_start = stamp
        origin = self.data.site_xpos[self.site_id].copy()
        rotation = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
        # A golden-ratio sequence avoids repeating the same azimuth grid
        # every revolution while retaining deterministic replay. Numpy keeps
        # direction generation out of the per-ray Python loop.
        sample_indices = self.sample_index + np.arange(self.azimuths_per_step, dtype=np.float64)
        phases = (sample_indices * self.azimuth_ratio) % 1.0
        azimuths = 2.0 * math.pi * phases
        elevations = self.elevations[None, :]
        cos_elevations = np.cos(elevations)
        local_directions = np.stack(
            (
                cos_elevations * np.cos(azimuths[:, None]),
                cos_elevations * np.sin(azimuths[:, None]),
                np.broadcast_to(np.sin(elevations), (self.azimuths_per_step, self.channels)),
            ),
            axis=-1,
        )
        self.sample_index += self.azimuths_per_step

        flat_local = local_directions.reshape(-1, 3)
        flat_world = (rotation @ flat_local.T).T
        geom_ids = np.full(len(flat_world), -1, dtype=np.int32)
        distances = np.full(len(flat_world), -1.0, dtype=np.float64)
        self.mujoco.mj_multiRay(
            self.model,
            self.data,
            origin,
            np.ascontiguousarray(flat_world.reshape(-1)),
            self.geomgroup,
            1,
            -1,
            geom_ids,
            distances,
            None,
            len(flat_world),
            self.max_range,
        )
        valid = (distances >= 0.12) & (distances < self.max_range)
        if np.any(valid):
            noisy_distances = distances[valid] + self.rng.normal(0.0, self.noise_std, int(valid.sum()))
            local_points = flat_local[valid] * noisy_distances[:, None]
            point_time = stamp - self.scan_start
            self.points.extend(
                (float(point[0]), float(point[1]), float(point[2]), point_time)
                for point in local_points
            )
        self.slice_index += self.azimuths_per_step
        if self.slice_index < self.azimuth_slices:
            return None
        result = (self.scan_start, np.asarray(self.points, dtype=np.float64), stamp)
        self.slice_index = 0
        self.scan_start = None
        self.points = []
        return result


class GLIMLocalizer:
    """Online range-inertial estimator backed by scan-built submaps."""

    def __init__(self, spawn: tuple[float, float], yaw: float = 0.0):
        self.pose = np.array([spawn[0], spawn[1], yaw], dtype=np.float64)
        self.predicted = self.pose.copy()
        self.velocity = np.zeros(2, dtype=np.float64)
        self.gyro_bias = 0.0
        self.vel_bias = np.zeros(2, dtype=np.float64)
        self.rng = np.random.default_rng(7)
        self.status = "TRACKING"
        self.inliers = 0
        self.rmse = 0.0
        self.correction = 0.0
        self.latency_ms = 0.0
        self.last_scan_time = 0.0
        self.lost_since = None
        self.trajectory: list[tuple[float, float]] = [(float(spawn[0]), float(spawn[1]))]
        self.last_scan = np.zeros((0, 2))
        self.scan_count = 0
        self.last_good_pose = self.pose.copy()
        self.yaw_observability = 0.0
        self.mapper = OnlineSubmapMapper()

    def reset(self, spawn: tuple[float, float], yaw: float = 0.0) -> None:
        self.pose[:] = [spawn[0], spawn[1], yaw]
        self.predicted[:] = self.pose
        self.velocity[:] = 0.0
        self.gyro_bias = 0.0
        self.vel_bias[:] = 0.0
        self.status = "TRACKING"
        self.inliers = 0
        self.rmse = 0.0
        self.correction = 0.0
        self.lost_since = None
        self.trajectory = [(float(spawn[0]), float(spawn[1]))]
        self.last_scan = np.zeros((0, 2))
        self.last_good_pose = self.pose.copy()
        self.yaw_observability = 0.0
        self.mapper.reset()
        self.scan_count = 0

    def predict(self, dt: float, v_body: np.ndarray, yaw_rate: float) -> None:
        """IMU + leg-odometry factor."""
        self.gyro_bias += self.rng.normal(0.0, 0.00025)
        self.vel_bias += self.rng.normal(0.0, 0.0008, 2)
        omega = yaw_rate + self.gyro_bias + self.rng.normal(0.0, 0.012)
        vel = np.asarray(v_body, dtype=np.float64)[:2] + self.vel_bias + self.rng.normal(0.0, 0.015, 2)
        yaw = self.pose[2] + omega * dt
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[0] += (c * vel[0] - s * vel[1]) * dt
        self.pose[1] += (s * vel[0] + c * vel[1]) * dt
        self.pose[2] = (yaw + math.pi) % (2 * math.pi) - math.pi
        self.predicted[:] = self.pose

    def update(self, scan_local: np.ndarray, gt: np.ndarray) -> LocalizationResult:
        t0 = time.perf_counter()
        self.last_scan = scan_local
        if len(scan_local) > 240:
            scan_local = scan_local[::2]
        if len(scan_local) >= 30:
            solved, inliers, rmse, accepted = self.mapper.track(
                scan_local, self.pose, self.scan_count * 0.1)
            self.scan_count += 1
            correction = float(np.hypot(*(solved[:2] - self.pose[:2])))
            if accepted:
                self.pose = solved
                self.last_good_pose = self.pose.copy()
                self.inliers = inliers
                self.rmse = rmse
                self.correction = correction
                self.status = "TRACKING" if (inliers >= 35 and rmse < 0.12) else "DEGRADED"
                self.lost_since = None
            else:
                self.inliers = inliers
                self.rmse = rmse
                self.correction = correction
                self.status = "DEGRADED"
                if self.lost_since is None:
                    self.lost_since = time.perf_counter()
                elif time.perf_counter() - self.lost_since > 3.0:
                    self.status = "LOST"
            if self.mapper.loop_closures and len(self.mapper.poses) > 1:
                # Rebuild the displayed path from optimized keyframe states.
                self.trajectory = [(float(p[0]), float(p[1])) for p in self.mapper.poses]
        self.last_scan_time = time.perf_counter()
        self.latency_ms = (self.last_scan_time - t0) * 1000.0
        if not self.trajectory or math.hypot(self.pose[0] - self.trajectory[-1][0], self.pose[1] - self.trajectory[-1][1]) > 0.12:
            self.trajectory.append((float(self.pose[0]), float(self.pose[1])))
            if len(self.trajectory) > 900:
                del self.trajectory[:200]
        err = float(math.hypot(self.pose[0] - gt[0], self.pose[1] - gt[1]))
        yaw_err = float(abs((self.pose[2] - gt[2] + math.pi) % (2 * math.pi) - math.pi))
        return LocalizationResult(
            pose=self.pose.copy(), gt=np.asarray(gt, dtype=np.float64), error=err,
            yaw_error=yaw_err, status=self.status, inliers=self.inliers,
            rmse=self.rmse, correction=self.correction, latency_ms=self.latency_ms,
            scan_local=scan_local, trajectory=self.trajectory[-240:],
        )

    def restore_last_good(self) -> None:
        """Fall back to the last accepted scan pose before a global retry."""
        self.pose = self.last_good_pose.copy()
        self.status = "DEGRADED"

    def relocalize(self, scan_local: np.ndarray) -> bool:
        """Relocalize against the accumulated scan map, never the simulator map."""
        if len(scan_local) < 25 or not self.mapper.poses:
            return False
        t0 = time.perf_counter()
        self.status = "RELOCALIZING"
        prev = self.pose.copy()
        candidates = sorted(range(len(self.mapper.poses)),
                            key=lambda i: np.linalg.norm(self.mapper.poses[i][:2] - prev[:2]))
        solved, inl, rmse = prev, 0, 1.0
        for i in candidates[:12]:
            target = self.mapper._target(self.mapper.poses[i], radius=2.5)
            candidate, count, error = self.mapper._align(scan_local, self.mapper.poses[i], target, 12)
            if count > inl or (count == inl and error < rmse):
                solved, inl, rmse = candidate, count, error
        if inl < 30 or rmse >= 0.16 or np.linalg.norm(solved[:2] - prev[:2]) > 3.0:
            self.status = "LOST"
            return False
        self.pose = solved
        self.predicted = self.pose.copy()
        self.last_good_pose = self.pose.copy()
        self.inliers = inl
        self.rmse = rmse
        self.status = "TRACKING" if (inl >= 35 and rmse < 0.12) else "DEGRADED"
        self.lost_since = None
        self.trajectory = [(float(p[0]), float(p[1])) for p in self.mapper.poses]
        self.latency_ms = (time.perf_counter() - t0) * 1000.0
        return inl >= 30 and rmse < 0.16
