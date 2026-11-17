"""GLIM-style 2.5D range-inertial localization against a prior field map.

The real GLIM pipeline couples IMU pre-integration with GPU-accelerated
scan-to-submap factors.  This simulator keeps the same estimator structure but
uses a lighter 2D formulation that is fast enough to run beside MuJoCo:

* a prior point/normal map is sampled from the static competition geometry,
* a 3-layer, 360-ray LiDAR is ray-cast against the live MuJoCo field,
* a constant-velocity + gyro prediction supplies the IMU/odometry factor,
* robust point-to-line scan matching supplies the range factor,
* a coarse global branch search recovers from tracking loss.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree


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

    def scan(self, base_pos: np.ndarray, yaw: float, rng: np.random.Generator | None = None) -> np.ndarray:
        rng = rng or np.random.default_rng()
        heights = np.array([max(0.09, base_pos[2] - 0.15), base_pos[2], base_pos[2] + 0.17, base_pos[2] + 0.33])
        out = []
        gid = np.zeros(1, dtype=np.int32)
        c, s = math.cos(yaw), math.sin(yaw)
        for h in heights:
            origin = np.array([base_pos[0], base_pos[1], h], dtype=np.float64)
            for a in self.angles:
                world_a = yaw + a + rng.normal(0.0, 0.0022)
                direction = np.array([math.cos(world_a), math.sin(world_a), 0.0])
                dist = self.mujoco.mj_ray(
                    self.model, self.data, origin, direction,
                    self.geomgroup, 1, -1, gid)
                if dist < 0.12 or dist > self.max_range:
                    continue
                dist += rng.normal(0.0, 0.009)
                out.append((dist * math.cos(a), dist * math.sin(a)))
        return np.asarray(out, dtype=np.float64).reshape(-1, 2)


class GLIMLocalizer:
    """Range-inertial localizer with robust point-to-line scan matching."""

    def __init__(self, prior: PriorMap, spawn: tuple[float, float], yaw: float = 0.0):
        self.prior = prior
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
            init = self._local_search(scan_local, self.pose)
            solved, inliers, rmse = self._icp(scan_local, init)
            correction = float(np.hypot(*(solved[:2] - self.pose[:2])))
            if inliers >= 25 and rmse < 0.20 and correction < 0.28:
                # Yaw is carried by the IMU/gyro factor.  The range factor
                # only corrects x/y; this avoids symmetric-corridor flips.
                solved[2] = self.pose[2]
                self.pose = solved
                self.last_good_pose = self.pose.copy()
                self.inliers = inliers
                self.rmse = rmse
                self.correction = correction
                self.status = "TRACKING" if (inliers >= 60 and rmse < 0.105) else "DEGRADED"
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

    def _local_search(self, scan: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """Coarse correlative search around the prediction.

        This is the same idea as a small branch-and-bound range factor: it
        escapes the point-to-line local minima that occur in a rectangular
        arena with repetitive walls.
        """
        sub = scan[:: max(1, len(scan) // 70)][:80]
        x0, y0 = self.prior.bounds[0], self.prior.bounds[2]
        res = self.prior.grid_res
        field = self.prior.distance_field
        nx, ny = field.shape
        best = (-1.0, pose.copy())
        for dyaw in (-0.14, -0.07, 0.0, 0.07, 0.14):
            yaw = pose[2] + dyaw
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.array([[c, -s], [s, c]])
            local = sub @ rot.T
            for dx in (-0.32, -0.16, 0.0, 0.16, 0.32):
                for dy in (-0.32, -0.16, 0.0, 0.16, 0.32):
                    pts = local + np.array([pose[0] + dx, pose[1] + dy])
                    ij = np.round((pts - np.array([x0, y0])) / res).astype(int)
                    if np.any(ij[:, 0] < 0) or np.any(ij[:, 0] >= nx) or np.any(ij[:, 1] < 0) or np.any(ij[:, 1] >= ny):
                        continue
                    dist = field[ij[:, 0], ij[:, 1]]
                    score = float(np.mean(np.exp(-(dist / 0.10) ** 2)))
                    if score > best[0]:
                        best = (score, np.array([pose[0] + dx, pose[1] + dy, yaw]))
        if best[0] < 0.18:
            return pose.copy()
        return best[1]

    def _icp(self, scan: np.ndarray, init: np.ndarray, iterations: int = 8):
        pose = init.copy()
        points, normals = self.prior.points, self.prior.normals
        if len(points) == 0:
            return pose, 0, 1.0
        best_inliers, best_rmse = 0, 1.0
        for _ in range(iterations):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            rot = np.array([[c, -s], [s, c]])
            world = scan @ rot.T + pose[:2]
            dist, idx = self.prior.tree.query(world, k=1)
            mask = dist < 0.50
            if mask.sum() < 10:
                break
            p = world[mask]
            q = points[idx[mask]]
            n = normals[idx[mask]]
            # Point-to-line residual and analytic Jacobian.
            diff = p - q
            r = np.einsum("ij,ij->i", n, diff)
            px, py = scan[mask, 0], scan[mask, 1]
            jx = n[:, 0]
            jy = n[:, 1]
            jt = n[:, 0] * (-s * px - c * py) + n[:, 1] * (c * px - s * py)
            J = np.stack([jx, jy, jt], axis=1)
            # Robust Cauchy weights and a hard gate.
            scale = 0.09
            w = 1.0 / (1.0 + (r / scale) ** 2)
            w *= np.clip(1.0 - (dist[mask] - 0.35) / 0.15, 0.0, 1.0)
            H = J.T @ (J * w[:, None])
            g = J.T @ (r * w)
            H += np.diag([1e-3, 1e-3, 2e-3])
            try:
                delta = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                break
            if not np.all(np.isfinite(delta)):
                break
            delta[:2] = np.clip(delta[:2], -0.18, 0.18)
            delta[2] = np.clip(delta[2], -0.10, 0.10)
            pose[:2] += delta[:2]
            pose[2] = (pose[2] + delta[2] + math.pi) % (2 * math.pi) - math.pi
            inl = int(mask.sum())
            rmse = float(np.sqrt(np.mean(r ** 2)))
            if inl > best_inliers:
                best_inliers, best_rmse = inl, rmse
            if abs(delta[0]) + abs(delta[1]) + abs(delta[2]) < 1e-4:
                break
        return pose, best_inliers, best_rmse

    def restore_last_good(self) -> None:
        """Fall back to the last accepted scan pose before a global retry."""
        self.pose = self.last_good_pose.copy()
        self.status = "DEGRADED"

    def relocalize(self, scan_local: np.ndarray) -> bool:
        """Coarse global branch search followed by local ICP."""
        if len(scan_local) < 25:
            return False
        t0 = time.perf_counter()
        self.status = "RELOCALIZING"
        step = max(1, len(scan_local) // 60)
        sub = scan_local[::step]
        x0, x1, y0, y1 = self.prior.bounds
        res = 0.40
        xs = np.arange(x0 + 0.4, x1 - 0.4, res)
        ys = np.arange(y0 + 0.4, y1 - 0.4, res)
        best = []
        for k in range(24):
            yaw = -math.pi + k * 2 * math.pi / 24
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.array([[c, -s], [s, c]])
            local = sub @ rot.T
            for x in xs:
                for y in ys:
                    pts = local + (x, y)
                    ij = np.round((pts - np.array([x0, y0])) / self.prior.grid_res).astype(int)
                    ij[:, 0] = np.clip(ij[:, 0], 0, self.prior.distance_field.shape[0] - 1)
                    ij[:, 1] = np.clip(ij[:, 1], 0, self.prior.distance_field.shape[1] - 1)
                    d = self.prior.distance_field[ij[:, 0], ij[:, 1]]
                    score = float(np.mean(d < 0.22))
                    if score > 0.35:
                        best.append((score, x, y, yaw, float(np.mean(d))))
        best.sort(reverse=True)
        chosen = None
        prev = self.pose.copy()
        for score, x, y, yaw, _ in best[:10]:
            dx, dy = x - prev[0], y - prev[1]
            dyaw = abs((yaw - prev[2] + math.pi) % (2 * math.pi) - math.pi)
            # Prefer the hypothesis that continues the current track.  A
            # symmetric arena can otherwise look identical 180 deg away.
            if math.hypot(dx, dy) > 2.2 or dyaw > 0.75:
                continue
            solved, inl, rmse = self._icp(scan_local, np.array([x, y, yaw]), iterations=14)
            if inl < 35 or rmse > 0.11:
                continue
            jump = math.hypot(solved[0] - prev[0], solved[1] - prev[1])
            if jump > 2.2:
                continue
            if chosen is None or (inl - 20 * rmse) > (chosen[1] - 20 * chosen[2]):
                chosen = (solved, inl, rmse)
        if chosen is None:
            self.status = "LOST"
            return False
        self.pose = chosen[0]
        self.inliers = chosen[1]
        self.rmse = chosen[2]
        self.status = "TRACKING" if (chosen[1] >= 60 and chosen[2] < 0.105) else "DEGRADED"
        self.lost_since = None
        self.trajectory = [(float(self.pose[0]), float(self.pose[1]))]
        self.latency_ms = (time.perf_counter() - t0) * 1000.0
        return chosen[1] >= 45 and chosen[2] < 0.10
