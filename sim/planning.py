"""Occupancy-grid A* and pure-pursuit guidance for the S-Dog base."""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from .config import ROBOT_RADIUS

PASSABLE_PREFIXES = (
    "ground", "arena", "extra_area_floor", "start_zone", "extra_start_zone",
    "extra_storage", "extra_home", "gravel_", "wood_", "tire_ring", "L_pile",
    "stepping_stones_", "stairs_", "platform_", "ramp_", "bridge", "big_slope",
    "crossbar",
)
BLOCKING_PREFIXES = ("extra_wall", "extra_bar", "high_wall", "limit_frame", "pit_wall", "pit_separator")


@dataclass
class PathPlan:
    points: list[tuple[float, float]]
    length: float
    ok: bool = True


def _densify(points, step: float = 0.35):
    """Insert intermediate points so pure pursuit follows curves instead of
    cutting a three-point diagonal."""
    if len(points) < 2:
        return list(points)
    out = [tuple(points[0])]
    for a, b in zip(points, points[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(dist / step)))
        for k in range(1, n + 1):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


def _geom_aabb(model, data, gid):
    import mujoco

    gtype = model.geom_type[gid]
    pos = data.geom_xpos[gid]
    mat = data.geom_xmat[gid].reshape(3, 3)
    size = model.geom_size[gid]
    if gtype == mujoco.mjtGeom.mjGEOM_BOX:
        ext = np.abs(mat) @ size
        return pos - ext, pos + ext
    if gtype in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
        local = np.array([size[0], size[0], size[1]])
        ext = np.abs(mat) @ local
        return pos - ext, pos + ext
    if gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
        ext = np.array([size[0], size[0], size[0]])
        return pos - ext, pos + ext
    if gtype == mujoco.mjtGeom.mjGEOM_MESH:
        adr = model.mesh_vertadr[model.geom_dataid[gid]]
        count = model.mesh_vertnum[model.geom_dataid[gid]]
        verts = model.mesh_vert[adr:adr + count] @ mat.T + pos
        return verts.min(axis=0), verts.max(axis=0)
    return pos, pos


class OccupancyGrid:
    """Static field occupancy used by A*; cargo can be injected per plan."""

    def __init__(self, model, data, resolution: float = 0.10):
        import mujoco

        self.mujoco = mujoco
        self.res = resolution
        self.x0, self.x1 = -7.00, 14.00
        self.y0, self.y1 = -2.92, 2.92
        self.nx = int(round((self.x1 - self.x0) / resolution)) + 1
        self.ny = int(round((self.y1 - self.y0) / resolution)) + 1
        self.static = np.zeros((self.nx, self.ny), dtype=bool)
        self.inflate = int(math.ceil((ROBOT_RADIUS - 0.04) / resolution))
        self._build(model, data)

    def _build(self, model, data) -> None:
        for gid in range(model.ngeom):
            name = model.geom(gid).name or ""
            if model.geom_group[gid] != 0:
                continue
            top = float(data.geom_xpos[gid][2])
            if not name.startswith(BLOCKING_PREFIXES):
                continue
            lo, hi = _geom_aabb(model, data, gid)
            if hi[2] < 0.14:
                continue
            self._mark_rect(lo[0], lo[1], hi[0], hi[1])
        # The 0.7 m gaps between a bar end and the side wall are narrower than
        # the robot's footprint.  Seal them so A* uses the 1 m centre gate.
        self._mark_rect(11.45, -2.0, 11.75, -0.50)
        self._mark_rect(11.45, 0.50, 11.75, 2.0)
        self.static = self._inflate(self.static)

    def _mark_rect(self, xmin, ymin, xmax, ymax) -> None:
        i0 = max(0, int(np.floor((xmin - self.x0) / self.res)))
        i1 = min(self.nx - 1, int(np.ceil((xmax - self.x0) / self.res)))
        j0 = max(0, int(np.floor((ymin - self.y0) / self.res)))
        j1 = min(self.ny - 1, int(np.ceil((ymax - self.y0) / self.res)))
        if i1 >= i0 and j1 >= j0:
            self.static[i0:i1 + 1, j0:j1 + 1] = True

    def _inflate(self, grid: np.ndarray) -> np.ndarray:
        out = grid.copy()
        r = self.inflate
        if r <= 0:
            return out
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj > r * r:
                    continue
                src = grid[max(0, -di):self.nx - max(0, di), max(0, -dj):self.ny - max(0, dj)]
                dst = out[max(0, di):self.nx - max(0, -di), max(0, dj):self.ny - max(0, -dj)]
                dst |= src
        return out

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        i = int(round((x - self.x0) / self.res))
        j = int(round((y - self.y0) / self.res))
        return int(np.clip(i, 0, self.nx - 1)), int(np.clip(j, 0, self.ny - 1))

    def to_world(self, i: int, j: int) -> tuple[float, float]:
        return self.x0 + i * self.res, self.y0 + j * self.res

    def _free(self, i, j) -> bool:
        return 0 <= i < self.nx and 0 <= j < self.ny and not self.static[i, j]

    def _nearest_free(self, cell, grid):
        i, j = cell
        if not grid[i, j]:
            return i, j
        for radius in range(1, 20):
            for di in range(-radius, radius + 1):
                for dj in range(-radius, radius + 1):
                    if max(abs(di), abs(dj)) != radius:
                        continue
                    ni, nj = i + di, j + dj
                    if 0 <= ni < self.nx and 0 <= nj < self.ny and not grid[ni, nj]:
                        return ni, nj
        return i, j

    def plan(self, start, goal, cargo_obstacles=(), clear_start=True, clear_goal=True,
             extra_clearance: int = 0) -> PathPlan:
        grid = self.static.copy()
        for _ in range(max(0, int(extra_clearance))):
            grid = self._inflate(grid)
        for (x, y, radius) in cargo_obstacles:
            r = max(1, int(math.ceil(radius / self.res)))
            ci, cj = self.to_cell(x, y)
            i0, i1 = max(0, ci - r), min(self.nx - 1, ci + r)
            j0, j1 = max(0, cj - r), min(self.ny - 1, cj + r)
            ii, jj = np.mgrid[i0:i1 + 1, j0:j1 + 1]
            mask = (ii - ci) ** 2 + (jj - cj) ** 2 <= r * r
            grid[i0:i1 + 1, j0:j1 + 1] |= mask
        # Force a straight pass through the 1 m bar gate.  A smoothed diagonal
        # through the gate looks valid on the grid but makes the quadruped
        # swing its rear legs into the bar.
        if (start[0] - 11.6) * (goal[0] - 11.6) < 0:
            east, west, gate = (12.25, 0.0), (10.95, 0.0), (11.60, 0.0)
            pts = [tuple(start)]
            pts += [east, gate, west] if start[0] > 11.6 else [west, gate, east]
            pts.append(tuple(goal))
            if all(self.line_free(a, b, grid) for a, b in zip(pts, pts[1:])):
                length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
                pts = _densify(pts)
                return PathPlan(pts, length, True)
        s = self.to_cell(*start)
        g = self.to_cell(*goal)
        if clear_start:
            self._clear_disk(grid, s, 2)
        if clear_goal:
            self._clear_disk(grid, g, 3)
        s = self._nearest_free(s, grid)
        g = self._nearest_free(g, grid)
        if grid[s] or grid[g]:
            return PathPlan([tuple(start), tuple(goal)], math.hypot(goal[0] - start[0], goal[1] - start[1]), False)
        came: dict[tuple[int, int], tuple[int, int] | None] = {s: None}
        cost = {s: 0.0}
        heap = [(0.0, s)]
        dirs = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))
        found = False
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == g:
                found = True
                break
            for di, dj, step in dirs:
                ni, nj = cur[0] + di, cur[1] + dj
                if not (0 <= ni < self.nx and 0 <= nj < self.ny) or grid[ni, nj]:
                    continue
                if di and dj and (grid[cur[0] + di, cur[1]] or grid[cur[0], cur[1] + dj]):
                    continue
                ncost = cost[cur] + step
                if ncost < cost.get((ni, nj), 1e18):
                    cost[(ni, nj)] = ncost
                    came[(ni, nj)] = cur
                    h = math.hypot(ni - g[0], nj - g[1])
                    heapq.heappush(heap, (ncost + h, (ni, nj)))
        if not found:
            return PathPlan([tuple(start), tuple(goal)], math.hypot(goal[0] - start[0], goal[1] - start[1]), False)
        cells = []
        cur = g
        while cur is not None:
            cells.append(cur)
            cur = came[cur]
        cells.reverse()
        pts = [self.to_world(i, j) for i, j in cells]
        pts[0] = tuple(start)
        pts[-1] = tuple(goal)
        pts = _densify(self.smooth(pts, grid))
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        return PathPlan(pts, length, True)

    def _clear_disk(self, grid, cell, r):
        ci, cj = cell
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                ni, nj = ci + di, cj + dj
                if 0 <= ni < self.nx and 0 <= nj < self.ny:
                    grid[ni, nj] = False

    def smooth(self, pts, grid) -> list[tuple[float, float]]:
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1:
                if self.line_free(pts[i], pts[j], grid):
                    break
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def line_free(self, a, b, grid=None) -> bool:
        grid = self.static if grid is None else grid
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(dist / (self.res * 0.5)))
        for k in range(steps + 1):
            t = k / steps
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            i, j = self.to_cell(x, y)
            if not (0 <= i < self.nx and 0 <= j < self.ny) or grid[i, j]:
                return False
        return True

    def snapshot(self):
        return {
            "x0": self.x0, "x1": self.x1, "y0": self.y0, "y1": self.y1,
            "res": self.res, "nx": self.nx, "ny": self.ny,
            "blocked": [[int(i), int(j)] for i, j in zip(*np.where(self.static))],
        }


class PathFollower:
    def __init__(self):
        self.plan: PathPlan | None = None
        self.index = 0
        self._window_time = 0.0
        self._window_distance = 0.0
        self._window_remain = None
        self.stuck = False

    def set(self, plan: PathPlan):
        self.plan = plan
        self.index = 1 if len(plan.points) > 1 else 0
        self.reset_progress()

    def reset_progress(self):
        self._window_time = 0.0
        self._window_distance = 0.0
        self._window_remain = None
        self._last_position = None
        self.stuck = False

    def clear(self):
        self.plan = None
        self.index = 0

    def command(self, pose, lookahead=0.75, speed=0.62, fine_distance=0.55, dt=0.02):
        if self.plan is None or not self.plan.points:
            return np.zeros(3, dtype=np.float32), None, 0.0
        pts = self.plan.points
        x, y, yaw = pose
        # Advance past points already reached.
        while self.index < len(pts) - 1 and math.hypot(pts[self.index][0] - x, pts[self.index][1] - y) < 0.30:
            self.index += 1
        # Look ahead along the polyline.
        target = pts[-1]
        acc = 0.0
        prev = (x, y)
        for k in range(self.index, len(pts)):
            seg = math.hypot(pts[k][0] - prev[0], pts[k][1] - prev[1])
            if acc + seg >= lookahead or k == len(pts) - 1:
                target = pts[k]
                break
            acc += seg
            prev = pts[k]
        # Cross-track correction: hold the gate centreline instead of relying
        # only on heading.  Pure pursuit alone drifts into the bar posts.
        nearest = min(range(len(pts)), key=lambda k: (pts[k][0] - x) ** 2 + (pts[k][1] - y) ** 2)
        k0 = max(0, nearest - 1)
        k1 = min(len(pts) - 1, nearest + 1)
        ax, ay = pts[k0]
        bx, by = pts[k1]
        vx, vy = bx - ax, by - ay
        norm = math.hypot(vx, vy) or 1.0
        ux, uy = vx / norm, vy / norm
        ex, ey = x - pts[nearest][0], y - pts[nearest][1]
        lat_world = ux * ey - uy * ex
        lat_body = -math.sin(yaw) * ex + math.cos(yaw) * ey
        dx, dy = target[0] - x, target[1] - y
        dist = math.hypot(dx, dy)
        err = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
        remain = math.hypot(pts[-1][0] - x, pts[-1][1] - y)
        v = speed
        if remain < fine_distance:
            v = max(0.16, speed * remain / max(0.05, fine_distance))
        v *= max(0.12, 1.0 - abs(err) / 1.35)
        if abs(err) > 1.15:
            v = 0.08
        wz = float(np.clip(1.9 * err, -1.18, 1.18))
        vy_cmd = float(np.clip(-0.75 * lat_body, -0.30, 0.30)) * (1.0 if remain > 0.5 else 0.0)
        position = np.array([x, y], dtype=np.float64)
        if self._last_position is not None:
            self._window_distance += float(np.linalg.norm(position - self._last_position))
        self._last_position = position
        if abs(err) < 0.9 and remain > max(0.65, fine_distance) and v > 0.20:
            if self._window_remain is None:
                self._window_remain = remain
            self._window_time += dt
            if self._window_time >= 1.6:
                progress = self._window_remain - remain
                self.stuck = self._window_distance < 0.12 and progress < 0.10
                self._window_time = 0.0
                self._window_distance = 0.0
                self._window_remain = remain
        else:
            self.reset_progress()
            self._last_position = position
        return np.array([v, vy_cmd, wz], dtype=np.float32), target, remain

    @property
    def done(self) -> bool:
        return self.plan is None
