"""Cargo boxes: side-contact pickup, rigid carry, and zone deposition."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import CATEGORIES, HOME, STORAGE


@dataclass
class Cargo:
    index: int
    category: str
    initial: tuple[float, float, float]
    body_id: int
    geom_id: int
    joint_id: int
    qpos_adr: int
    dof_adr: int
    side: int = 0                 # +1 left, -1 right, 0 free
    storage_departed: bool = False
    deposited_zone: str | None = None
    deposited_full: bool = False
    stack_upper: bool = False


class CargoManager:
    """Owns the eight 200 mm cargo boxes and the simplified manipulation model.

    The challenge does not require a manipulation arm: a robot flank touching
    a box is treated as a grasp, and at most one box may be carried per flank.
    Carried boxes are kinematically locked to the base and their collision is
    disabled to avoid solver explosions from an intersecting grasp volume.
    """

    def __init__(self, model, data):
        import mujoco

        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.cargo: list[Cargo] = []
        for i, (x, y, category) in enumerate(STORAGE):
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"block_{i}")
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"block_{i}_geom")
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"block_{i}_joint")
            self.cargo.append(Cargo(
                index=i, category=category, initial=(x, y, 0.101),
                body_id=body, geom_id=geom, joint_id=joint,
                qpos_adr=int(model.jnt_qposadr[joint]),
                dof_adr=int(model.jnt_dofadr[joint]),
            ))
        self.base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self._lock_collision = {}
        self.reset()

    # ------------------------------------------------------------------ state
    @property
    def carried(self) -> list[Cargo]:
        return [c for c in self.cargo if c.side != 0]

    @property
    def carried_count(self) -> int:
        return sum(1 for c in self.cargo if c.side != 0)

    def free_side(self) -> int:
        for side in (1, -1):
            if self.side_available(side):
                return side
        return 0

    def side_available(self, side: int) -> bool:
        return not any(c.side == side for c in self.cargo)

    def position(self, c: Cargo) -> np.ndarray:
        return self.data.xpos[c.body_id].copy()

    def category_remaining(self, category: str | None = None) -> list[Cargo]:
        out = []
        for c in self.cargo:
            if c.side == 0 and not c.deposited_zone:
                if category is None or c.category == category:
                    out.append(c)
        return out

    def carried_of(self, category: str) -> list[Cargo]:
        return [c for c in self.cargo if c.side != 0 and c.category == category]

    # ------------------------------------------------------------------ reset
    def reset(self) -> None:
        for c in self.cargo:
            self._collision(c, True)
            c.side = 0
            c.storage_departed = False
            c.deposited_zone = None
            c.deposited_full = False
            c.stack_upper = False
            self.data.qpos[c.qpos_adr:c.qpos_adr + 3] = c.initial
            self.data.qpos[c.qpos_adr + 3:c.qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
            self.data.qvel[c.dof_adr:c.dof_adr + 6] = 0.0
        self.mujoco.mj_forward(self.model, self.data)

    def _collision(self, c: Cargo, enabled: bool) -> None:
        self.model.geom_contype[c.geom_id] = 4 if enabled else 0
        self.model.geom_conaffinity[c.geom_id] = 4 if enabled else 0
        self.model.geom_group[c.geom_id] = 0 if enabled else 2

    # ----------------------------------------------------------------- pickup
    def try_pickup(self, base_pos: np.ndarray, yaw: float, max_carry: int = 2, target_index: int | None = None) -> Cargo | None:
        """Attach a box that is touching a free robot flank (optionally only the target)."""
        if self.carried_count >= max_carry:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        for cargo in self.cargo:
            if cargo.side != 0 or cargo.deposited_zone:
                continue
            if target_index is not None and cargo.index != target_index:
                continue
            p = self.position(cargo)
            if p[2] < -0.2 or p[2] > 0.42:
                continue
            dx, dy = p[0] - base_pos[0], p[1] - base_pos[1]
            lx = c * dx + s * dy
            ly = -s * dx + c * dy
            if abs(lx) > 0.46:
                continue
            if not (0.12 < abs(ly) < 0.72):
                continue
            side = 1 if ly > 0 else -1
            if not self.side_available(side):
                continue
            cargo.storage_departed = True
            cargo.side = side
            self._collision(cargo, False)
            return cargo
        return None

    def sync_carried(self) -> None:
        """Called before every physics substep while boxes are carried."""
        # xpos/xquat already contains the forward kinematics for this step.
        pos = self.data.xpos[self.base_id]
        quat = self.data.xquat[self.base_id]
        qw, qx, qy, qz = quat
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        c, s = math.cos(yaw), math.sin(yaw)
        for cargo in self.cargo:
            if cargo.side == 0:
                continue
            off_y = cargo.side * 0.34
            x = pos[0] - s * off_y
            y = pos[1] + c * off_y
            z = max(0.18, pos[2] - 0.02)
            self.data.qpos[cargo.qpos_adr:cargo.qpos_adr + 3] = [x, y, z]
            self.data.qpos[cargo.qpos_adr + 3:cargo.qpos_adr + 7] = quat
            self.data.qvel[cargo.dof_adr:cargo.dof_adr + 6] = 0.0

    # --------------------------------------------------------------- delivery
    def zone_occupants(self, category: str, max_distance: float = 0.32) -> list[Cargo]:
        cx, cy = HOME[category]
        out = []
        for c in self.cargo:
            if c.side != 0 or c.category != category:
                continue
            p = self.position(c)
            if math.hypot(p[0] - cx, p[1] - cy) < max_distance:
                out.append(c)
        return out

    def deposit(self, category: str) -> list[Cargo]:
        """Place all carried boxes of ``category`` into its home cell."""
        cx, cy = HOME[category]
        existing = [c for c in self.zone_occupants(category) if self.position(c)[2] < 0.22]
        level = 0 if not existing else len(existing)
        out = []
        for cargo in [c for c in self.cargo if c.side != 0 and c.category == category]:
            self._collision(cargo, True)
            cargo.side = 0
            z = 0.101 + 0.20 * level
            self.data.qpos[cargo.qpos_adr:cargo.qpos_adr + 3] = [cx, cy, z]
            self.data.qpos[cargo.qpos_adr + 3:cargo.qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
            self.data.qvel[cargo.dof_adr:cargo.dof_adr + 6] = 0.0
            cargo.deposited_zone = category
            cargo.stack_upper = level > 0
            cargo.deposited_full = True
            level += 1
            out.append(cargo)
        return out

    # --------------------------------------------------------------- telemetry
    def snapshot(self) -> list[dict]:
        out = []
        for c in self.cargo:
            p = self.position(c)
            if c.side != 0:
                status = "carried"
            elif c.deposited_zone:
                status = "stack" if c.stack_upper else "home"
            elif math.hypot(p[0] - c.initial[0], p[1] - c.initial[1]) > 0.16:
                status = "loose"
            else:
                status = "storage"
            out.append({
                "id": c.index,
                "category": c.category,
                "status": status,
                "side": c.side,
                "pos": [round(float(v), 3) for v in p],
                "storage_departed": c.storage_departed,
            })
        return out
