"""S-Dog TorchScript policy, observation assembly, and PD actuation."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Callable

import numpy as np
import torch

JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]
STAND = np.array([-0.1, -0.9, 1.8, 0.1, 0.9, -1.8, 0.1, -0.9, 1.8, -0.1, 0.9, -1.8], dtype=np.float32)
ACTION_SCALE = 0.25
OBS_COMMAND_SCALE = np.array([0.5, 1.0, 1.0], dtype=np.float32)
ANGULAR_SCALE = 0.25
JOINT_VEL_SCALE = 0.05
KP = 25.0
KD = 0.5
CONTROL_DT = 0.02
SIM_DT = 0.002
DECIMATION = 10


def quat_to_yaw(q: np.ndarray) -> float:
    qw, qx, qy, qz = q
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def projected_gravity(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = q
    return np.array([
        2.0 * (-qz * qx + qw * qy),
        -2.0 * (qz * qy + qw * qx),
        1.0 - 2.0 * (qw * qw + qz * qz),
    ], dtype=np.float32)


def velocity_body(qvel: np.ndarray, quat: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = quat
    xyz = np.array([qx, qy, qz], dtype=np.float32)
    v = np.asarray(qvel[:3], dtype=np.float32)
    return v * (2.0 * qw * qw - 1.0) - 2.0 * qw * np.cross(xyz, v) + 2.0 * xyz * np.dot(xyz, v)


def pd_control(target: np.ndarray, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
    """Position PD with the RS02 torque-speed envelope used by sim2sim."""
    effort = KP * (target - position) - KD * velocity
    same_direction = velocity * effort > 0.0
    maximum = np.where(same_direction, 17.0, 17.0)
    ramp = maximum * (1.0 - (np.abs(velocity) - 20.944) / (42.935 - 20.944))
    maximum = np.where(np.abs(velocity) < 20.944, maximum, np.maximum(ramp, 0.0))
    return np.clip(effort, -maximum, maximum)


class SdogPolicy:
    """Thin sim2sim driver for the bundled 100K mixed-terrain student policy."""

    def __init__(self, model, data, policy_path: Path):
        import mujoco

        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.policy = torch.jit.load(str(policy_path), map_location="cpu").eval()
        self.root_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        self.qpos_adr = int(model.jnt_qposadr[self.root_joint])
        self.dof_adr = int(model.jnt_dofadr[self.root_joint])
        self.base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.joint_ids = []
        self.actuator_ids = []
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.removesuffix("_joint"))
            if jid < 0 or aid < 0:
                raise ValueError(f"missing S-Dog joint/actuator {name}")
            self.joint_ids.append(jid)
            self.actuator_ids.append(aid)
        self.joint_ids = np.asarray(self.joint_ids, dtype=np.int32)
        self.actuator_ids = np.asarray(self.actuator_ids, dtype=np.int32)
        self.qpos_ids = np.asarray([model.jnt_qposadr[j] for j in self.joint_ids], dtype=np.int32)
        self.qvel_ids = np.asarray([model.jnt_dofadr[j] for j in self.joint_ids], dtype=np.int32)
        self.action = np.zeros(12, dtype=np.float32)
        self.obs = np.zeros(45, dtype=np.float32)
        self.obs_tensor = torch.zeros((1, 45), dtype=torch.float32)
        self.belly_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.default = STAND.copy()
        self._last_yaw = None

    def reset(self, spawn: tuple[float, float, float], yaw: float = 0.0) -> None:
        data = self.data
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qpos[self.qpos_adr:self.qpos_adr + 7] = [
            spawn[0], spawn[1], spawn[2],
            math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0),
        ]
        data.qpos[self.qpos_ids] = self.default
        self.action[:] = 0.0
        self.obs[:] = 0.0
        self._last_yaw = None
        if hasattr(self.policy, "reset"):
            self.policy.reset()
        self.mujoco.mj_forward(self.model, data)

    def infer_step(self, command: np.ndarray, pre_step: Callable[[], None] | None = None) -> None:
        data = self.data
        self.obs[0:3] = data.qvel[self.dof_adr + 3:self.dof_adr + 6] * ANGULAR_SCALE
        self.obs[3:6] = projected_gravity(data.qpos[self.qpos_adr + 3:self.qpos_adr + 7])
        self.obs[6:9] = np.asarray(command, dtype=np.float32) * OBS_COMMAND_SCALE
        self.obs[9:21] = data.qpos[self.qpos_ids] - self.default
        self.obs[21:33] = data.qvel[self.qvel_ids] * JOINT_VEL_SCALE
        self.obs[33:45] = self.action
        self.obs_tensor[0].copy_(torch.from_numpy(self.obs))
        with torch.inference_mode():
            self.action = self.policy(self.obs_tensor).cpu().numpy().reshape(-1).astype(np.float32)
        if self.action.shape != (12,) or not np.all(np.isfinite(self.action)):
            raise RuntimeError("policy returned an invalid action")
        self.action = np.clip(self.action, -100.0, 100.0)
        target = self.default + ACTION_SCALE * self.action
        for _ in range(DECIMATION):
            if pre_step is not None:
                pre_step()
            effort = pd_control(target, data.qpos[self.qpos_ids], data.qvel[self.qvel_ids])
            data.ctrl[self.actuator_ids] = effort
            self.mujoco.mj_step(self.model, data)

    def pose(self) -> tuple[np.ndarray, np.ndarray, float]:
        data = self.data
        pos = data.qpos[self.qpos_adr:self.qpos_adr + 3].copy()
        quat = data.qpos[self.qpos_adr + 3:self.qpos_adr + 7].copy()
        return pos, quat, quat_to_yaw(quat)

    def body_velocity(self) -> np.ndarray:
        pos, quat, _ = self.pose()
        return velocity_body(self.data.qvel[self.dof_adr:self.dof_adr + 3], quat)

    def world_velocity(self) -> np.ndarray:
        return self.data.qvel[self.dof_adr:self.dof_adr + 3].copy()

    def yaw_rate(self) -> float:
        return float(self.data.qvel[self.dof_adr + 5])

    def yaw_rate_world(self) -> float:
        """Yaw rate from the IMU attitude, valid under roll/pitch motion."""
        _, _, yaw = self.pose()
        if self._last_yaw is None:
            self._last_yaw = yaw
            return 0.0
        dyaw = (yaw - self._last_yaw + math.pi) % (2 * math.pi) - math.pi
        self._last_yaw = yaw
        return float(dyaw / 0.02)

    def fallen(self) -> bool:
        pos, quat, _ = self.pose()
        return bool(pos[2] < 0.12 or abs(projected_gravity(quat)[2]) < 0.45)

    def tilt(self) -> float:
        _, quat, _ = self.pose()
        return float(math.degrees(math.acos(np.clip(abs(projected_gravity(quat)[2]), -1.0, 1.0))))

    def base_forward(self) -> np.ndarray:
        _, quat, yaw = self.pose()
        return np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
