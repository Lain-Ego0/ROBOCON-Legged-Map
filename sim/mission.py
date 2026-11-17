"""Block-logistics decision tree, task state machines, and rule-book scoring."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import (
    CARRY_LIMIT, CATEGORIES, CATEGORY_ZH, HOME, MISSION_SECONDS,
    PENALTY_WRONG_ZONE, SCORE_BAR, SCORE_HOME_FULL, SCORE_HOME_PARTIAL,
    SCORE_STACK, SCORE_STORAGE, START_LOGISTICS, TREE,
)
from .planning import OccupancyGrid, PathFollower, PathPlan


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _pose_dict(p):
    return {"x": round(float(p[0]), 3), "y": round(float(p[1]), 3), "yaw": round(float(p[2]), 3)}


class RuleScoring:
    def __init__(self):
        self.bar_crossed = False

    def reset(self):
        self.bar_crossed = False

    def evaluate(self, cargo, robot_z: float = 0.0):
        items = []
        total = 0.0
        zones = {c: [] for c in CATEGORIES}
        for c in cargo.cargo:
            p = cargo.position(c)
            if c.side != 0:
                continue
            for cat, (zx, zy) in HOME.items():
                if abs(p[0] - zx) < 0.30 and abs(p[1] - zy) < 0.30:
                    zones[cat].append((c, p))
                    break
        for c in cargo.cargo:
            p = cargo.position(c)
            storage = SCORE_STORAGE if (c.storage_departed or math.hypot(p[0] - c.initial[0], p[1] - c.initial[1]) > 0.16) else 0
            home = 0
            stack = 0
            penalty = 0
            zone = None
            for cat, (zx, zy) in HOME.items():
                if abs(p[0] - zx) < 0.27 and abs(p[1] - zy) < 0.27:
                    zone = cat
                    break
            if c.side == 0 and zone is not None:
                zx, zy = HOME[zone]
                if zone == c.category:
                    dx, dy = abs(p[0] - zx), abs(p[1] - zy)
                    if p[2] > 0.22:
                        below = [o for o, op in zones[zone] if o is not c and o.category == c.category and op[2] < 0.22 and math.hypot(op[0] - p[0], op[1] - p[1]) < 0.16]
                        if below:
                            stack = SCORE_STACK
                    elif dx <= 0.055 and dy <= 0.055:
                        home = SCORE_HOME_FULL
                    elif dx < 0.25 and dy < 0.25:
                        home = SCORE_HOME_PARTIAL
                else:
                    penalty = PENALTY_WRONG_ZONE
            points = storage + home + stack - penalty
            total += points
            items.append({
                "id": c.index, "category": c.category, "storage": storage,
                "home": home, "stack": stack, "penalty": penalty,
                "points": points, "zone": zone,
            })
        bar = SCORE_BAR if self.bar_crossed else 0
        total += bar
        return {
            "total": int(round(total)),
            "battle": int(round(total)),
            "bar": bar,
            "items": items,
            "max": 1900,
        }


@dataclass
class TaskState:
    name: str
    state: str = "idle"
    target: int | None = None
    side: int = 0
    direction: int = 1
    approach: tuple[float, float] | None = None
    points: list[tuple[float, float]] = field(default_factory=list)
    elapsed: float = 0.0
    timeout: float = 14.0
    note: str = ""
    started: bool = False


class PickTask:
    def __init__(self, mission, cargo_id: int):
        self.m = mission
        self.cargo = mission.cargo.cargo[cargo_id]
        self.t = TaskState("pick", target=cargo_id, timeout=22.0)
        self.follower = PathFollower()
        self.retry_side = 0
        self.sweep_elapsed = 0.0
        self.phase_elapsed = 0.0
        self.plan_timer = 0.0
        self._dbg_t = -10.0
        self._initial_side()

    def _initial_side(self):
        p = self.m.cargo.position(self.cargo)
        # Put the box on the field-centre side so the robot is not pinned to a wall.
        self.t.side = -1 if p[1] <= 0.0 else 1
        self._compute_approach()

    def _compute_approach(self):
        p = self.m.cargo.position(self.cargo)
        # Approach from the west for both rows.  Cargo is collision-transparent
        # to the robot, so a first-row box never wedges the approach.
        self.t.direction = 1
        start_x = p[0] - 0.68
        lane_y = p[1] - self.t.side * 0.33
        start_x = float(np.clip(start_x, 8.62, 11.18))
        lane_y = float(np.clip(lane_y, -1.82, 1.82))
        self.t.approach = (start_x, lane_y)
        obs = [(self.m.cargo.position(o)[0], self.m.cargo.position(o)[1], 0.05) for o in self.m.cargo.cargo if o.side == 0 and o.index != self.cargo.index]
        self.t.points = self.m.grid.plan(self.m.pose[:2], self.t.approach, ()).points
        self.follower.set(PathPlan(self.t.points, 0.0))

    def tick(self, dt: float) -> np.ndarray:
        self.t.elapsed += dt
        if self.cargo.side != 0:
            self.t.state = "done"
            return np.zeros(3, dtype=np.float32)
        if self.t.elapsed > self.t.timeout:
            self.t.state = "failed"
            self.t.note = "取箱超时"
            return np.zeros(3, dtype=np.float32)
        if self.t.state == "idle":
            self.t.state = "approach"
            self.t.note = "前往箱体侧方"
            self.m.log(f"取箱 #{self.cargo.index} 接近点 {self.t.approach} 侧 {self.t.side} 方向 {self.t.direction}", "info")
        if self.t.state == "approach":
            if self.t.elapsed - self._dbg_t > 4.0:
                self._dbg_t = self.t.elapsed
                self.m.log(f"接近 #{self.cargo.index}: pos ({self.m.pose[0]:.2f},{self.m.pose[1]:.2f}) 目标 {self.t.approach}", "info")
            cmd, target, remain = self.follower.command(self.m.pose, lookahead=0.75, speed=0.48, fine_distance=0.45)
            if remain < 0.22:
                self.t.state = "align"
                self.t.note = "对准取箱方向"
                self.phase_elapsed = 0.0
                self.m.log(f"到达接近点 #{self.cargo.index} ({self.m.pose[0]:.2f},{self.m.pose[1]:.2f})", "info")
            return cmd
        if self.t.state == "align":
            goal_yaw = 0.0 if self.t.direction > 0 else math.pi
            err = _wrap(goal_yaw - self.m.pose[2])
            self.phase_elapsed += dt
            cmd = np.array([0.0, 0.0, float(np.clip(2.2 * err, -0.9, 0.9))], dtype=np.float32)
            if abs(err) < 0.12 or self.phase_elapsed > 2.0:
                self.t.state = "sweep"
                self.t.note = "侧向扫掠 · 接触取物"
                self.sweep_elapsed = 0.0
                self.m.log(f"开始扫掠 #{self.cargo.index} yaw {self.m.pose[2]:.2f}", "info")
            return cmd
        if self.t.state == "sweep":
            goal_yaw = 0.0 if self.t.direction > 0 else math.pi
            err = _wrap(goal_yaw - self.m.pose[2])
            cmd = np.array([0.42 * self.t.direction, 0.0, float(np.clip(1.4 * err, -0.45, 0.45))], dtype=np.float32)
            self.sweep_elapsed += dt
            p = self.m.cargo.position(self.cargo)
            if math.hypot(p[0] - self.m.pose[0], p[1] - self.m.pose[1]) < 0.58 and p[2] < 0.40:
                # The cargo manager attaches on contact; slow down to guarantee it.
                cmd[0] *= 0.6
            if self.sweep_elapsed > 4.2:
                self.t.state = "retry"
            return cmd
        if self.t.state == "retry":
            if self.retry_side == 0:
                self.retry_side = 1
                p = self.m.cargo.position(self.cargo)
                # Never retry on the wall side of a boundary row.
                if abs(p[1]) < 1.2:
                    self.t.side = -self.t.side
                self._compute_approach()
                self.t.state = "approach"
                self.t.note = "换侧重试"
                self.t.elapsed = 0.0
                return np.zeros(3, dtype=np.float32)
            self.t.state = "failed"
            self.t.note = "未检测到侧面接触"
            return np.zeros(3, dtype=np.float32)
        return np.zeros(3, dtype=np.float32)

    @property
    def done(self):
        return self.t.state in ("done", "failed")


class DeliverTask:
    def __init__(self, mission, category: str):
        self.m = mission
        self.category = category
        self.t = TaskState("deliver", timeout=24.0)
        self.follower = PathFollower()
        self.deposit_timer = 0.0
        zx, zy = HOME[category]
        self.zone = (zx, zy)
        self.t.approach = (zx - 0.57, zy)
        obs = [(self.m.cargo.position(o)[0], self.m.cargo.position(o)[1], 0.16) for o in mission.cargo.cargo if o.side == 0]
        plan = mission.grid.plan(mission.pose[:2], self.t.approach, ())
        self.t.points = plan.points
        self.follower.set(plan)

    def tick(self, dt: float) -> np.ndarray:
        self.t.elapsed += dt
        if self.m.cargo.carried_count == 0:
            self.t.state = "done"
            return np.zeros(3, dtype=np.float32)
        if self.t.elapsed > self.t.timeout:
            self.t.state = "failed"
            self.t.note = "投放超时"
            return np.zeros(3, dtype=np.float32)
        if self.t.state == "idle":
            self.t.state = "approach"
            self.t.note = "前往归位区"
        if self.t.state == "approach":
            cmd, target, remain = self.follower.command(self.m.pose, lookahead=0.70, speed=0.45, fine_distance=0.45)
            d_zone = math.hypot(self.m.pose[0] - self.zone[0], self.m.pose[1] - self.zone[1])
            if remain < 0.22 or d_zone < 0.72:
                self.t.state = "deposit"
                self.deposit_timer = 0.0
                self.t.note = "投放 · 叠放"
                cmd = np.zeros(3, dtype=np.float32)
            return cmd
        if self.t.state == "deposit":
            self.m.cargo.deposit(self.category)
            self.deposit_timer += dt
            if self.deposit_timer > 0.8:
                self.t.state = "retreat"
                self.retreat_timer = 0.0
            return np.zeros(3, dtype=np.float32)
        if self.t.state == "retreat":
            self.retreat_timer += dt
            if self.retreat_timer > 1.8:
                self.t.state = "done"
                return np.zeros(3, dtype=np.float32)
            return np.array([-0.32, 0.0, 0.0], dtype=np.float32)
        return np.zeros(3, dtype=np.float32)

    @property
    def done(self):
        return self.t.state in ("done", "failed")


class ReturnTask:
    def __init__(self, mission):
        self.m = mission
        self.t = TaskState("return", timeout=25.0)
        self.follower = PathFollower()
        plan = mission.grid.plan(mission.pose[:2], mission.start_xy)
        self.follower.set(plan)
        self.t.state = "approach"
        self.t.note = "返回启动区"

    def tick(self, dt):
        self.t.elapsed += dt
        cmd, _, remain = self.follower.command(self.m.pose, lookahead=0.65, speed=0.55, fine_distance=0.35)
        if remain < 0.25 or self.t.elapsed > self.t.timeout:
            self.t.state = "done"
            self.m.finished = True
            return np.zeros(3, dtype=np.float32)
        return cmd

    @property
    def done(self):
        return self.t.state == "done"


class MissionController:
    """Reactive decision tree on top of three task state machines."""

    def __init__(self, cargo, grid: OccupancyGrid, scoring: RuleScoring, start=(7.6, 0.0)):
        self.cargo = cargo
        self.grid = grid
        self.scoring = scoring
        self.start_xy = tuple(start)
        self.task = None
        self.pose = np.zeros(3, dtype=np.float64)
        self.mission_time = 0.0
        self.started = False
        self.finished = False
        self.decision_path: list[str] = ["root"]
        self.reason = "等待启动"
        self.action = "idle"
        self.node_states: dict[str, str] = {}
        self.logs: list[dict] = []
        self.pick_failures: dict[int, float] = {}
        self.last_decision_time = -1.0
        self.bar_crossed = False
        self.bar_latch = False
        self.carried_prev = 0
        self.delivered_total = 0
        self._last_log = ""
        self._status_text = "待机"
        self.m_robot = None

    # ------------------------------------------------------------------ logs
    def log(self, text: str, kind: str = "info"):
        if text == self._last_log:
            return
        self._last_log = text
        self.logs.append({"t": round(self.mission_time, 1), "kind": kind, "text": text})
        self.logs = self.logs[-40:]

    def reset(self, start=(7.6, 0.0)):
        self.start_xy = tuple(start)
        self.task = None
        self.pose[:] = 0.0
        self.mission_time = 0.0
        self.started = False
        self.finished = False
        self.decision_path = ["root"]
        self.reason = "等待启动"
        self.action = "idle"
        self.logs = []
        self.pick_failures = {}
        self.bar_crossed = False
        self.bar_latch = False
        self.carried_prev = 0
        self.delivered_total = 0
        self._status_text = "待机"

    def start(self):
        self.started = True
        self.finished = False
        self.task = None
        self.log("自动任务启动 · 决策树上线", "start")

    # -------------------------------------------------------------- decisions
    def _zone_capacity(self, category: str) -> int:
        zx, zy = HOME[category]
        n = 0
        for c in self.cargo.cargo:
            if c.category != category or c.side != 0:
                continue
            p = self.cargo.position(c)
            if math.hypot(p[0] - zx, p[1] - zy) < 0.28 and p[2] < 0.5:
                n += 1
        n += len([c for c in self.cargo.cargo if c.side != 0 and c.category == category])
        return n

    def _choose_category(self):
        best = None
        for cat in CATEGORIES:
            remaining = [c for c in self.cargo.cargo if c.side == 0 and c.category == cat and not c.deposited_zone
                         and self.mission_time - self.pick_failures.get(c.index, -999.0) > 18.0]
            if not remaining or self._zone_capacity(cat) >= 2:
                continue
            nearest = min(remaining, key=lambda c: self._dist_to(c))
            score = 2.2 * min(2, len(remaining)) - 0.55 * self._dist_to(nearest) - (0.6 if self._zone_capacity(cat) >= 1 else 0.0)
            if best is None or score > best[0]:
                best = (score, cat, nearest)
        return best

    def _dist_to(self, c):
        p = self.cargo.position(c)
        return math.hypot(p[0] - self.pose[0], p[1] - self.pose[1])

    def _decide(self, dt: float) -> str:
        self.node_states = {}
        carried = self.cargo.carried
        left = MISSION_SECONDS - self.mission_time
        path = ["root"]
        # Safety guard first.
        if self.m_robot is not None and self.m_robot.fallen():
            self.node_states = {"guard": "active", "recover": "active"}
            self.decision_path = ["root", "guard", "recover"]
            self.reason = "姿态异常 · 安全恢复"
            self.action = "recover"
            return "recover"
        if left < 16 and len(carried) == 0:
            self.node_states = {"guard": "active", "return": "active"}
            self.decision_path = ["root", "guard", "return"]
            self.reason = "剩余时间不足 · 返回启动区"
            self.action = "return"
            return "return"
        if len(carried) >= CARRY_LIMIT:
            self.node_states = {"full": "active", "deliver": "active"}
            self.decision_path = ["root", "full", "deliver"]
            self.reason = "已满载 2 箱 · 前往归位区"
            self.action = "deliver"
            return "deliver"
        if len(carried) == 1:
            cat = carried[0].category
            remaining = [c for c in self.cargo.cargo if c.side == 0 and c.category == cat and not c.deposited_zone
                         and self.mission_time - self.pick_failures.get(c.index, -999.0) > 18.0]
            can_pair = bool(remaining) and self._zone_capacity(cat) < 2
            if can_pair:
                self.node_states = {"full": "active", "pair": "active", "pick_same": "active"}
                self.decision_path = ["root", "full", "pair", "pick_same"]
                self.reason = f"已载 {CATEGORY_ZH[cat]} · 优先配同类别第二箱"
                self.action = "pick_same"
                return "pick_same"
            self.node_states = {"full": "active", "pair": "active", "deliver_one": "active"}
            self.decision_path = ["root", "full", "pair", "deliver_one"]
            self.reason = f"无同类别可配对 · 先投放 {CATEGORY_ZH[cat]}"
            self.action = "deliver_one"
            return "deliver_one"
        available = [c for c in self.cargo.category_remaining()
                     if self.mission_time - self.pick_failures.get(c.index, -999.0) > 18.0]
        if available:
            self.node_states = {"empty": "active", "pick_new": "active"}
            self.decision_path = ["root", "empty", "pick_new"]
            self.reason = "空载 · 选择最优类别取箱"
            self.action = "pick_new"
            return "pick_new"
        self.node_states = {"empty": "active", "done": "active"}
        self.decision_path = ["root", "empty", "done"]
        self.reason = "物资全部处理完毕 · 返回启动区"
        self.action = "done"
        return "done"

    # ------------------------------------------------------------------- tick
    def step(self, pose: np.ndarray, robot, dt: float) -> np.ndarray:
        self.pose = np.asarray(pose, dtype=np.float64).copy()
        self.m_robot = robot
        if not self.started or self.finished:
            return np.zeros(3, dtype=np.float32)
        self.mission_time += dt
        if self.mission_time >= MISSION_SECONDS:
            self.finished = True
            self.log("180 s 比赛时间到", "end")
            return np.zeros(3, dtype=np.float32)
        # Bar-crossing bonus (only when actually above the bar).
        if not self.bar_latch and 11.5 < pose[0] < 11.75 and 0.5 < abs(pose[1]) < 1.35 and robot.pose()[0][2] > 0.55:
            self.bar_crossed = True
            self.bar_latch = True
            self.log("首次跨越挡车杆 +100", "score")
        if self.task is None or self.task.done:
            if self.task is not None:
                if getattr(self.task.t, "state", "") == "failed":
                    self.log(f"任务失败: {self.task.t.note}", "warn")
                    if isinstance(self.task, PickTask):
                        self.pick_failures[self.task.t.target] = self.mission_time
                self.task = None
            action = self._decide(dt)
            self._status_text = self.reason
            if action in ("deliver", "deliver_one"):
                carried = self.cargo.carried
                cat = carried[0].category if carried else "food"
                self.task = DeliverTask(self, cat)
                self.log(f"决策 → 投放 {CATEGORY_ZH[cat]} 至 {HOME[cat]}", "decision")
            elif action in ("pick_new", "pick_same"):
                if action == "pick_same" and self.cargo.carried:
                    cat = self.cargo.carried[0].category
                    candidates = [c for c in self.cargo.cargo if c.side == 0 and c.category == cat and not c.deposited_zone
                                  and self.mission_time - self.pick_failures.get(c.index, -999.0) > 18.0]
                    target = min(candidates, key=self._dist_to) if candidates else None
                else:
                    chosen = self._choose_category()
                    target = chosen[2] if chosen else None
                if target is None:
                    self.finished = True
                    self.log("没有可达物资 · 任务结束", "end")
                    return np.zeros(3, dtype=np.float32)
                # Avoid repeatedly failing on the same box.
                if self.mission_time - self.pick_failures.get(target.index, -999) < 8.0:
                    others = [c for c in self.cargo.category_remaining() if c.index != target.index]
                    if others:
                        target = min(others, key=self._dist_to)
                self.task = PickTask(self, target.index)
                self.log(f"决策 → 取箱 #{target.index} {CATEGORY_ZH[target.category]}", "decision")
            elif action == "return":
                self.task = ReturnTask(self)
                self.log("决策 → 返回启动区", "decision")
            elif action == "recover":
                self.task = None
                self.log("等待姿态恢复", "warn")
                return np.zeros(3, dtype=np.float32)
            elif action == "done":
                self.task = ReturnTask(self)
                self.log("决策 → 返回启动区", "decision")
            else:
                return np.zeros(3, dtype=np.float32)
        cmd = self.task.tick(dt) if self.task is not None else np.zeros(3, dtype=np.float32)
        if self.cargo.carried_count != self.carried_prev:
            if self.cargo.carried_count > self.carried_prev:
                self.log(f"侧面接触 · 取物成功 ({self.cargo.carried_count}/2)", "pickup")
            self.carried_prev = self.cargo.carried_count
        return cmd

    # --------------------------------------------------------------- telemetry
    def telemetry(self):
        task_info = None
        if self.task is not None:
            task_info = {
                "kind": self.task.t.name,
                "state": self.task.t.state,
                "note": self.task.t.note,
                "target": self.task.t.target,
                "elapsed": round(self.task.t.elapsed, 1),
                "timeout": self.task.t.timeout,
            }
        return {
            "started": self.started,
            "finished": self.finished,
            "time": round(self.mission_time, 2),
            "time_left": round(max(0.0, MISSION_SECONDS - self.mission_time), 1),
            "decision_path": self.decision_path,
            "node_states": self.node_states,
            "reason": self.reason,
            "action": self.action,
            "status": self._status_text,
            "task": task_info,
            "logs": self.logs[-14:],
            "bar_crossed": self.bar_crossed,
            "carried": self.cargo.carried_count,
            "carry_limit": CARRY_LIMIT,
        }
