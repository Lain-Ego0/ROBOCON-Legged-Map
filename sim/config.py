"""Shared constants, rule-book geometry, and the policy decision tree."""
from __future__ import annotations

from pathlib import Path

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"
SCENE = ASSETS / "scene.xml"
POLICY = ASSETS / "sdog_mixed_moe_cts_100k.pt"
WEB = HERE / "web"

HOST = "127.0.0.1"
PORT = 8731

# Rule-book cargo classes (食品 / 工具 / 仪器 / 药品).
CATEGORIES = ("food", "tool", "instrument", "medicine")
CATEGORY_ZH = {
    "food": "食品箱",
    "tool": "工具箱",
    "instrument": "仪器箱",
    "medicine": "药品箱",
}
CATEGORY_COLORS = {
    "food": "#19c46a",
    "tool": "#9aa7b6",
    "instrument": "#337df4",
    "medicine": "#e8474d",
}
CATEGORY_RGBA = {
    "food": (0.10, 0.77, 0.42, 1.0),
    "tool": (0.61, 0.66, 0.73, 1.0),
    "instrument": (0.20, 0.49, 0.96, 1.0),
    "medicine": (0.91, 0.28, 0.31, 1.0),
}

STORAGE = [
    (9.30, -1.50, "food"), (9.30, -0.50, "tool"),
    (9.30, 0.50, "instrument"), (9.30, 1.50, "medicine"),
    (10.30, -1.50, "medicine"), (10.30, -0.50, "instrument"),
    (10.30, 0.50, "tool"), (10.30, 1.50, "food"),
]
HOME = {
    "food": (12.85, -1.35),
    "tool": (12.85, -0.45),
    "instrument": (12.85, 0.45),
    "medicine": (12.85, 1.35),
}
HOME_ORDER = ("food", "tool", "instrument", "medicine")

START_LOGISTICS = (7.60, 0.00, 0.34)
START_OBSTACLE = (0.00, 0.00, 0.34)
START_MANUAL = (7.60, 0.00, 0.34)

# Free-space guidepoints used by the obstacle demonstration.  Each checkpoint
# is (x, y, label, points).
OBSTACLE_ROUTE = [
    (0.00, 0.00, "启动区 1", 0),
    (0.00, 1.10, "高墙接近", 0),
    (1.20, 1.10, "高墙", 150),
    (-2.00, 1.20, "单木桥", 150),
    (-4.40, 1.20, "砖木坑", 150),
    (-4.40, -1.20, "沟壑组合", 150),
    (-2.83, -1.20, "轮胎阵", 150),
    (1.00, -1.20, "限高框架", 150),
    (3.80, -1.20, "绕桩 · A", 110),
    (6.00, -0.40, "绕桩 · B", 110),
    (4.60, 1.20, "梅花桩", 150),
    (0.00, 0.00, "返回启动区", 0),
]
OBSTACLE_CHECKPOINT_RADIUS = 0.62

MISSION_SECONDS = 180.0
CARRY_LIMIT = 2
ROBOT_RADIUS = 0.30

# Policy command envelope (m/s, m/s, rad/s).
CMD_LIMITS = (1.15, 0.55, 1.20)
CRUISE_SPEED = 0.62
FINE_SPEED = 0.34

# Camera presets: azimuth / elevation / distance.
CAMERA_PRESETS = {
    "chase": {"azimuth": 180.0, "elevation": -18.0, "distance": 3.9},
    "top": {"azimuth": 180.0, "elevation": -89.5, "distance": 13.5},
    "wide": {"azimuth": 145.0, "elevation": -32.0, "distance": 9.0},
}

# Static decision tree shown in the dashboard.  ``path`` from the live
# controller is a list of these ids.
TREE = {
    "id": "root",
    "label": "任务选择",
    "hint": "每 200 ms 评估一次",
    "children": [
        {
            "id": "guard",
            "label": "安全 / 时间守卫",
            "hint": "跌倒或剩余时间不足？",
            "children": [
                {"id": "recover", "label": "恢复姿态", "leaf": True},
                {"id": "return", "label": "返回启动区", "leaf": True},
            ],
        },
        {
            "id": "full",
            "label": "载运数 = 2 ?",
            "hint": "一次最多携带两个物块",
            "children": [
                {"id": "deliver", "label": "前往对应归位区投放", "leaf": True},
                {
                    "id": "pair",
                    "label": "有同类别待运箱 ?",
                    "hint": "优先配对以形成叠放",
                    "children": [
                        {"id": "pick_same", "label": "搬运同类别第二箱", "leaf": True},
                        {"id": "deliver_one", "label": "先投放单箱", "leaf": True},
                    ],
                },
            ],
        },
        {
            "id": "empty",
            "label": "场上有待运箱 ?",
            "hint": "读取定位与库存状态",
            "children": [
                {"id": "pick_new", "label": "选择最优类别并取箱", "leaf": True},
                {"id": "done", "label": "任务完成 · 返回", "leaf": True},
            ],
        },
    ],
}

# Rules scoring.
SCORE_STORAGE = 100
SCORE_HOME_FULL = 100
SCORE_HOME_PARTIAL = 50
SCORE_STACK = 150
SCORE_BAR = 100
PENALTY_WRONG_ZONE = 20
