"""Offline-rendered tactical map of the combined ROBOCON 2027 field."""
from __future__ import annotations

import io
import math
import pathlib

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .config import HOME, HOME_ORDER, STORAGE, CATEGORY_COLORS

BOUNDS = (-7.35, 14.35, -3.15, 3.15)
WIDTH = 1880
HEIGHT = int(WIDTH * (BOUNDS[3] - BOUNDS[2]) / (BOUNDS[1] - BOUNDS[0]))
FONT_PATHS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]


def _font(size: int):
    for path in FONT_PATHS:
        if pathlib.Path(path).exists():
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return ImageFont.load_default()


def _sx(x):
    return (x - BOUNDS[0]) / (BOUNDS[1] - BOUNDS[0]) * WIDTH


def _sy(y):
    return (BOUNDS[3] - y) / (BOUNDS[3] - BOUNDS[2]) * HEIGHT


def _rect(draw, x0, y0, x1, y1, **kw):
    draw.rectangle([_sx(x0), _sy(y1), _sx(x1), _sy(y0)], **kw)


def _aabb(model, data, gid):
    import mujoco

    gtype = model.geom_type[gid]
    pos = data.geom_xpos[gid]
    mat = data.geom_xmat[gid].reshape(3, 3)
    size = model.geom_size[gid]
    if gtype == mujoco.mjtGeom.mjGEOM_BOX:
        ext = np.abs(mat) @ size
    elif gtype in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
        ext = np.abs(mat) @ np.array([size[0], size[0], size[1]])
    elif gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
        ext = np.array([size[0]] * 3)
    else:
        ext = np.array([0.1, 0.1, 0.1])
    return pos - ext, pos + ext


def render(model, data) -> bytes:
    img = Image.new("RGB", (WIDTH, HEIGHT), "#070b12")
    d = ImageDraw.Draw(img, "RGBA")
    # Subtle 1 m grid.
    for x in range(-7, 15):
        d.line([(_sx(x), 0), (_sx(x), HEIGHT)], fill=(30, 48, 70, 90), width=1)
    for y in range(-3, 4):
        d.line([(0, _sy(y)), (WIDTH, _sy(y))], fill=(30, 48, 70, 90), width=1)
    # Arena beds.
    _rect(d, -7, -3, 7, 3, fill=(26, 22, 11, 255), outline=(196, 148, 32, 220), width=3)
    _rect(d, 8, -2, 14, 2, fill=(28, 17, 9, 255), outline=(235, 112, 30, 230), width=3)
    _rect(d, 7, -3, 8, 3, fill=(12, 13, 15, 255), outline=(60, 75, 95, 120), width=1)
    # Task field walls (opening on the west side).
    wall = (238, 121, 32, 220)
    _rect(d, 8.0, -2.0, 8.05, -0.5, fill=wall, outline=None)
    _rect(d, 8.0, 0.5, 8.05, 2.0, fill=wall, outline=None)
    _rect(d, 8.0, -2.05, 14.0, -2.0, fill=wall, outline=None)
    _rect(d, 8.0, 2.0, 14.0, 2.05, fill=wall, outline=None)
    _rect(d, 14.0, -2.05, 14.05, 2.05, fill=wall, outline=None)
    # Bars.
    for y0, y1 in ((-1.3, -0.5), (0.5, 1.3)):
        _rect(d, 11.56, y0, 11.64, y1, fill=(250, 196, 55, 255), outline=(20, 20, 20, 255), width=2)
    # Storage cells.
    for i, (x, y, cat) in enumerate(STORAGE):
        _rect(d, x - 0.1, y - 0.1, x + 0.1, y + 0.1, fill=(120, 20, 20, 60), outline=(235, 60, 60, 200), width=2)
    # Home cells.
    for cat in HOME_ORDER:
        x, y = HOME[cat]
        col = CATEGORY_COLORS[cat]
        rgb = tuple(int(col[i:i + 2], 16) for i in (1, 3, 5))
        _rect(d, x - 0.15, y - 0.15, x + 0.15, y + 0.15, fill=(*rgb, 70), outline=(*rgb, 255), width=3)
    # Obstacles, from live geometry AABBs.
    draw_colors = {
        "high_wall": (220, 90, 90, 230),
        "limit_frame": (250, 170, 40, 220),
        "extra_wall": (238, 121, 32, 220),
        "extra_bar": (250, 196, 55, 255),
        "stairs": (52, 170, 120, 200),
        "platform": (52, 170, 120, 200),
        "ramp": (52, 170, 120, 200),
        "stepping_stones": (70, 190, 130, 220),
        "L_pile": (80, 210, 140, 230),
        "tire_ring": (120, 150, 170, 180),
        "pit_wall": (60, 170, 110, 180),
        "pit_separator": (60, 170, 110, 180),
        "single_bridge": (60, 180, 130, 200),
    }
    seen_tires = set()
    for gid in range(model.ngeom):
        name = model.geom(gid).name or ""
        if not name:
            continue
        if name.startswith("tire_ring"):
            pos = data.geom_xpos[gid]
            key = (round(pos[0], 1), round(pos[1], 1))
            if key in seen_tires:
                continue
            seen_tires.add(key)
            r = 0.275
            d.ellipse([_sx(pos[0] - r), _sy(pos[1] + r), _sx(pos[0] + r), _sy(pos[1] - r)],
                      outline=(150, 175, 195, 170), width=2)
            d.ellipse([_sx(pos[0] - 0.165), _sy(pos[1] + 0.165), _sx(pos[0] + 0.165), _sy(pos[1] - 0.165)],
                      outline=(150, 175, 195, 110), width=1)
            continue
        if name.startswith(("gravel_", "wood_", "extra_storage", "extra_home", "start_zone",
                            "extra_start_zone", "arena", "ground", "extra_area_floor",
                            "meic_", "extra_wall")):
            continue
        color = None
        for prefix, col in draw_colors.items():
            if name.startswith(prefix):
                color = col
                break
        if color is None:
            continue
        lo, hi = _aabb(model, data, gid)
        if hi[2] < 0.04:
            continue
        _rect(d, lo[0], lo[1], hi[0], hi[1], fill=(*color[:3], 80), outline=color, width=1)
    # Special single bridge (platform_04 is 3.0 x 0.4 at y=2).
    _rect(d, -3.5, 1.8, -0.5, 2.2, fill=(60, 180, 130, 120), outline=(90, 230, 160, 255), width=3)
    # Start zones.
    _rect(d, -0.4, -0.4, 0.4, 0.4, fill=(240, 60, 60, 30), outline=(250, 80, 80, 230), width=3)
    _rect(d, 7.2, -0.5, 8.0, 0.5, fill=(240, 60, 60, 30), outline=(250, 80, 80, 230), width=3)
    # Labels.
    f_small = _font(22)
    f_mid = _font(26)
    f_title = _font(30)
    labels = [
        (4.4, -1.2, "绕桩", (150, 230, 190)),
        (1.0, -2.0, "限高框架", (250, 200, 110)),
        (1.2, 2.0, "高墙", (240, 140, 140)),
        (-5.4, -2.0, "沟壑组合", (140, 230, 190)),
        (-5.7, 1.35, "砖木坑", (140, 230, 190)),
        (-2.83, -2.0, "轮胎阵", (180, 205, 225)),
        (4.6, 2.0, "梅花桩", (140, 230, 190)),
        (-2.0, 2.0, "单木桥", (140, 230, 190)),
    ]
    for x, y, text, col in labels:
        tw = d.textlength(text, font=f_small)
        d.rounded_rectangle([_sx(x) - tw / 2 - 7, _sy(y) - 15, _sx(x) + tw / 2 + 7, _sy(y) + 15],
                            radius=5, fill=(5, 9, 15, 210), outline=(*col, 90), width=1)
        d.text((_sx(x) - tw / 2, _sy(y) - 13), text, font=f_small, fill=(*col, 255))
    d.text((_sx(-6.8), 14), "障碍赛场地 · 14 m × 6 m", font=f_title, fill=(224, 190, 120, 255))
    d.text((_sx(8.2), 14), "任务赛场地 · 6 m × 4 m", font=f_title, fill=(245, 160, 100, 255))
    d.text((_sx(8.2), HEIGHT - 34), "存放区", font=f_small, fill=(240, 120, 120, 230))
    d.text((_sx(12.0), HEIGHT - 34), "归位区", font=f_small, fill=(200, 200, 200, 230))
    # Scale bar.
    sx, sy = _sx(-6.8), HEIGHT - 30
    d.line([(sx, sy), (sx + WIDTH / (BOUNDS[1] - BOUNDS[0]), sy)], fill=(180, 200, 220, 220), width=4)
    d.text((sx, sy - 26), "1 m", font=f_small, fill=(180, 200, 220, 220))
    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return buf.getvalue()
