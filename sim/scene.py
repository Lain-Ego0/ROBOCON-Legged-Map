"""Build the combined ROBOCON 2027 MuJoCo scene (field + S-Dog + cargo)."""
from __future__ import annotations

import pathlib
import re

HERE = pathlib.Path(__file__).resolve().parent
ASSETS = HERE / "assets"
FIELD = ASSETS / "field.xml"
ROBOT = ASSETS / "sdog2.xml"
SCENE = ASSETS / "scene.xml"

# Cargo categories in rule-book order.
CATEGORIES = ("food", "tool", "instrument", "medicine")
CATEGORY_COLORS = {
    "food": "0.16 0.78 0.42 1",
    "tool": "0.62 0.68 0.76 1",
    "instrument": "0.20 0.50 0.96 1",
    "medicine": "0.94 0.28 0.30 1",
}
# Storage slots: two rows of four, 1.0 m pitch.  Row order is announced before
# the match; the far row is randomised.  This is one valid draw.
STORAGE = [
    (9.30, -1.50, "food"), (9.30, -0.50, "tool"),
    (9.30, 0.50, "instrument"), (9.30, 1.50, "medicine"),
    (10.30, -1.50, "medicine"), (10.30, -0.50, "instrument"),
    (10.30, 0.50, "tool"), (10.30, 1.50, "food"),
]
HOME = [
    (12.85, -1.35, "food"),
    (12.85, -0.45, "tool"),
    (12.85, 0.45, "instrument"),
    (12.85, 1.35, "medicine"),
]
HOME_COLORS = {
    "food": "0.12 0.78 0.38 1",
    "tool": "0.58 0.63 0.70 1",
    "instrument": "0.18 0.45 0.96 1",
    "medicine": "0.91 0.22 0.26 1",
}


def _section(text: str, tag: str, end_marker: str | None = None) -> str:
    """Return a complete top-level XML section, matching nested tags correctly."""
    start = text.index(f"<{tag}>")
    end = text.rfind(f"</{tag}>", 0, text.index(end_marker) if end_marker else len(text))
    return text[start:end + len(tag) + 3]


def _robot_fragment() -> tuple[str, str, str, str]:
    robot = ROBOT.read_text(encoding="utf-8")
    default_block = _section(robot, "default", "<asset>")
    inner = default_block[len("<default>"):-len("</default>")]
    # Do not install an unnamed default: it would silently change every field
    # geom's collision flags.  Scope robot materials to a named class.
    defaults = f'<default class="sdog_visual">{inner}</default>'
    assets = _section(robot, "asset")[len("<asset>"):-len("</asset>")]
    world = _section(robot, "worldbody")[len("<worldbody>"):-len("</worldbody>")]
    actuators = _section(robot, "actuator")[len("<actuator>"):-len("</actuator>")]
    # Every visual robot geom uses the named class so the field remains untouched.
    world = re.sub(r'<geom mesh="([A-Za-z0-9_]+_vis)"', r'<geom class="sdog_visual" mesh="\1"', world)
    return defaults, assets, world, actuators


def build() -> pathlib.Path:
    """Compose assets/scene.xml and return its path."""
    field = FIELD.read_text(encoding="utf-8")
    defaults, robot_assets, robot_world, actuators = _robot_fragment()

    # Correct the rule-book home-zone colours (the source map had R/G swapped).
    for kind, rgba in HOME_COLORS.items():
        field = re.sub(
            rf'(<geom name="extra_home_{kind}"[^>]*?rgba=")[^"]+(")',
            rf'\g<1>{rgba}\g<2>', field, count=1,
        )

    # Embed the robot assets and scoped defaults.
    field = field.replace(
        '<compiler angle="radian" coordinate="local" />',
        '<compiler angle="radian" coordinate="local" meshdir="meshes"/>', 1)
    field_default = _section(field, "default", "<option")
    merged_default = field_default[: -len("</default>")] + f"  {defaults}\n  </default>"
    field = field.replace(field_default, merged_default, 1)
    floor_assets = (
        '    <texture name="field_checker" type="2d" builtin="checker" mark="none" '
        'rgb1="0.54 0.45 0.10" rgb2="0.58 0.49 0.11" width="256" height="256"/>\n'
        '    <material name="field_material" texture="field_checker" texuniform="true" '
        'texrepeat="12 6" reflectance="0.04"/>\n'
    )
    field = field.replace('</asset>', floor_assets + robot_assets + '\n  </asset>', 1)

    # Cargo boxes.  Collision flags are explicit because the robot default class
    # only covers robot geoms.
    blocks = []
    for i, (x, y, category) in enumerate(STORAGE):
        blocks.append(
            f'    <body name="block_{i}" pos="{x} {y} 0.101">'
            f'<freejoint name="block_{i}_joint"/>'
            f'<geom name="block_{i}_geom" type="box" size="0.1 0.1 0.1" '
            f'rgba="{CATEGORY_COLORS[category]}" mass="0.3" '
            f'friction="2.0 0.05 0.001" contype="4" conaffinity="4"/></body>'
        )
    field = field.replace('</worldbody>', robot_world + '\n' + '\n'.join(blocks) + '\n  </worldbody>', 1)
    field = field.replace('</mujoco>', f'<actuator>{actuators}</actuator>\n</mujoco>', 1)

    # Cinematic viewport and lighting for the WebUI.
    field = re.sub(
        r'<global azimuth="[^"]*" elevation="[^"]*" />',
        '<global azimuth="-130" elevation="-20" offwidth="1600" offheight="900"/>',
        field, count=1)
    if '<quality ' not in field:
        field = field.replace(
            '<visual>\n',
            '<visual>\n    <quality shadowsize="8192" offsamples="4"/>\n', 1)
    field = field.replace(
        '<light pos="0 0 8" directional="true" dir="0 0 -1"/>',
        '<light pos="6 0 8" directional="true" dir="0 0 -1" diffuse="0.75 0.75 0.75" castshadow="true"/>'
        '<light pos="-6 4 6" directional="true" dir="0 0 -1" diffuse="0.22 0.25 0.30"/>', 1)
    field = field.replace(
        '<texture name="terrain_checker" type="2d" builtin="checker" mark="edge" rgb1="0.08 0.20 0.38" rgb2="0.24 0.50 0.78" markrgb="0.86 0.93 1.0" width="300" height="300" />',
        '<texture name="terrain_checker" type="2d" builtin="checker" mark="edge" rgb1="0.045 0.065 0.10" rgb2="0.075 0.11 0.16" markrgb="0.20 0.30 0.42" width="300" height="300" />', 1)
    field = field.replace('<map fogstart="15.565" fogend="50.94" />', '<map fogstart="22" fogend="70" />', 1)

    for floor_name in ("arena", "extra_area_floor"):
        pattern = re.compile(rf'(<geom name="{floor_name}".*?)(/?>)', re.S)

        def _floor_mat(match):
            tag = match.group(1)
            tag = re.sub(r'rgba="[^"]*"', 'rgba="0.55 0.45 0.10 1"', tag)
            if 'material=' not in tag:
                tag = tag.rstrip() + ' material="field_material"'
            return tag + match.group(2)

        field = pattern.sub(_floor_mat, field, count=1)
    field = field.replace('rgba="1.0 0.55 0.0 1"', 'rgba="0.58 0.30 0.06 1"')
    field = field.replace('<headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />',
                          '<headlight diffuse="0.46 0.46 0.46" ambient="0.17 0.17 0.17" specular="0.04 0.04 0.04" />')
    field = field.replace('diffuse="0.75 0.75 0.75" castshadow="true"', 'diffuse="0.55 0.55 0.55" castshadow="true"')

    # Collision bits: robot/field use bit 1, cargo uses bit 4.  Cargo can rest
    # and stack on the floor but is invisible to the robot's collision mask,
    # so side-contact pickup never launches a box across the arena.
    for floor_name in ("ground", "arena", "extra_area_floor"):
        pattern = re.compile(rf'(<geom name="{floor_name}".*?)(/?>)', re.S)

        def _floor(match, _name=floor_name):
            tag = re.sub(r'contype="[^"]*"', '', match.group(1))
            tag = re.sub(r'conaffinity="[^"]*"', '', tag)
            return tag.rstrip() + ' contype="5" conaffinity="5"' + match.group(2)

        field = pattern.sub(_floor, field, count=1)
    SCENE.write_text(field, encoding="utf-8")
    return SCENE


if __name__ == "__main__":
    print(build())
