#!/usr/bin/env python3
"""
Single source of truth for the "obstacle_course" environment.

Generates, from ONE obstacle list:
  * worlds/obstacle_course.sdf.xacro  -> the Gazebo (gz sim Harmonic) world
  * maps/obstacle_course.pgm/.yaml    -> the matching occupancy map for Nav2

Obstacles marked in_map=False are put in Gazebo but NOT in the map, so the
robot only discovers them with its LiDAR -> demonstrates live obstacle
avoidance by the local/global costmap obstacle layers.

Usage (from the package root):
    python3 tools/generate_world_and_map.py
Only needs the Python standard library.
"""
import math
import os

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------- layout ----
ARENA = 10.0          # inner size of the square arena (m), centred at 0,0
WALL_T = 0.20         # wall thickness (m)
WALL_H = 0.50         # wall height (m)

# boxes: (name, cx, cy, sx, sy, height, rgb, in_map)
BOXES = [
    # outer walls
    ("wall_north", 0.0,  ARENA / 2 + WALL_T / 2, ARENA + 2 * WALL_T, WALL_T, WALL_H, (0.6, 0.6, 0.6), True),
    ("wall_south", 0.0, -ARENA / 2 - WALL_T / 2, ARENA + 2 * WALL_T, WALL_T, WALL_H, (0.6, 0.6, 0.6), True),
    ("wall_east",  ARENA / 2 + WALL_T / 2, 0.0, WALL_T, ARENA, WALL_H, (0.6, 0.6, 0.6), True),
    ("wall_west", -ARENA / 2 - WALL_T / 2, 0.0, WALL_T, ARENA, WALL_H, (0.6, 0.6, 0.6), True),
    # two inner partitions -> S-shaped route from bottom-left to top-right
    ("partition_low",  -1.75, -1.5, 6.5, WALL_T, WALL_H, (0.8, 0.5, 0.2), True),   # x -5.0 .. 1.5
    ("partition_high",  1.75,  1.5, 6.5, WALL_T, WALL_H, (0.8, 0.5, 0.2), True),   # x -1.5 .. 5.0
    # static clutter (known in map)
    ("crate_a",  2.5, -3.5, 0.8, 0.8, 0.6, (0.2, 0.4, 0.8), True),
    ("crate_b", -3.5, -0.5, 0.6, 0.6, 0.6, (0.2, 0.4, 0.8), True),
    ("table",    0.0,  3.3, 1.0, 0.6, 0.6, (0.2, 0.4, 0.8), True),
    # surprise obstacle: in Gazebo, NOT in the map
    ("surprise_box", -1.0, 0.0, 0.5, 0.5, 0.5, (0.9, 0.1, 0.1), False),
]

# cylinders: (name, cx, cy, radius, height, rgb, in_map)
CYLINDERS = [
    ("pillar_a", -2.0, -3.2, 0.35, 0.8, (0.2, 0.7, 0.3), True),
    ("pillar_b",  2.5,  0.3, 0.35, 0.8, (0.2, 0.7, 0.3), True),
    ("pillar_c", -3.0,  3.5, 0.30, 0.8, (0.2, 0.7, 0.3), True),
    # surprise obstacle: in Gazebo, NOT in the map
    ("surprise_barrel", 2.0, 3.9, 0.25, 0.6, (0.9, 0.1, 0.1), False),
]

# ------------------------------------------------------------------ map -----
RES = 0.05
MAP_SIZE = ARENA + 2 * WALL_T + 0.6          # a little margin around the walls
ORIGIN = (-MAP_SIZE / 2, -MAP_SIZE / 2)
FREE, OCC, UNKNOWN = 254, 0, 205


def build_map():
    n = int(round(MAP_SIZE / RES))
    img = [[UNKNOWN] * n for _ in range(n)]
    half = ARENA / 2
    for r in range(n):
        y = ORIGIN[1] + (n - r - 0.5) * RES      # row 0 is the TOP of the image
        for c in range(n):
            x = ORIGIN[0] + (c + 0.5) * RES
            if abs(x) <= half and abs(y) <= half:
                img[r][c] = FREE
            for (_, cx, cy, sx, sy, _, _, in_map) in BOXES:
                if in_map and abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2:
                    img[r][c] = OCC
            for (_, cx, cy, rad, _, _, in_map) in CYLINDERS:
                if in_map and math.hypot(x - cx, y - cy) <= rad:
                    img[r][c] = OCC
    return n, img


def write_map():
    n, img = build_map()
    os.makedirs(os.path.join(PKG, "maps"), exist_ok=True)
    with open(os.path.join(PKG, "maps", "obstacle_course.pgm"), "wb") as f:
        f.write(f"P5\n# obstacle_course\n{n} {n}\n255\n".encode())
        f.write(bytes(v for row in img for v in row))
    with open(os.path.join(PKG, "maps", "obstacle_course.yaml"), "w") as f:
        f.write(
            "image: obstacle_course.pgm\n"
            "mode: trinary\n"
            f"resolution: {RES}\n"
            f"origin: [{ORIGIN[0]:.3f}, {ORIGIN[1]:.3f}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n"
        )


# ---------------------------------------------------------------- world -----
def material(rgb):
    r, g, b = rgb
    return (f"<material><ambient>{r} {g} {b} 1</ambient>"
            f"<diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def box_model(name, cx, cy, sx, sy, h, rgb):
    geom = f"<geometry><box><size>{sx} {sy} {h}</size></box></geometry>"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {h / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">{geom}</collision>
        <visual name="visual">{geom}{material(rgb)}</visual>
      </link>
    </model>"""


def cyl_model(name, cx, cy, rad, h, rgb):
    geom = f"<geometry><cylinder><radius>{rad}</radius><length>{h}</length></cylinder></geometry>"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {h / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">{geom}</collision>
        <visual name="visual">{geom}{material(rgb)}</visual>
      </link>
    </model>"""


WORLD_HEADER = """<?xml version="1.0"?>
<!-- AUTO-GENERATED by tools/generate_world_and_map.py - edit that script, not this file -->
<sdf version="1.6" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="headless" default="true"/>

  <world name="default">
    <physics name="3ms" type="ode">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <xacro:unless value="$(arg headless)">
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    </xacro:unless>
    <!-- needed for the TurtleBot3 LiDAR / camera -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <shadows>0</shadows>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.85 0.95 1</background>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

WORLD_FOOTER = """
  </world>
</sdf>
"""


def write_world():
    parts = [WORLD_HEADER]
    for (name, cx, cy, sx, sy, h, rgb, _) in BOXES:
        parts.append(box_model(name, cx, cy, sx, sy, h, rgb))
    for (name, cx, cy, rad, h, rgb, _) in CYLINDERS:
        parts.append(cyl_model(name, cx, cy, rad, h, rgb))
    parts.append(WORLD_FOOTER)
    os.makedirs(os.path.join(PKG, "worlds"), exist_ok=True)
    with open(os.path.join(PKG, "worlds", "obstacle_course.sdf.xacro"), "w") as f:
        f.write("".join(parts))


if __name__ == "__main__":
    write_world()
    write_map()
    print("Generated worlds/obstacle_course.sdf.xacro and maps/obstacle_course.{pgm,yaml}")
