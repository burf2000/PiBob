#!/usr/bin/env python3
"""Generate meshes/Servo-9g.stl - a simple 9 g micro servo (SG90 / MG90S class) in mm.

    uv run --with numpy --with trimesh ros2/pibob_description/scripts/make_servo_stl.py

Frame (so the visual's <origin> in the URDF IS the servo's output shaft):
  origin = on the output-shaft axis, at the horn face (top of the gear boss)
  +z     = along the shaft, out of the servo
  +x     = along the body length, from the shaft end towards the far end
  y      = z cross x... i.e. right-handed; the body is centred in y

Dimensions (mm). Pocket values measured from the PiBob holder STLs; the rest are
standard SG90/MG90S datasheet values:
  body            22.8 L x 12.2 W x 22.7 H   (holder pockets: 23.0 x 12.2 and 23.0 x 12.1)
  shaft offset    5.85 from the shaft-end face  (Head-BottomServoHolder: pocket end x=-1.75,
                                                 coaxial pivot pin at x=4.10 -> 5.85)
  mounting tabs   32.2 span x 12.2 x 2.5, underside 16.0 above the body bottom
  gear boss       dia 11.8 x 4.0 on the case top, centred on the shaft (SG90 / MG90S
                  overall height ~28.5-31 incl. spline; without it the horn face sat 4 mm
                  too low and the head's middle holder collided with its base holder)
  output spline   dia 4.8, 3.0 proud of the boss

The origin (and so every servo joint pivot) is the horn face = top of the gear boss.
"""
import os

import numpy as np
import trimesh

L, W, H = 22.8, 12.2, 22.7
SHAFT_OFF = 5.85
TAB_SPAN, TAB_T, TAB_Z = 32.2, 2.5, 16.0
BOSS_D, BOSS_H = 11.8, 4.0
SPLINE_D, SPLINE_H = 4.8, 3.0


def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


def main():
    x0 = -SHAFT_OFF                      # shaft-end face
    x1 = x0 + L
    tab_over = (TAB_SPAN - L) / 2
    top = -BOSS_H                        # case top; the origin is the boss (horn) face
    parts = [
        box(x0, x1, -W / 2, W / 2, top - H, top),                                                # body
        box(x0 - tab_over, x1 + tab_over, -W / 2, W / 2, top - H + TAB_Z, top - H + TAB_Z + TAB_T),  # tabs
    ]
    boss = trimesh.creation.cylinder(radius=BOSS_D / 2, height=BOSS_H, sections=48)
    boss.apply_translation([0, 0, -BOSS_H / 2])
    parts.append(boss)
    spline = trimesh.creation.cylinder(radius=SPLINE_D / 2, height=SPLINE_H, sections=32)
    spline.apply_translation([0, 0, SPLINE_H / 2])
    parts.append(spline)
    m = trimesh.util.concatenate(parts)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "meshes", "Servo-9g.stl")
    m.export(os.path.normpath(out))
    print("wrote", os.path.normpath(out), "bounds", m.bounds.round(2).tolist(), "faces", len(m.faces))


if __name__ == "__main__":
    main()
