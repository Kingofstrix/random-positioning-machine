"""Machine geometry taken from the STL files (docs/PLAN.md, section 2). All lengths in mm, STL frame.

The meshes are read straight from `rpm_printables/` instead of a copy, so the hardware team's
files stay the single source and their licence attribution stays in one place.

The frames and the platform use the shared assembly frame. The gears, the end cap and the stand
do not: each gear STL is centred on the origin with its axis along X, so they are placed here.
Each placement is checked against the mating part's mesh in tests/sim/test_geometry.py, and
against the assembly guide pictures.
"""

import math
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
STL_DIR = REPO_ROOT / "rpm_printables"

# Both rotation axes pass through this point; outer along X, inner along Y.
AXIS_POINT = (0.0, 0.0, 2.5)
FLOOR_Y = -91.19            # underside of the stands

OUTER_FRAME_X = 84.61       # outer face of the outer frame's side bars
OUTER_FRAME_INNER_X = 76.68 # inner face of the outer frame's side bars, around the bearing hole
INNER_FRAME_Y = 60.0        # outer face of the inner frame on the +Y side

GEAR_CENTER_DIST = 38.0     # 60-tooth gear to 15-tooth pinion, module ~1
SERVO_SHAFT_Z = 14.5        # servo screw holes lie on this height
# Pinion centre on the line z = 14.5 at 38 mm from the axis: sqrt(38^2 - 12^2) = 36.06 mm.
# That matches where a 9 g servo's shaft sits between its screw holes.
PINION_OFFSET = math.sqrt(GEAR_CENTER_DIST**2 - (SERVO_SHAFT_Z - AXIS_POINT[2]) ** 2)

# Outer gear (guide step 3): on the inner frame's square -X stub, between the two frames. Its
# collar (STL x -2.5..0) bears on the outer frame's inner face, its disc (0..3) holds the stub.
OUTER_GEAR_X = -(OUTER_FRAME_INNER_X - 2.5)
# Inner gear (guide step 1): collar (STL x -4..0) on the inner frame's +Y face, disc (0..3) on the
# platform's angled shaft end, a square turned 45 deg. The gear's square bore is turned to match.
INNER_GEAR_Y = INNER_FRAME_Y + 4.0
SHAFT_END_TURN = 45.0       # deg
OUTER_PINION_X = OUTER_GEAR_X - 2.3         # pinion spans x 0..5 in its STL
INNER_PINION_Y = INNER_GEAR_Y - 2.5

SERVO_BOX = (22.5, 12.1, 27.7)   # FS90R length, width, height (height along the shaft)
# Servo bodies [prüfen]. Inner: inside the frame between its screw holes (x 29.1/56.6, bar at
# y 55), shaft through the bar to the pinion. Outer (guide step 4): body outside the outer
# frame, passing the bar between its two screw posts, the box ending at its pinion inside.
OUTER_SERVO_CENTER = (OUTER_PINION_X - SERVO_BOX[2] / 2, 41.6, SERVO_SHAFT_Z)
INNER_SERVO_CENTER = (42.85, 55.0 - SERVO_BOX[2] / 2, SERVO_SHAFT_Z)

# name: (STL file, body, pos mm, euler deg xyz)
PARTS = {
    "outer_frame": ("01_outer_frame.stl", "base", (0, 0, 0), (0, 0, 0)),
    "stand_right": ("optional_stand/Stand.stl", "base", (OUTER_FRAME_X, 0, 0), (0, 0, 0)),
    # Mirrored by a half turn about Y around the axis height, so its holes still line up.
    "stand_left": ("optional_stand/Stand.stl", "base", (-OUTER_FRAME_X, 0, 2 * AXIS_POINT[2]), (0, 180, 0)),
    "inner_frame": ("02_inner_frame.stl", "inner_frame", (0, 0, 0), (0, 0, 0)),
    "gear_large_outer": ("04_spur_gear_large_outer.stl", "inner_frame", (OUTER_GEAR_X, 0, AXIS_POINT[2]), (0, 0, 0)),
    "platform": ("03_experiment_platform.stl", "platform", (0, 0, 0), (0, 0, 0)),
    "gear_large_inner": ("05_spur_gear_large_inner.stl", "platform", (0, INNER_GEAR_Y, AXIS_POINT[2]), (0, SHAFT_END_TURN, 90)),
    "end_cap": ("07_end_cap.stl", "platform", (0, -63.5, AXIS_POINT[2]), (90, 0, 0)),
}

# Pinion turn about its own axis at q = 0, so its teeth sit in the gear's tooth gaps (one pinion
# tooth is 24 deg). The gear equality turns a pinion exactly 4x its frame (60/15 teeth), so a
# clean mesh at q = 0 stays clean at every angle.
OUTER_PINION_PHASE = -4.0   # deg
INNER_PINION_PHASE = 4.0    # deg, for the inner gear turned by SHAFT_END_TURN

PINIONS = {
    # name: (parent body, joint axis, pos mm, euler deg)
    "outer_pinion": (
        "base", (1, 0, 0), (OUTER_PINION_X, PINION_OFFSET, SERVO_SHAFT_Z), (OUTER_PINION_PHASE, 0, 0)
    ),
    "inner_pinion": (
        "inner_frame", (0, 1, 0), (PINION_OFFSET, INNER_PINION_Y, SERVO_SHAFT_Z), (0, INNER_PINION_PHASE, 90)
    ),
}
PINION_STL = "06_spur_gear_small.stl"
