"""Gimbal kinematics of the RPM: where gravity points as seen by the sample.

Conventions follow the team's docs/interfaces.md:

- World frame: x along the horizontal **outer** axis, z up, gravity (0, 0, -1) g.
- ``theta_outer`` turns the inner frame inside the fixed outer frame, about x.
- ``theta_inner`` turns the platform inside the inner frame, about the inner
  frame's own z axis (vertical when both angles are 0).
- Radians, positive by the right-hand rule. At (0, 0) the sample frame is the
  world frame.

The sample orientation is R = Rx(theta_outer) @ Rz(theta_inner), so gravity in
the sample frame is R^T @ (0, 0, -1).

MicroPython compatible: only ``math`` and plain tuples.
"""

import math

TWO_PI = 2.0 * math.pi
DEG = math.pi / 180.0


def gravity_in_sample(theta_outer: float, theta_inner: float) -> "tuple[float, float, float]":
    """Unit gravity vector (in g) expressed in the sample frame."""
    s_o = math.sin(theta_outer)
    s_i = math.sin(theta_inner)
    c_i = math.cos(theta_inner)
    return (-s_i * s_o, -c_i * s_o, -math.cos(theta_outer))


def wrap_angle(theta: float) -> float:
    """Map an angle to [0, 2*pi). Keeps float32 precision on the Pico bounded."""
    return theta % TWO_PI
