"""Physical parameters of the simulated machine (docs/PLAN.md 6.3).

Values marked [Annahme] in the plan are estimates until they are measured on the machine (M7).
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MachineParams:
    gear_ratio: float = 4.0
    # FS90R at 5 V, interpolated from the datasheet values for 4.8 V and 6 V [Annahme].
    servo_noload_rpm: float = 83.0
    servo_stall_torque: float = 0.13          # N m at the servo shaft
    servo_neutral_us: int = 1500
    deadband_half_us: int = 45
    sat_offset_us: int = 350                  # pulse offset where the servo saturates [Annahme]
    # How the real servo of each axis differs from the nominal one, as (inner, outer). The firmware
    # keeps its nominal table, so this models calibration mismatch (robustness envelope, PR #25).
    servo_speed_scale: tuple = (1.0, 1.0)
    servo_neutral_shift_us: tuple = (0, 0)

    pla_density: float = 1240.0               # kg/m^3, solid PLA
    fill_factor: float = 0.6                  # infill share of a printed part [Annahme]
    servo_mass: float = 0.009                 # kg, FS90R

    # Reflected rotor inertia at the frame joint. It dominates the frame inertia [Annahme].
    armature_outer: float = 8e-4              # kg m^2
    armature_inner: float = 8e-4
    frictionloss_outer: float = 0.01          # N m, slip ring + PLA bearings [Annahme]
    frictionloss_inner: float = 0.005
    damping: float = 1e-4                     # N m s/rad
    # Time constant of MuJoCo's soft dry friction. The default (0.02 s) lets a held frame creep;
    # the FS90R gearbox barely back-drives, so friction is made stiff (>= 2 timesteps).
    friction_solref: float = 0.002
    # Even stiff soft friction lets a tilted frame creep ~2.4 deg/min under gravity with PWM off.
    # MuJoCo's noslip post-solver turns joint dry friction into a hard constraint, so frictionloss
    # really holds the frame at standstill (PLAN 6.2). 0 switches it off.
    noslip_iterations: int = 10

    payload_mass: float = 0.015               # 50 mm Petri dish with medium [Annahme]
    payload_radius: float = 0.025
    payload_half_height: float = 0.006
    payload_offset: tuple = (0.0, 0.0, 0.0)   # m, from the axis intersection, STL frame

    timestep: float = 0.001
    control_dt: float = 0.020

    @property
    def omega0(self) -> float:
        """No-load servo shaft speed in rad/s."""
        return self.servo_noload_rpm * 2.0 * math.pi / 60.0

    @property
    def density(self) -> float:
        return self.pla_density * self.fill_factor


@dataclass(frozen=True)
class ReportParams:
    """Plot and video settings of sim.report (docs/PLAN.md 6.8)."""

    fig_size: tuple = (8.0, 4.5)              # inch
    dpi: int = 120

    video_seconds: float = 20.0               # length of the MP4
    video_speedup: float = 4.0                # sim seconds per video second
    video_fps: int = 30
    # Must not exceed the model's offscreen buffer (MuJoCo default 640 x 480).
    video_width: int = 640
    video_height: int = 480
    cam_azimuth: float = 135.0                # deg
    cam_elevation: float = -20.0              # deg
    cam_distance: float = 1.2                 # in units of the model extent
