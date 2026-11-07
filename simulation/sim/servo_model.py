"""FS90R continuous servo: pulse width -> drive level, and its DC-motor torque curve.

This is the sim's "true" servo. The firmware has its own calibration table (rpmcore/config.py);
the two may disagree, just like the real servo and an imperfect calibration.
"""

from sim.params import MachineParams


def u_from_pulse(pulse_us: int | None, p: MachineParams) -> float:
    """Drive level in [-1, 1]. None (PWM off) and pulses inside the deadband give 0."""
    if pulse_us is None:
        return 0.0
    off = pulse_us - p.servo_neutral_us
    mag = abs(off) - p.deadband_half_us
    if mag <= 0:
        return 0.0
    u = min(1.0, mag / (p.sat_offset_us - p.deadband_half_us))
    return u if off > 0 else -u


def torque(u: float, omega: float, p: MachineParams) -> float:
    """Shaft torque tau = tau_stall * (u - omega/omega0), limited to +-tau_stall.

    Reference formula; MuJoCo realises the same curve with a velocity actuator (sim/mjcf.py).
    """
    t = p.servo_stall_torque * (u - omega / p.omega0)
    return max(-p.servo_stall_torque, min(p.servo_stall_torque, t))
