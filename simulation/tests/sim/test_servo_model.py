import pytest

from sim.params import MachineParams
from sim.servo_model import torque, u_from_pulse

P = MachineParams()


@pytest.mark.parametrize("us", [None, 1500, 1455, 1545, 1520, 1480])
def test_deadband_and_off_give_zero(us):
    assert u_from_pulse(us, P) == 0.0


@pytest.mark.parametrize("us, u", [(1850, 1.0), (2300, 1.0), (1150, -1.0), (700, -1.0)])
def test_saturation(us, u):
    assert u_from_pulse(us, P) == u


def test_sign_and_linear_between():
    mid = 1500 + (45 + 350) / 2
    assert u_from_pulse(mid, P) == pytest.approx(0.5)
    assert u_from_pulse(3000 - mid, P) == pytest.approx(-0.5)
    assert u_from_pulse(1600, P) > 0 > u_from_pulse(1400, P)


def test_neutral_offset_respected():
    p = MachineParams(servo_neutral_us=1520)
    assert u_from_pulse(1560, p) == 0.0
    assert u_from_pulse(1520 + 350, p) == 1.0


def test_dc_motor_curve():
    w0 = P.omega0
    assert torque(1.0, 0.0, P) == pytest.approx(P.servo_stall_torque)
    assert torque(1.0, w0, P) == pytest.approx(0.0)
    assert torque(0.5, 0.0, P) == pytest.approx(0.5 * P.servo_stall_torque)
    assert torque(0.0, -w0, P) == pytest.approx(P.servo_stall_torque)  # braking
    assert torque(-1.0, w0, P) == pytest.approx(-P.servo_stall_torque)  # clipped
