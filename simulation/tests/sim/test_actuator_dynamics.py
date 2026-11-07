import numpy as np
import pytest

from sim.machine import SimMachine
from sim.params import MachineParams


@pytest.fixture
def m():
    return SimMachine(MachineParams())


def test_saturated_pulse_reaches_noload_speed(m):
    p = m.params
    m.step(1500, 1850, duration=0.5)
    w = m.state()["w_outer"]
    # Gravity torque on the balanced frame is small; friction takes a few percent.
    assert w == pytest.approx(p.omega0 / p.gear_ratio, rel=0.05)


def test_stalled_joint_gives_stall_torque():
    # Held by a huge inertia: MuJoCo's dry friction is soft and always lets a joint creep.
    p = MachineParams(armature_outer=1e3)
    m = SimMachine(p)
    m.step(None, 1850, duration=0.2)
    s = m.state()
    assert abs(s["w_outer"]) < 1e-3
    assert s["tau_outer"] == pytest.approx(p.servo_stall_torque, rel=0.02)


def test_deadband_pulse_does_not_move(m):
    m.step(1530, 1470, duration=1.0)
    s = m.state()
    assert abs(s["q_outer"]) < 1e-3 and abs(s["q_inner"]) < 1e-3


def test_pinions_turn_minus_four_times(m):
    m.step(1850, 1850, duration=1.0)
    s = m.state()
    d, mdl = m.data, m.model
    q_op = d.qpos[mdl.jnt_qposadr[mdl.joint("outer_pinion").id]]
    q_ip = d.qpos[mdl.jnt_qposadr[mdl.joint("inner_pinion").id]]
    assert q_op == pytest.approx(-4 * s["q_outer"], rel=0.01)
    assert q_ip == pytest.approx(-4 * s["q_inner"], rel=0.01)


def test_positive_pulse_turns_positive(m):
    m.step(1700, 1700, duration=0.5)
    s = m.state()
    assert s["q_outer"] > 0 and s["q_inner"] > 0


def test_gravity_in_sample_frame_at_rest(m):
    # The sample frame is the platform's STL frame, where +Y is up at q = 0.
    s = m.state()
    assert np.allclose(s["g_sample"], [0, -9.81, 0], atol=1e-6)
    # An accelerometer at rest reads the opposite of gravity.
    assert np.allclose(s["acc_sample"] + s["g_sample"], 0, atol=1e-3)
