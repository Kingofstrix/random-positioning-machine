import mujoco
import numpy as np
import pytest

from sim.mjcf import build_mjcf
from sim.params import MachineParams

DOWN = np.array([0.0, 0.0, -1.0])


@pytest.fixture(scope="module")
def md():
    m = mujoco.MjModel.from_xml_string(build_mjcf(MachineParams()))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def joint_axis(m, d, name):
    j = m.joint(name).id
    return d.xanchor[j].copy(), d.xaxis[j].copy()


def test_joints(md):
    m, _ = md
    names = {m.joint(i).name for i in range(m.njnt)}
    assert names == {"outer", "inner", "outer_pinion", "inner_pinion"}
    assert all(m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE for i in range(m.njnt))
    assert m.neq == 2


def test_collisions_off(md):
    m, _ = md
    assert not m.geom_contype.any() and not m.geom_conaffinity.any()


def test_axes_geometry_at_zero(md):
    m, d = md
    p_o, a_o = joint_axis(m, d, "outer")
    p_i, a_i = joint_axis(m, d, "inner")
    assert abs(a_o @ DOWN) < 1e-9          # outer axis horizontal
    assert abs(a_o @ a_i) < 1e-9           # axes perpendicular
    assert abs(abs(a_i @ DOWN) - 1) < 1e-9  # inner axis vertical at q = 0
    # Distance between the two axis lines: they must intersect.
    n = np.cross(a_o, a_i)
    assert abs((p_i - p_o) @ n) / np.linalg.norm(n) < 1e-4


def test_sample_site_at_axis_intersection(md):
    m, d = md
    p_o, _ = joint_axis(m, d, "outer")
    p_i, _ = joint_axis(m, d, "inner")
    site = d.site("sample").xpos
    assert np.linalg.norm(site - p_o - ((site - p_o) @ d.xaxis[m.joint("outer").id]) * d.xaxis[m.joint("outer").id]) < 1e-6
    assert np.linalg.norm(np.cross(site - p_i, d.xaxis[m.joint("inner").id])) < 1e-6


OFFSET = (0.004, -0.003, 0.010)   # m, platform STL frame


def posed(params, q_outer, q_inner):
    m = mujoco.MjModel.from_xml_string(build_mjcf(params))
    d = mujoco.MjData(m)
    for name, q in (("outer", q_outer), ("inner", q_inner)):
        d.qpos[m.jnt_qposadr[m.joint(name).id]] = q
        d.qpos[m.jnt_qposadr[m.joint(name + "_pinion").id]] = -params.gear_ratio * q
    mujoco.mj_forward(m, d)
    return m, d


def test_payload_and_site_follow_offset_on_platform():
    # PLAN 6.1: site 'sample' = axis intersection + payload_offset, given in the platform's STL
    # frame, and the payload sits there on the platform. Checked at a tilted pose so that a
    # mirrored offset or an offset applied in the wrong frame cannot pass.
    p = MachineParams(payload_offset=OFFSET)
    m, d = posed(p, 0.7, -1.1)
    centre = d.xanchor[m.joint("inner").id]          # the axes intersect here
    R = d.xmat[m.body("platform").id].reshape(3, 3)
    want = centre + R @ np.array(OFFSET)
    assert np.allclose(d.site("sample").xpos, want, atol=1e-6)
    assert np.allclose(d.geom("payload").xpos, want, atol=1e-6)
    assert m.geom_bodyid[m.geom("payload").id] == m.body("platform").id


# Distinct values for every per-axis field, so that a swapped or ignored parameter shows up.
WIRED = MachineParams(armature_outer=7e-4, armature_inner=3e-4, frictionloss_outer=0.011,
                      frictionloss_inner=0.004, damping=2e-4, friction_solref=0.003,
                      noslip_iterations=7, timestep=0.0005)


def test_params_reach_the_model():
    p = WIRED
    m = mujoco.MjModel.from_xml_string(build_mjcf(p))
    dof = {n: m.jnt_dofadr[m.joint(n).id] for n in ("outer", "inner", "outer_pinion", "inner_pinion")}
    assert m.dof_armature[dof["outer"]] == pytest.approx(p.armature_outer)
    assert m.dof_armature[dof["inner"]] == pytest.approx(p.armature_inner)
    assert m.dof_frictionloss[dof["outer"]] == pytest.approx(p.frictionloss_outer)
    assert m.dof_frictionloss[dof["inner"]] == pytest.approx(p.frictionloss_inner)
    assert m.dof_damping[dof["outer"]] == pytest.approx(p.damping)
    assert m.dof_damping[dof["inner"]] == pytest.approx(p.damping)
    for n in ("outer", "inner"):
        assert m.dof_solref[dof[n]] == pytest.approx([p.friction_solref, 1.0])
        # Pinions: no drag, no friction; the servo torque reaches the frame through the gear.
        assert m.dof_damping[dof[n + "_pinion"]] == 0 and m.dof_frictionloss[dof[n + "_pinion"]] == 0
    # Decision A6: implicitfast integrator.
    assert m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert m.opt.timestep == pytest.approx(p.timestep)
    assert m.opt.noslip_iterations == p.noslip_iterations
    assert np.allclose(m.opt.gravity, [0, 0, -9.81])


def test_gears_couple_pinions_at_the_gear_ratio():
    p = MachineParams(gear_ratio=3.0)
    m = mujoco.MjModel.from_xml_string(build_mjcf(p))
    for n in ("outer", "inner"):
        a = m.actuator(n + "_servo")
        assert m.actuator_trnid[a.id, 0] == m.joint(n).id
        assert m.actuator_gear[a.id, 0] == pytest.approx(p.gear_ratio)
    for e in range(m.neq):
        assert m.eq_data[e, :5] == pytest.approx([0, -p.gear_ratio, 0, 0, 0])


def test_pinion_axes_parallel_to_their_frame_axes(md):
    m, d = md
    for n in ("outer", "inner"):
        a_frame = d.xaxis[m.joint(n).id]
        a_pinion = d.xaxis[m.joint(n + "_pinion").id]
        assert abs(abs(a_frame @ a_pinion) - 1) < 1e-9


# PLAN 2.4: which MuJoCo body carries each part.
PLAN_BODIES = {
    "outer_frame": "base", "stand_right": "base", "stand_left": "base", "outer_servo": "base",
    "inner_frame": "inner_frame", "gear_large_outer": "inner_frame", "inner_servo": "inner_frame",
    "platform": "platform", "gear_large_inner": "platform", "end_cap": "platform",
    "payload": "platform",
}


def test_parts_on_their_plan_bodies(md):
    m, _ = md
    for geom, body in PLAN_BODIES.items():
        assert m.body(m.geom_bodyid[m.geom(geom).id]).name == body, geom
    assert m.body(m.body_parentid[m.body("outer_pinion").id]).name == "base"
    assert m.body(m.body_parentid[m.body("inner_pinion").id]).name == "inner_frame"


# PLAN 2.4 [gemessen] STL volumes in cm^3.
VOLUME_CM3 = {"inner_frame": 25.5 + 4.8, "platform": 14.2 + 4.1 + 0.1}


@pytest.mark.parametrize("fill", [0.6, 0.3])
def test_body_masses_follow_density_payload_and_servo(fill):
    # Printed parts weigh density * volume; the inner servo and the payload add their own mass.
    p = MachineParams(fill_factor=fill, payload_mass=0.021, servo_mass=0.011)
    m = mujoco.MjModel.from_xml_string(build_mjcf(p))
    mass = lambda b: m.body_mass[m.body(b).id]  # noqa: E731
    assert mass("inner_frame") == pytest.approx(VOLUME_CM3["inner_frame"] * 1e-6 * p.density + p.servo_mass, rel=0.01)
    assert mass("platform") == pytest.approx(VOLUME_CM3["platform"] * 1e-6 * p.density + p.payload_mass, rel=0.01)


def test_rotating_mass(md):
    m, _ = md
    mass = m.body_subtreemass[m.body("inner_frame").id]
    assert 0.030 <= mass <= 0.080


def test_floor_at_zero(md):
    m, d = md
    zmin = np.inf
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        v = m.mesh_vert[a:a + n] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        zmin = min(zmin, v[:, 2].min())
    assert zmin == pytest.approx(0.0, abs=1e-4)
