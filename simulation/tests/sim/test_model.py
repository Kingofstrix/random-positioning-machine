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
