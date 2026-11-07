"""Where the printed parts sit relative to each other (docs/PLAN.md 2, sim/geometry.py).

Every check measures the STL meshes as MuJoCo loaded them (`mesh_vert`), not the constants in
`sim/geometry.py`, so a wrong constant cannot hide behind a test that repeats it. Lengths are in
mm in the STL frame of the body the checked part hangs on (base, inner frame).
"""

import mujoco
import numpy as np
import pytest

from sim import geometry as G
from sim.mjcf import build_mjcf
from sim.params import MachineParams

P = MachineParams()
AXIS = np.array(G.AXIS_POINT)
TOL = 0.2               # mm, how far a face may be from the face it should touch
NEAR_AXIS = 12.0        # mm, radius around an axis that holds the bearings and shaft ends
TOOTH_DEPTH = 2.0       # mm, radial band below the tip circle that counts as teeth
MIN_FACE_WIDTH = 2.0    # mm, axial overlap a gear pair needs to mesh
MAX_TOOTH_OVERLAP = 0.05  # mm^2, tooth interpenetration in a gear section (raster noise)
BAR_SEARCH = 30.0       # mm, how far around a servo box to look for its bar and posts
POST_DEPTH = 1.0        # mm, depth of the post faces that hold the servo flange
RASTER = 0.05           # mm, raster step for the section overlap
RASTER_HALF_WIDTH = 4.0  # mm, half the raster window around the pitch point (> 1 tooth pitch)


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_string(build_mjcf(P))


def posed(m, q_outer=0.0, q_inner=0.0):
    """Data at the given frame angles, pinions turned as the gear equality would turn them."""
    d = mujoco.MjData(m)
    d.joint("outer").qpos = q_outer
    d.joint("inner").qpos = q_inner
    d.joint("outer_pinion").qpos = -P.gear_ratio * q_outer
    d.joint("inner_pinion").qpos = -P.gear_ratio * q_inner
    mujoco.mj_kinematics(m, d)
    return d


@pytest.fixture(scope="module")
def d0(model):
    return posed(model)


def to_frame(m, d, world, frame):
    b = m.body(frame).id
    return (world - d.xpos[b]) @ d.xmat[b].reshape(3, 3) / 0.001


def gid(m, name):
    """Geom id by geom name, or the single geom of a body (the pinions)."""
    try:
        return m.geom(name).id
    except KeyError:
        b = m.body(name).id
        (g,) = [i for i in range(m.ngeom) if m.geom_bodyid[i] == b]
        return g


def verts(m, d, name, frame="base"):
    g = gid(m, name)
    mid = m.geom_dataid[g]
    a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
    world = d.geom_xpos[g] + m.mesh_vert[a:a + n] @ d.geom_xmat[g].reshape(3, 3).T
    return to_frame(m, d, world, frame)


def tris(m, d, name, frame="base"):
    mid = m.geom_dataid[gid(m, name)]
    a, n = m.mesh_faceadr[mid], m.mesh_facenum[mid]
    return verts(m, d, name, frame)[m.mesh_face[a:a + n]]


def geom_points(m, d, g, frame):
    """Mesh vertices, or a grid over the bounding box of a primitive geom (conservative)."""
    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
        return verts(m, d, g, frame)
    s = m.geom_size[g]
    half = s if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX else (s[0], s[0], s[1])   # box, cylinder
    grid = np.stack(np.meshgrid(*[np.linspace(-h, h, 7) for h in half]), -1).reshape(-1, 3)
    return to_frame(m, d, d.geom_xpos[g] + grid @ d.geom_xmat[g].reshape(3, 3).T, frame)


def box_bounds(m, d, name, frame):
    g = m.geom(name).id
    assert m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
    c = to_frame(m, d, d.geom_xpos[g], frame)
    rot = d.geom_xmat[g].reshape(3, 3).T @ d.xmat[m.body(frame).id].reshape(3, 3)
    assert np.allclose(rot, np.eye(3), atol=1e-9)        # axis-aligned in the frame
    h = m.geom_size[g] / 0.001
    return c - h, c + h


def radius(v, axis):
    """Distance of points from the rotation axis (0 = outer along X, 1 = inner along Y)."""
    other = [i for i in range(3) if i != axis]
    return np.hypot(*(v[:, i] - AXIS[i] for i in other))


def subtree_geoms(m, body):
    root = m.body(body).id
    inside = {root}
    for b in range(root + 1, m.nbody):
        if m.body_parentid[b] in inside:
            inside.add(b)
    return [g for g in range(m.ngeom) if m.geom_bodyid[g] in inside]


def tip_centre(v, axis):
    """Gear centre in the plane normal to `axis`: mean of the tooth-tip vertices.

    The teeth repeat around the axis, so their tips average to the centre even for an odd tooth
    count, where the bounding box would be off by half a tooth.
    """
    other = [i for i in range(3) if i != axis]
    mid = (v.min(0) + v.max(0)) / 2
    r = np.hypot(*(v[:, i] - mid[i] for i in other))
    return v[r > r.max() - 0.3][:, other].mean(0)


def teeth_span(v, axis, centre):
    other = [i for i in range(3) if i != axis]
    r = np.hypot(*(v[:, i] - centre[k] for k, i in enumerate(other)))
    t = v[r > r.max() - TOOTH_DEPTH][:, axis]
    return t.min(), t.max()


# --- outer pair: gear on the inner frame's -X stub, between the frames ---------------------------


def test_outer_gear_on_inner_frame_stub_against_outer_frame(model, d0):
    """Assembly guide p5: the gear goes on the inner frame's square stub, the outer frame on top.

    Its collar bears on the outer frame's inner face and its disc sits on the stub, so the gear
    lies between the two frames and the frame bar hides its middle (guide p5-p7).
    """
    of, inf = verts(model, d0, "outer_frame"), verts(model, d0, "inner_frame")
    gear = verts(model, d0, "gear_large_outer")
    face = of[(of[:, 0] < 0) & (radius(of, 0) < NEAR_AXIS)][:, 0].max()
    stub = inf[(inf[:, 0] < 0) & (radius(inf, 0) < NEAR_AXIS)]
    bar = inf[(inf[:, 0] < 0) & (radius(inf, 0) > NEAR_AXIS) & (np.abs(inf[:, 1]) < 40)]
    teeth = teeth_span(gear, 0, AXIS[1:])
    assert gear[:, 0].min() == pytest.approx(face, abs=TOL)
    assert stub[:, 0].min() <= teeth[0]                  # disc on the stub
    assert teeth[1] < bar[:, 0].min()                     # teeth clear of the inner frame bar


# --- inner pair: gear on the platform's angled +Y shaft end, end cap on the -Y end -------------


def test_inner_gear_on_angled_shaft_end(model, d0):
    """Guide step 1: the gear goes on the platform's angled square end, against the frame."""
    inf, plat = verts(model, d0, "inner_frame"), verts(model, d0, "platform")
    gear = verts(model, d0, "gear_large_inner")
    face = inf[(inf[:, 1] > 0) & (radius(inf, 1) < NEAR_AXIS)][:, 1].max()
    shaft_end = plat[:, 1].max()
    assert gear[:, 1].min() == pytest.approx(face, abs=TOL)
    assert gear[:, 1].max() == pytest.approx(shaft_end, abs=TOL)


def test_inner_gear_bore_turned_onto_the_diamond(model, d0):
    """The shaft end is a square turned 45 deg; the gear's square bore must be turned the same."""
    plat = verts(model, d0, "platform")
    gear = verts(model, d0, "gear_large_inner")
    end = plat[:, 1].max()

    def l1(v):
        return np.abs(v[:, 0] - AXIS[0]) + np.abs(v[:, 2] - AXIS[2])

    diamond = plat[(plat[:, 1] > end - 1.0) & (radius(plat, 1) < 4)]
    bore = gear[(gear[:, 1] > end - 1.0) & (radius(gear, 1) < 4)]
    assert len(bore) >= 4
    # Axis-aligned, the bore corners would reach |x| + |z| = 3.65 mm, past the 2.5 mm diamond.
    assert l1(bore).max() <= l1(diamond).max() + TOL
    assert l1(bore).min() >= l1(diamond).max() - TOL


def test_end_cap_on_shaft_end(model, d0):
    """The end cap holds the platform against the frame's -Y face, on the shaft's square end."""
    inf, plat = verts(model, d0, "inner_frame"), verts(model, d0, "platform")
    cap = verts(model, d0, "end_cap")
    face = inf[(inf[:, 1] < 0) & (radius(inf, 1) < NEAR_AXIS)][:, 1].min()
    assert cap[:, 1].max() == pytest.approx(face, abs=TOL)
    assert cap[:, 1].min() >= plat[:, 1].min() - 1e-6       # on the shaft, not past its tip


# --- gear meshing -------------------------------------------------------------------------------

PAIRS = [
    # gear geom, pinion body, axis, frame the pair is fixed in
    ("gear_large_outer", "outer_pinion", 0, "base"),
    ("gear_large_inner", "inner_pinion", 1, "inner_frame"),
]


@pytest.mark.parametrize("gear, pinion, axis, frame", PAIRS)
def test_pinion_centre_distance(model, d0, gear, pinion, axis, frame):
    g = verts(model, d0, gear, frame)
    p = verts(model, d0, pinion, frame)
    cg, cp = tip_centre(g, axis), tip_centre(p, axis)
    assert cg == pytest.approx(np.delete(AXIS, axis), abs=0.05)
    assert np.linalg.norm(cp - cg) == pytest.approx(G.GEAR_CENTER_DIST, abs=1.0)


@pytest.mark.parametrize("gear, pinion, axis, frame", PAIRS)
def test_pinion_teeth_overlap_gear_teeth_axially(model, d0, gear, pinion, axis, frame):
    g = verts(model, d0, gear, frame)
    p = verts(model, d0, pinion, frame)
    tg = teeth_span(g, axis, tip_centre(g, axis))
    tp = teeth_span(p, axis, tip_centre(p, axis))
    assert min(tg[1], tp[1]) - max(tg[0], tp[0]) >= MIN_FACE_WIDTH


def section(t, axis, val):
    """Cut triangles with the plane x[axis] = val; returns 2D segments in the other two axes."""
    dd = t[:, :, axis] - val
    keep = (dd.min(1) < 0) & (dd.max(1) > 0)
    t, dd = t[keep], dd[keep]
    pts, cut = [], []
    for i, j in ((0, 1), (1, 2), (2, 0)):
        u = dd[:, i] / np.where(dd[:, i] == dd[:, j], 1.0, dd[:, i] - dd[:, j])
        pts.append(t[:, i] + u[:, None] * (t[:, j] - t[:, i]))
        cut.append((dd[:, i] < 0) != (dd[:, j] < 0))
    pts, cut = np.stack(pts, 1), np.stack(cut, 1)
    assert (cut.sum(1) == 2).all()
    other = [i for i in range(3) if i != axis]
    return pts[cut].reshape(-1, 2, 3)[:, :, other]


def inside(seg, pts):
    """Even-odd rule: is each point inside the closed outline made of `seg`?"""
    # Only segments a +x ray from some point can cross matter; dropping the rest keeps this fast.
    lo, hi = pts.min(0), pts.max(0)
    seg = seg[(seg[:, :, 1].max(1) >= lo[1]) & (seg[:, :, 1].min(1) <= hi[1]) & (seg[:, :, 0].max(1) >= lo[0])]
    (x1, y1), (x2, y2) = seg[:, 0].T, seg[:, 1].T
    crosses = (y1[:, None] > pts[:, 1]) != (y2[:, None] > pts[:, 1])
    dy = np.where(y2 == y1, 1.0, y2 - y1)
    xi = x1[:, None] + (pts[:, 1] - y1[:, None]) * ((x2 - x1) / dy)[:, None]
    return (crosses & (pts[:, 0] < xi)).sum(0) % 2 == 1


@pytest.mark.parametrize("q", [(0.0, 0.0), (1.1, -2.0)])
@pytest.mark.parametrize("gear, pinion, axis, frame", PAIRS)
def test_gear_teeth_do_not_interpenetrate(model, gear, pinion, axis, frame, q):
    """The pinions turn exactly 4x their frame, so a tooth clash at one angle shows at every angle."""
    d = posed(model, *q)
    g, p = tris(model, d, gear, frame), tris(model, d, pinion, frame)
    gv, pv = g.reshape(-1, 3), p.reshape(-1, 3)
    cg, cp = tip_centre(gv, axis), tip_centre(pv, axis)
    tg, tp = teeth_span(gv, axis, cg), teeth_span(pv, axis, cp)
    lo, hi = max(tg[0], tp[0]), min(tg[1], tp[1])
    assert hi > lo
    plane = (lo + hi) / 2                                 # mid of the common face width
    # Pitch point: where the pitch circles touch, on the line between the centres.
    contact = cg + (cp - cg) * P.gear_ratio / (P.gear_ratio + 1)
    xs = np.arange(-RASTER_HALF_WIDTH, RASTER_HALF_WIDTH, RASTER)
    grid = np.stack(np.meshgrid(xs, xs), -1).reshape(-1, 2) + contact
    both = inside(section(g, axis, plane), grid) & inside(section(p, axis, plane), grid)
    assert both.sum() * RASTER**2 <= MAX_TOOTH_OVERLAP


# --- stands -------------------------------------------------------------------------------------


def hole_centres(v, guesses, window=3.0):
    """Centres of round holes along X near `guesses` (y, z), by a least-squares circle fit."""
    out = []
    for c in guesses:
        for _ in range(3):
            q = v[np.hypot(v[:, 1] - c[0], v[:, 2] - c[1]) < window][:, 1:]
            A = np.c_[2 * q, np.ones(len(q))]
            sol = np.linalg.lstsq(A, (q**2).sum(1), rcond=None)[0]
            c, r = sol[:2], np.sqrt(sol[2] + sol[:2] @ sol[:2])
            window = r + 0.3
        assert np.abs(np.hypot(q[:, 0] - c[0], q[:, 1] - c[1]) - r).max() < 0.05   # a clean circle
        out.append(c)
    return np.array(out)


@pytest.mark.parametrize("stand, side", [("stand_right", 1), ("stand_left", -1)])
def test_stand_holes_line_up_with_frame_holes(model, d0, stand, side):
    """Guide step 5: each stand is bolted to the outer frame through two holes (M3x6)."""
    of, st = verts(model, d0, "outer_frame"), verts(model, d0, stand)
    frame_face = (side * of[:, 0]).max()
    assert (side * st[:, 0]).min() == pytest.approx(frame_face, abs=TOL)   # flat on the frame
    # Rough hole positions between both parts; the fit then finds the real centres in each.
    guess = [(-74.4, AXIS[2]), (-49.4, AXIS[2])]
    frame_bar = of[side * of[:, 0] > frame_face - 5]
    np.testing.assert_allclose(hole_centres(st, guess), hole_centres(frame_bar, guess), atol=0.1)


# --- servos -------------------------------------------------------------------------------------

SERVOS = [
    # servo box, its frame, the frame's own geom, pinion, shaft axis, lateral axis (box length)
    ("outer_servo", "base", "outer_frame", "outer_pinion", 0, 1),
    ("inner_servo", "inner_frame", "inner_frame", "inner_pinion", 1, 0),
]
MOVING = {"outer_servo": ("inner_frame", 0), "inner_servo": ("platform", 1)}


@pytest.mark.parametrize("servo, frame, frame_geom, pinion, axis, lat", SERVOS)
def test_servo_box_outside_swept_volume(model, d0, servo, frame, frame_geom, pinion, axis, lat):
    """No part that turns past the servo may reach into its box, at any angle."""
    lo, hi = box_bounds(model, d0, servo, frame)
    body, rot_axis = MOVING[servo]
    other = [i for i in range(3) if i != rot_axis]
    # Closest point of the box to the rotation axis, in the plane normal to it.
    near = np.clip(AXIS[other], lo[other], hi[other])
    r_box = np.hypot(*(near - AXIS[other]))
    pts = np.vstack([geom_points(model, d0, g, frame) for g in subtree_geoms(model, body)])
    pts = pts[(pts[:, rot_axis] >= lo[rot_axis]) & (pts[:, rot_axis] <= hi[rot_axis])]
    r_moving = radius(pts, rot_axis).max() if len(pts) else 0.0
    assert r_moving < r_box


@pytest.mark.parametrize("servo, frame, frame_geom, pinion, axis, lat", SERVOS)
def test_servo_box_ends_at_its_pinion(model, d0, servo, frame, frame_geom, pinion, axis, lat):
    """The box includes the output shaft, so it reaches the pinion it drives."""
    lo, hi = box_bounds(model, d0, servo, frame)
    p = verts(model, d0, pinion, frame)
    assert hi[axis] == pytest.approx(p[:, axis].min(), abs=TOL)


@pytest.mark.parametrize("servo, frame, frame_geom, pinion, axis, lat", SERVOS)
def test_servo_sits_in_its_slot(model, d0, servo, frame, frame_geom, pinion, axis, lat):
    """Guide steps 2 and 4: the servo passes through the frame bar between its two screw posts."""
    lo, hi = box_bounds(model, d0, servo, frame)
    centre = (lo + hi) / 2
    up = 3 - axis - lat                                  # the remaining axis (STL Z)
    t = tris(model, d0, frame_geom, frame)

    def cut(normal):
        """Section through the box middle; returns columns (shaft axis, the other axis)."""
        seg = section(t, normal, centre[normal]).reshape(-1, 2)
        other = [i for i in range(3) if i != normal]
        return seg[:, other.index(axis)], seg[:, other.index(3 - axis - normal)]

    # The bar: frame material under the box, cut along the shaft.
    a, u = cut(lat)
    near = (u < lo[up]) & (a > lo[axis] - BAR_SEARCH) & (a < hi[axis] + BAR_SEARCH)
    bar_lo, bar_hi = a[near].min(), a[near].max()
    assert lo[axis] < bar_lo and hi[axis] > bar_hi       # the box passes through the bar
    # The posts, cut at shaft height on the bar face the body sits against. Further on, the outer
    # frame narrows the slot to the shaft boss, which the plain box does not model.
    body_face = bar_lo if centre[axis] < bar_lo else bar_hi
    a, l = cut(up)
    post = (np.abs(a - body_face) <= POST_DEPTH) & (np.abs(l - centre[lat]) < BAR_SEARCH)
    below, above = l[post & (l < centre[lat])], l[post & (l > centre[lat])]
    assert len(below) and len(above)
    assert below.max() <= lo[lat] and hi[lat] <= above.min()
