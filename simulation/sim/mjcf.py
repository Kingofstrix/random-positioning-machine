"""Build the MuJoCo model (MJCF XML) of the RPM from the STL parts (docs/PLAN.md 6.1, 6.2).

Everything below `base` is written in STL coordinates (converted to m); `base` turns STL +Y
into world +Z and lifts the floor to z = 0.
"""

from sim import geometry as G
from sim.params import MachineParams

MM = 0.001


def _v(vals, scale=MM):
    return " ".join("%.6g" % (v * scale) for v in vals)


def _mesh_geoms(body: str) -> str:
    out = []
    for name, (_, b, pos, euler) in G.PARTS.items():
        if b == body:
            out.append(
                f'<geom name="{name}" type="mesh" mesh="{name}" pos="{_v(pos)}" euler="{_v(euler, 1)}"/>'
            )
    return "\n".join(out)


def _servo_box(name: str, center, along_x: bool, mass: float | None) -> str:
    l, w, h = G.SERVO_BOX
    # Outer servo: shaft along X, body length along Y. Inner servo: shaft along Y, length along X.
    half = (h / 2, l / 2, w / 2) if along_x else (l / 2, h / 2, w / 2)
    m = f' mass="{mass}"' if mass is not None else ""
    return f'<geom name="{name}" type="box" size="{_v(half)}" pos="{_v(center)}"{m} rgba="0.2 0.2 0.25 1"/>'


def _pinion(name: str, p: MachineParams) -> str:
    _, axis, pos, euler = G.PINIONS[name]
    return (
        f'<body name="{name}" pos="{_v(pos)}">'
        # No damping: on a pinion this light any drag would make the soft gear coupling lag.
        f'<joint name="{name}" type="hinge" axis="{_v(axis, 1)}" damping="0"/>'
        f'<geom type="mesh" mesh="pinion" euler="{_v(euler, 1)}" rgba="0.9 0.6 0.1 1"/>'
        f"</body>"
    )


def build_mjcf(p: MachineParams) -> str:
    stl = G.STL_DIR
    meshes = "\n".join(
        f'<mesh name="{n}" file="{stl / f}" scale="{MM} {MM} {MM}" inertia="exact"/>' for n, (f, *_) in G.PARTS.items()
    )
    meshes += f'\n<mesh name="pinion" file="{stl / G.PINION_STL}" scale="{MM} {MM} {MM}" inertia="exact"/>'
    floor_z = -G.FLOOR_Y * MM
    axis = _v(G.AXIS_POINT)
    sample = _v(tuple(a * MM + o for a, o in zip(G.AXIS_POINT, p.payload_offset)), 1)
    kv = p.servo_stall_torque / p.omega0
    tau = p.servo_stall_torque
    return f"""
<mujoco model="rpm">
  <compiler angle="degree" eulerseq="xyz"/>
  <option timestep="{p.timestep}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <default>
    <geom contype="0" conaffinity="0" density="{p.density}" rgba="0.85 0.85 0.8 1"/>
    <joint damping="{p.damping}" solreffriction="{p.friction_solref} 1"/>
  </default>
  <asset>
{meshes}
  </asset>
  <worldbody>
    <light pos="0 -0.6 0.8" dir="0 0.6 -0.8"/>
    <geom name="floor" type="plane" size="0.3 0.3 0.01" rgba="0.6 0.65 0.7 1"/>
    <body name="base" pos="0 0 {floor_z:.6g}" quat="0.7071068 0.7071068 0 0">
{_mesh_geoms("base")}
      {_servo_box("outer_servo", G.OUTER_SERVO_CENTER, True, None)}
      {_pinion("outer_pinion", p)}
      <body name="inner_frame">
        <joint name="outer" type="hinge" axis="1 0 0" pos="{axis}"
               armature="{p.armature_outer}" frictionloss="{p.frictionloss_outer}"/>
{_mesh_geoms("inner_frame")}
        {_servo_box("inner_servo", G.INNER_SERVO_CENTER, False, p.servo_mass)}
        {_pinion("inner_pinion", p)}
        <body name="platform">
          <joint name="inner" type="hinge" axis="0 1 0" pos="{axis}"
                 armature="{p.armature_inner}" frictionloss="{p.frictionloss_inner}"/>
{_mesh_geoms("platform")}
          <geom name="payload" type="cylinder" size="{p.payload_radius} {p.payload_half_height}"
                pos="{sample}" mass="{p.payload_mass}" rgba="0.6 0.8 1 0.6"/>
          <site name="sample" pos="{sample}" size="0.004"/>
        </body>
      </body>
    </body>
  </worldbody>
  <equality>
    <joint joint1="outer_pinion" joint2="outer" polycoef="0 {-p.gear_ratio} 0 0 0"/>
    <joint joint1="inner_pinion" joint2="inner" polycoef="0 {-p.gear_ratio} 0 0 0"/>
  </equality>
  <actuator>
    <velocity name="outer_servo" joint="outer" gear="{p.gear_ratio}" kv="{kv:.6g}"
              forcelimited="true" forcerange="{-tau} {tau}"/>
    <velocity name="inner_servo" joint="inner" gear="{p.gear_ratio}" kv="{kv:.6g}"
              forcelimited="true" forcerange="{-tau} {tau}"/>
  </actuator>
  <sensor>
    <accelerometer name="acc" site="sample"/>
    <gyro name="gyro" site="sample"/>
    <framequat name="quat" objtype="site" objname="sample"/>
    <jointpos name="q_outer" joint="outer"/>
    <jointpos name="q_inner" joint="inner"/>
    <jointvel name="w_outer" joint="outer"/>
    <jointvel name="w_inner" joint="inner"/>
    <actuatorfrc name="tau_outer" actuator="outer_servo"/>
    <actuatorfrc name="tau_inner" actuator="inner_servo"/>
  </sensor>
</mujoco>
"""
