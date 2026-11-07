"""The simulated machine: MuJoCo model + data + the two servos (docs/PLAN.md 6.4)."""

import mujoco
import numpy as np

from sim.mjcf import build_mjcf
from sim.params import MachineParams
from sim.servo_model import u_from_pulse


class SimMachine:
    def __init__(self, params: MachineParams | None = None):
        self.params = params or MachineParams()
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(self.params))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self._act = {"inner": m.actuator("inner_servo").id, "outer": m.actuator("outer_servo").id}
        self._site = m.site("sample").id
        self._sens = {n: slice(m.sensor_adr[m.sensor(n).id], m.sensor_adr[m.sensor(n).id] + m.sensor_dim[m.sensor(n).id])
                      for n in ("acc", "gyro", "q_outer", "q_inner", "w_outer", "w_inner", "tau_outer", "tau_inner")}
        mujoco.mj_forward(self.model, self.data)

    def step(self, inner_us: int | None, outer_us: int | None, duration: float | None = None) -> None:
        """Hold both pulses for `duration` seconds (one control tick by default). None = PWM off."""
        p = self.params
        d = self.data
        # The velocity actuator realises tau = tau_stall * (u - omega/omega0) with ctrl = u*omega0.
        d.ctrl[self._act["inner"]] = u_from_pulse(inner_us, p) * p.omega0
        d.ctrl[self._act["outer"]] = u_from_pulse(outer_us, p) * p.omega0
        n = max(1, int(round((p.control_dt if duration is None else duration) / p.timestep)))
        mujoco.mj_step(self.model, d, nstep=n)

    def state(self) -> dict:
        m, d = self.model, self.data
        # Sensors are computed at the start of mj_step; refresh them for the state after it.
        mujoco.mj_forward(m, d)
        s = d.sensordata
        R = d.site_xmat[self._site].reshape(3, 3)
        return {
            "t": d.time,
            "q_outer": float(s[self._sens["q_outer"]][0]),
            "q_inner": float(s[self._sens["q_inner"]][0]),
            "w_outer": float(s[self._sens["w_outer"]][0]),
            "w_inner": float(s[self._sens["w_inner"]][0]),
            "g_sample": R.T @ m.opt.gravity,
            "acc_sample": s[self._sens["acc"]].copy(),
            "gyro": s[self._sens["gyro"]].copy(),
            # actuatorfrc is the force at the actuator, i.e. torque at the servo shaft.
            "tau_outer": float(s[self._sens["tau_outer"]][0]),
            "tau_inner": float(s[self._sens["tau_inner"]][0]),
        }

    @property
    def q(self) -> np.ndarray:
        return np.array([self.data.qpos[self.model.jnt_qposadr[self.model.joint(n).id]] for n in ("inner", "outer")])
