"""Closed loop: the rpmcore controller drives the simulated machine (docs/PLAN.md 6.5).

python -m sim.run --mode random --minutes 30 --seed 1 --out results/
"""

import argparse
import copy
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import sim  # noqa: F401  (puts firmware/ on sys.path)
from rpmcore.config import CONFIG
from rpmcore.controller import Controller
from sim.machine import SimMachine
from sim.metrics import extra_accel, residual_g, speed_tracking, sphere_coverage, windowed_residual
from sim.params import MachineParams


@dataclass
class SimLog:
    t: np.ndarray
    q_outer: np.ndarray
    q_inner: np.ndarray
    w_dps: np.ndarray          # (N, 2) actual frame speed, columns inner, outer
    cmd_dps: np.ndarray        # (N, 2) ramped setpoint from the controller
    pulses: np.ndarray         # (N, 2) us, 0 = PWM off
    g_sample: np.ndarray       # (N, 3)
    acc_sample: np.ndarray     # (N, 3)
    meta: dict = field(default_factory=dict)

    def save(self, path: Path) -> None:
        arrays = {k: v for k, v in self.__dict__.items() if isinstance(v, np.ndarray)}
        np.savez_compressed(path, **arrays, meta=json.dumps(self.meta))


def simulate(mode: str, minutes: float, seed: int, params: MachineParams | None = None,
             cfg: dict | None = None, log_hz: float = 50.0) -> SimLog:
    p = params or MachineParams()
    cfg = copy.deepcopy(cfg or CONFIG)
    cfg["seed"] = seed
    ctrl = Controller(cfg)
    ctrl.start(mode)
    machine = SimMachine(p)
    n_ticks = int(round(minutes * 60.0 / p.control_dt))
    every = max(1, int(round(1.0 / (log_hz * p.control_dt))))
    rows = []
    for i in range(n_ticks):
        inner_us, outer_us = ctrl.tick(p.control_dt)
        machine.step(inner_us, outer_us)
        if i % every == 0:
            s = machine.state()
            st = ctrl.status()
            rows.append((s["t"], s["q_outer"], s["q_inner"], s["w_inner"], s["w_outer"],
                         st["inner_dps"], st["outer_dps"], inner_us or 0, outer_us or 0,
                         *s["g_sample"], *s["acc_sample"]))
    a = np.array(rows)
    return SimLog(
        t=a[:, 0], q_outer=a[:, 1], q_inner=a[:, 2],
        w_dps=np.degrees(a[:, 3:5]), cmd_dps=a[:, 5:7], pulses=a[:, 7:9].astype(int),
        g_sample=a[:, 9:12], acc_sample=a[:, 12:15],
        meta={"mode": mode, "minutes": minutes, "seed": seed, "log_hz": log_hz,
              "params": {k: v for k, v in p.__dict__.items()}},
    )


def summarize(log: SimLog) -> dict:
    hz = log.meta["log_hz"]
    frac, nonuni = sphere_coverage(log.g_sample)
    mx, rms = extra_accel(log.acc_sample, log.g_sample)
    out = {
        "residual_g": residual_g(log.g_sample),
        "coverage": frac,
        "coverage_nonuniformity": nonuni,
        "extra_accel_max_g": mx,
        "extra_accel_rms_g": rms,
        "tracking_rms_dps": dict(zip(("inner", "outer"), speed_tracking(log.cmd_dps, log.w_dps).tolist())),
    }
    for w in (60, 600):
        med, p95 = windowed_residual(log.g_sample, w, hz)
        if not math.isnan(med):
            out["residual_g_%ds" % w] = {"median": med, "p95": p95}
    return out


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mode", default="random", choices=["random", "clino3d", "clino2d"])
    ap.add_argument("--minutes", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", type=Path, default=Path("results"))
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    log = simulate(a.mode, a.minutes, a.seed)
    name = "%s_s%d" % (a.mode, a.seed)
    log.save(a.out / (name + ".npz"))
    summary_path = a.out / "summary.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    summary[name] = summarize(log)
    summary_path.write_text(json.dumps(summary, indent=2))
    print(json.dumps({name: summary[name]}, indent=2))


if __name__ == "__main__":
    main()
