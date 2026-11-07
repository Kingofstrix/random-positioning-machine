"""Interactive MuJoCo viewer with the live controller (docs/PLAN.md 6.7).

python -m sim.viewer --mode random --seed 1 --speed 10

Commands typed in the terminal go through `rpmcore.commands.handle`, the same protocol as on
the Pico's USB serial; `quit` closes the viewer. Two arrows are drawn at the sample: the current
gravity direction (always straight down) and the cumulative mean of gravity in the sample frame,
scaled by the residual g, so it should shrink while the machine runs.

All logic lives in `LiveSession` and the helper functions, so it is tested without a window.
"""

import argparse
import copy
import math
import sys
import threading
import time
from dataclasses import dataclass

import mujoco
import numpy as np

import sim  # noqa: F401  (puts firmware/ on sys.path)
from rpmcore import commands
from rpmcore.config import CONFIG
from rpmcore.controller import Controller
from sim.machine import SimMachine
from sim.metrics import G0
from sim.params import MachineParams

# Display settings, not physics, so they live here and not in params.py.
# A full 1 g arrow should stick out of the frames (~0.1 m across) without dwarfing them.
ARROW_LENGTH_M = 0.08
ARROW_WIDTH_M = 0.004
GRAVITY_RGBA = (0.9, 0.2, 0.2, 1.0)
MEAN_RGBA = (0.2, 0.8, 0.3, 1.0)
# Often enough to follow a run, rarely enough that typed commands stay readable.
STATUS_EVERY_S = 3.0

# Most control ticks one advance() may run. After a stall (window drag, GC pause, debugger)
# the simulation falls behind real time instead of fast-forwarding through hundreds of ticks:
# a burst would freeze the window further and look like a jump. 10 ticks = 0.2 s at 20 ms,
# far more than one 60 Hz frame needs.
MAX_CATCHUP_TICKS = 10

# Time lapse: sim seconds per wall second. Headless the sim reaches about 60x real time on a
# laptop; above ~50x the window would fall behind anyway, so faster requests are refused.
MAX_SPEED = 50.0
SPEED_WORD = "speed"   # handled by the viewer itself, not part of the device protocol

QUIT_WORDS = ("quit", "exit")

# Wall time is kept in integer nanoseconds so leftover fractions of a tick never round away.
_NS_PER_S = 1_000_000_000


@dataclass
class Arrow:
    name: str
    start: np.ndarray       # world coordinates, m
    end: np.ndarray
    rgba: np.ndarray


class LiveSession:
    """Controller + simulated machine paced by wall time; safe to command from another thread."""

    def __init__(self, params: MachineParams, cfg: dict, mode: str, seed: int, clock=time.monotonic,
                 speed: float = 1.0):
        self.params = params
        self.speed = 1.0
        self.set_speed(speed)
        cfg = copy.deepcopy(cfg)
        cfg["seed"] = seed
        self.controller = Controller(cfg)
        if cfg["autostart"]:
            self.controller.start(mode)
        else:
            self.controller.set_mode(mode)
        self.machine = SimMachine(params)
        self.ticks = 0
        self._lock = threading.Lock()
        self._clock = clock
        self._last = clock()
        self._tick_ns = int(round(params.control_dt * _NS_PER_S))
        self._backlog_ns = 0
        self._g_sum = np.zeros(3)
        self._g_count = 0

    def submit(self, line: str) -> str:
        with self._lock:
            return commands.handle(self.controller, line)

    def set_speed(self, value) -> None:
        """Time-lapse factor; raises ValueError and keeps the old value if out of range."""
        try:
            v = float(value)
        except (TypeError, ValueError):
            raise ValueError("speed must be a number") from None
        if not (math.isfinite(v) and 0 < v <= MAX_SPEED):
            raise ValueError("speed must be in (0, %g]" % MAX_SPEED)
        self.speed = v

    def poll(self) -> int:
        """Advance by the wall time since the last poll; returns the ticks run."""
        now = self._clock()
        dt, self._last = now - self._last, now
        return self.advance(dt)

    def advance(self, wall_dt: float) -> int:
        """Run the control ticks that `wall_dt` seconds of real time are worth (capped)."""
        self._backlog_ns += max(0, int(round(wall_dt * self.speed * _NS_PER_S)))
        n = self._backlog_ns // self._tick_ns
        # The cap scales with the speed, or it would silently limit the time lapse itself.
        cap = math.ceil(MAX_CATCHUP_TICKS * self.speed)
        if n > cap:
            n = cap
            self._backlog_ns = 0
        else:
            self._backlog_ns -= n * self._tick_ns
        dt = self.params.control_dt
        for _ in range(n):
            with self._lock:
                inner_us, outer_us = self.controller.tick(dt)
            self.machine.step(inner_us, outer_us)
            self._g_sum += self.machine.state()["g_sample"]
            self._g_count += 1
        self.ticks += n
        return n

    def mean_g(self) -> np.ndarray:
        """Cumulative mean of gravity in the sample frame (m/s^2)."""
        if self._g_count == 0:
            # The mean over no samples is undefined; the current vector is its natural limit.
            return self.machine.state()["g_sample"]
        return self._g_sum / self._g_count

    def residual_g(self) -> float:
        return float(np.linalg.norm(self.mean_g()) / G0)

    def arrows(self) -> list[Arrow]:
        g_sample = self.machine.state()["g_sample"]   # also refreshes the site pose
        pos, R = self.machine.sample_pose()
        g_world = R @ g_sample
        # Full arrow = 1 g, so the mean arrow's length reads directly as the residual g.
        mean_world = R @ self.mean_g() * (ARROW_LENGTH_M / G0)
        return [
            Arrow("gravity", pos.copy(), pos + g_world / np.linalg.norm(g_world) * ARROW_LENGTH_M,
                  np.array(GRAVITY_RGBA, dtype=np.float32)),
            Arrow("mean", pos.copy(), pos + mean_world, np.array(MEAN_RGBA, dtype=np.float32)),
        ]

    def status_line(self) -> str:
        with self._lock:
            s = self.controller.status()
        return "state=%s mode=%s inner_dps=%.1f outer_dps=%.1f t=%.1f s residual_g=%.3f speed=%gx" % (
            s["state"], s["mode"], s["inner_dps"], s["outer_dps"], self.machine.data.time, self.residual_g(),
            self.speed)


def draw_arrows(scn: mujoco.MjvScene, arrows: list[Arrow]) -> None:
    """Replace the user geoms of `scn` with the arrows (zero-length ones are skipped)."""
    scn.ngeom = 0
    for a in arrows:
        if scn.ngeom >= scn.maxgeom:
            break
        if np.allclose(a.start, a.end):
            continue
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).ravel(), a.rgba)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, ARROW_WIDTH_M, a.start, a.end)
        scn.ngeom += 1


def command_loop(session: LiveSession, lines, write, stop: threading.Event) -> None:
    """Feed terminal lines to the session until `quit` (sets `stop`) or end of input."""
    for line in lines:
        words = line.strip().lower().split()
        if words and words[0] in QUIT_WORDS:
            stop.set()
            return
        if words and words[0] == SPEED_WORD:
            reply = _speed_command(session, words[1:])
        else:
            reply = session.submit(line)
        if reply:
            write(reply)


def _speed_command(session: LiveSession, args: list[str]) -> str:
    if len(args) > 1:
        return "err usage: speed [factor]"
    if args:
        try:
            session.set_speed(args[0])
        except ValueError as e:
            return "err %s" % e
    return "ok speed=%g" % session.speed


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default=CONFIG["default_mode"], choices=["random", "clino3d", "clino2d"])
    ap.add_argument("--seed", type=int, default=CONFIG["seed"])
    ap.add_argument("--speed", type=float, default=1.0,
                    help="time lapse, sim seconds per wall second (also 'speed <x>' while running)")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    import mujoco.viewer  # only here: needs a display, tests import this module without one

    a = parse_args(argv)
    session = LiveSession(MachineParams(), CONFIG, a.mode, a.seed, speed=a.speed)
    stop = threading.Event()
    reader = threading.Thread(
        target=command_loop, args=(session, sys.stdin, lambda s: print(s, flush=True), stop), daemon=True)
    print("type commands (help, start, stop, estop, ..., speed <x>), 'quit' to exit", flush=True)
    print(session.status_line(), flush=True)
    m, d = session.machine.model, session.machine.data
    with mujoco.viewer.launch_passive(m, d) as v:
        reader.start()
        session.poll()  # drop the window start-up time
        next_status = time.monotonic() + STATUS_EVERY_S
        while v.is_running() and not stop.is_set():
            session.poll()
            with v.lock():
                draw_arrows(v.user_scn, session.arrows())
            v.sync()
            if time.monotonic() >= next_status:
                print(session.status_line(), flush=True)
                next_status += STATUS_EVERY_S
            time.sleep(session.params.control_dt)
    print(session.status_line(), flush=True)


if __name__ == "__main__":
    main()
