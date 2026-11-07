"""Live viewer session without a display (docs/PLAN.md 6.7).

Everything except opening the window is tested here: command handling, real-time pacing with a
fake clock, the running residual g and the two arrows drawn at the sample.
"""

import copy
import io
import math
import threading

import mujoco
import numpy as np
import pytest

from rpmcore.config import CONFIG
from rpmcore.controller import RUNNING, STOPPED
from sim.metrics import G0
from sim.params import MachineParams
from sim.viewer import (
    ARROW_LENGTH_M,
    MAX_CATCHUP_TICKS,
    LiveSession,
    command_loop,
    draw_arrows,
    main,
    parse_args,
)

P = MachineParams()
FRAME_S = 1.0 / 60.0


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def make(autostart=False, mode="random", seed=1, clock=None):
    cfg = copy.deepcopy(CONFIG)
    cfg["autostart"] = autostart
    return LiveSession(P, cfg, mode, seed, clock=clock or FakeClock())


def run_wall(session, seconds, frame=FRAME_S):
    for _ in range(int(round(seconds / frame))):
        session.advance(frame)


def sample_site(session):
    m, d = session.machine.model, session.machine.data
    sid = m.site("sample").id
    return d.site_xpos[sid].copy(), d.site_xmat[sid].reshape(3, 3).copy()


# --- construction --------------------------------------------------------------------------


def test_autostart_follows_config():
    s = make(autostart=True, mode="clino3d")
    assert s.controller.state == RUNNING and s.controller.mode == "clino3d"
    s = make(autostart=False, mode="clino2d")
    assert s.controller.state == STOPPED and s.controller.mode == "clino2d"


def test_seed_goes_into_controller_and_cfg_is_not_mutated():
    cfg = copy.deepcopy(CONFIG)
    before = copy.deepcopy(cfg)
    s = LiveSession(P, cfg, "random", 7, clock=FakeClock())
    assert s.controller.cfg["seed"] == 7
    assert cfg == before


def test_session_owns_controller_and_machine():
    s = make()
    assert s.machine.params is P
    assert s.ticks == 0 and s.machine.data.time == 0.0


# --- commands ------------------------------------------------------------------------------


def test_submit_uses_the_rpmcore_protocol():
    s = make()
    assert s.submit("start clino2d") == "ok state=RUNNING mode=clino2d"
    assert s.submit("status").startswith("ok state=RUNNING mode=clino2d")
    assert s.submit("bogus").startswith("err unknown command")
    assert s.submit("   ") == ""


def test_start_clino2d_reaches_30_dps_after_5_s():
    s = make()
    s.submit("start clino2d")
    run_wall(s, 5.0)
    st = s.machine.state()
    assert abs(math.degrees(st["w_outer"]) - CONFIG["clino2d"]["outer_dps"]) < 2.0
    assert abs(math.degrees(st["w_inner"])) < 1.0
    # Real time: 5 s of wall time is 5 s of simulated time (to one control tick).
    assert abs(s.machine.data.time - 5.0) <= P.control_dt + 1e-9


def test_estop_from_another_thread_while_advancing():
    s = make(autostart=True)
    done = threading.Event()
    errors = []

    def spam():
        try:
            for i in range(200):
                s.submit("start clino3d" if i % 2 else "estop")
            s.submit("estop")
        except Exception as e:  # pragma: no cover - reported below
            errors.append(e)
        finally:
            done.set()

    t = threading.Thread(target=spam)
    t.start()
    while not done.is_set():
        s.advance(P.control_dt)
    t.join()
    run_wall(s, 0.5)
    assert errors == []
    assert s.controller.state == STOPPED
    assert s.submit("status").startswith("ok state=STOPPED")


# --- pacing --------------------------------------------------------------------------------


def test_advance_runs_ticks_for_elapsed_wall_time_and_keeps_remainder():
    s = make()
    assert s.advance(0.05) == 2          # 2.5 ticks owed -> 2 now
    assert s.advance(0.01) == 1          # the half tick carried over completes
    assert s.advance(0.0) == 0
    assert s.ticks == 3
    assert s.machine.data.time == pytest.approx(3 * P.control_dt)


def test_catchup_is_capped_and_the_stall_is_dropped():
    s = make()
    assert s.advance(10.0) == MAX_CATCHUP_TICKS
    # The rest of the stall is forgotten, not paid back as a burst later.
    assert s.advance(0.0) == 0
    assert s.advance(P.control_dt) == 1
    assert s.ticks == MAX_CATCHUP_TICKS + 1


def test_cap_is_small_but_covers_a_normal_frame():
    # A 60 Hz frame needs about one tick; a burst of hundreds must be impossible.
    assert 2 <= MAX_CATCHUP_TICKS <= 50


def test_poll_reads_the_injected_clock():
    clock = FakeClock()
    s = make(clock=clock)
    assert s.poll() == 0
    clock.now += 0.1
    assert s.poll() == 5
    clock.now += 0.03
    assert s.poll() == 1
    clock.now += 0.01
    assert s.poll() == 1
    clock.now += 1000.0
    assert s.poll() == MAX_CATCHUP_TICKS


def test_negative_wall_dt_is_ignored():
    s = make()
    assert s.advance(-1.0) == 0
    assert s.advance(P.control_dt) == 1


# --- residual g ----------------------------------------------------------------------------


def test_mean_before_any_tick_is_current_gravity():
    s = make()
    assert np.allclose(s.mean_g(), s.machine.state()["g_sample"])
    assert s.residual_g() == pytest.approx(1.0, abs=1e-3)


def test_running_mean_matches_the_samples():
    s = make()
    s.submit("start clino3d")
    samples = []
    for _ in range(30):
        s.advance(P.control_dt)
        samples.append(s.machine.state()["g_sample"])
    mean = np.mean(samples, axis=0)
    assert np.allclose(s.mean_g(), mean)
    assert s.residual_g() == pytest.approx(np.linalg.norm(mean) / G0)


def test_residual_drops_while_clino2d_runs():
    s = make()
    s.submit("start clino2d")
    run_wall(s, 15.0)
    assert s.residual_g() < 0.3


# --- arrows --------------------------------------------------------------------------------


def test_arrows_start_at_the_sample():
    s = make(autostart=True, mode="clino3d")
    run_wall(s, 2.0)
    pos, _ = sample_site(s)
    arrows = s.arrows()
    assert [a.name for a in arrows] == ["gravity", "mean"]
    for a in arrows:
        assert np.allclose(a.start, pos)
        assert a.rgba.shape == (4,)


def test_gravity_arrow_is_unit_length_and_points_down():
    s = make(autostart=True, mode="clino3d")
    run_wall(s, 2.0)
    g = s.arrows()[0]
    v = g.end - g.start
    assert np.linalg.norm(v) == pytest.approx(ARROW_LENGTH_M)
    assert np.allclose(v / ARROW_LENGTH_M, [0.0, 0.0, -1.0], atol=1e-6)


def test_mean_arrow_is_scaled_and_turns_with_the_sample():
    s = make(autostart=True, mode="clino3d")
    run_wall(s, 3.0)
    _, R = sample_site(s)
    mean = s.arrows()[1]
    v = mean.end - mean.start
    assert np.linalg.norm(v) == pytest.approx(ARROW_LENGTH_M * s.residual_g())
    # The mean lives in the sample frame; drawn in the world it rotates with the sample.
    want = R @ s.mean_g()
    assert np.allclose(v / np.linalg.norm(v), want / np.linalg.norm(want))
    assert np.linalg.norm(v) < ARROW_LENGTH_M


def test_both_arrows_coincide_before_the_machine_moves():
    s = make()
    g, mean = s.arrows()
    assert np.allclose(g.end, mean.end, atol=1e-6)


# --- drawing into a scene (no window, no GL context) ---------------------------------------


def test_draw_arrows_replaces_user_geoms():
    s = make(autostart=True, mode="clino3d")
    run_wall(s, 2.0)
    scn = mujoco.MjvScene(s.machine.model, maxgeom=10)
    scn.ngeom = 3
    draw_arrows(scn, s.arrows())
    assert scn.ngeom == 2
    for i in range(2):
        assert scn.geoms[i].type == mujoco.mjtGeom.mjGEOM_ARROW
    assert np.allclose(scn.geoms[0].pos, s.arrows()[0].start)
    draw_arrows(scn, s.arrows())
    assert scn.ngeom == 2


def test_draw_arrows_respects_maxgeom_and_skips_zero_length():
    s = make()
    scn = mujoco.MjvScene(s.machine.model, maxgeom=1)
    draw_arrows(scn, s.arrows())
    assert scn.ngeom == 1
    a = s.arrows()[0]
    a.end = a.start.copy()
    scn = mujoco.MjvScene(s.machine.model, maxgeom=4)
    draw_arrows(scn, [a])
    assert scn.ngeom == 0


# --- terminal ------------------------------------------------------------------------------


def test_command_loop_replies_and_stops_on_quit():
    s = make()
    out = []
    stop = threading.Event()
    command_loop(s, iter(["status\n", "", "start clino2d\n", "QUIT\n", "estop\n"]), out.append, stop)
    assert stop.is_set()
    assert out[0].startswith("ok state=STOPPED")
    assert out[1] == "ok state=RUNNING mode=clino2d"
    assert len(out) == 2                  # nothing after quit is executed
    assert s.controller.state == RUNNING


def test_command_loop_ends_quietly_at_eof():
    s = make()
    out = []
    stop = threading.Event()
    command_loop(s, iter(["help\n"]), out.append, stop)
    assert not stop.is_set()
    assert out[0].startswith("ok commands")


def test_status_line_has_state_and_residual():
    s = make(autostart=True, mode="clino2d")
    run_wall(s, 1.0)
    line = s.status_line()
    assert "state=RUNNING" in line and "mode=clino2d" in line
    assert ("residual_g=%.3f" % s.residual_g()) in line


def test_parse_args_defaults_and_choices():
    a = parse_args([])
    assert a.mode == CONFIG["default_mode"] and a.seed == CONFIG["seed"]
    a = parse_args(["--mode", "clino3d", "--seed", "4"])
    assert a.mode == "clino3d" and a.seed == 4
    with pytest.raises(SystemExit):
        parse_args(["--mode", "nope"])


# --- main loop with a fake viewer (no window is opened) ------------------------------------


class FakeViewer:
    """Stands in for the handle of mujoco.viewer.launch_passive; closes itself after `frames`."""

    def __init__(self, model, frames):
        self.user_scn = mujoco.MjvScene(model, maxgeom=10)
        self.frames = frames
        self.syncs = 0
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.closed = True

    def is_running(self):
        return self.syncs < self.frames

    def lock(self):
        return threading.Lock()

    def sync(self):
        self.syncs += 1


def run_main(monkeypatch, stdin_text, frames, argv):
    import mujoco.viewer

    made = []

    def fake_launch(model, data, **kw):
        made.append(FakeViewer(model, frames))
        return made[0]

    monkeypatch.setattr(mujoco.viewer, "launch_passive", fake_launch)
    monkeypatch.setattr("sys.stdin", io.StringIO(stdin_text))
    main(argv)
    return made[0]


def test_main_exits_when_the_viewer_closes(monkeypatch, capsys):
    v = run_main(monkeypatch, "", frames=5, argv=["--mode", "clino2d", "--seed", "2"])
    assert v.syncs == 5 and v.closed
    assert v.user_scn.ngeom == 2
    out = capsys.readouterr().out
    assert "mode=clino2d" in out and "residual_g=" in out


def test_main_exits_on_quit(monkeypatch, capsys):
    v = run_main(monkeypatch, "status\nquit\n", frames=10_000, argv=[])
    assert v.closed and v.syncs < 10_000
    assert "ok state=" in capsys.readouterr().out
