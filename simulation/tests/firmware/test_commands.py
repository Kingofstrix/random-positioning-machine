import copy
import random

import pytest

from rpmcore.commands import execute, handle, parse
from rpmcore.controller import CAL, RUNNING, STOPPED, STOPPING, Controller

DT = 0.02


@pytest.fixture
def c(cfg):
    return Controller(cfg)


def snapshot(ctrl):
    return copy.deepcopy(ctrl.cfg), ctrl.status(), ctrl.mode, ctrl.state


def fields(reply):
    """'ok a=1 b=x' -> {'a': '1', 'b': 'x'}"""
    assert reply.startswith("ok ")
    return dict(tok.split("=", 1) for tok in reply[3:].split() if "=" in tok)


# --- parse ---------------------------------------------------------------------------------

@pytest.mark.parametrize(
    "line, expected",
    [
        ("help", ("help", [])),
        ("status", ("status", [])),
        ("start", ("start", [])),
        ("start random", ("start", ["random"])),
        ("start clino3d", ("start", ["clino3d"])),
        ("start clino2d", ("start", ["clino2d"])),
        ("stop", ("stop", [])),
        ("estop", ("estop", [])),
        ("mode clino3d", ("mode", ["clino3d"])),
        ("set v_max 55.5", ("set", ["v_max", "55.5"])),
        ("pulse inner 1600", ("pulse", ["inner", "1600"])),
        ("pulse outer 1400", ("pulse", ["outer", "1400"])),
        ("pulse off", ("pulse", ["off"])),
    ],
)
def test_parse_every_command(line, expected):
    assert parse(line) == expected


@pytest.mark.parametrize(
    "line",
    ["  START   Clino2D ", "start clino2d\n", "start clino2d\r\n", "\tstart\tclino2d", "Start CLINO2D"],
)
def test_parse_ignores_case_and_whitespace(line):
    assert parse(line) == ("start", ["clino2d"])


@pytest.mark.parametrize("line", ["", "   ", "\r\n", "\t"])
def test_parse_empty(line):
    assert parse(line) == ("", [])


# --- execute: happy paths ------------------------------------------------------------------

def test_help_lists_every_command(c):
    reply = handle(c, "help")
    assert reply.startswith("ok ")
    for word in ("help", "status", "start", "stop", "estop", "mode", "set", "pulse"):
        assert word in reply


def test_status_format(c):
    f = fields(handle(c, "status"))
    assert set(f) == {"state", "mode", "inner_dps", "outer_dps", "inner_us", "outer_us", "t"}
    assert f["state"] == STOPPED and f["mode"] == "random"
    assert f["inner_us"] == f["outer_us"] == "off"
    assert float(f["inner_dps"]) == 0.0 and float(f["t"]) == 0.0


def test_status_reflects_running_controller(c):
    handle(c, "start clino2d")
    for _ in range(150):
        out = c.tick(DT)
    f = fields(handle(c, "status"))
    assert f["state"] == RUNNING and f["mode"] == "clino2d"
    assert float(f["outer_dps"]) == pytest.approx(c.status()["outer_dps"], abs=0.01)
    assert (int(f["inner_us"]), int(f["outer_us"])) == out
    assert f["t"] == "3.000"


def test_start_stop_estop(c):
    assert fields(handle(c, "start clino3d"))["state"] == RUNNING
    assert c.mode == "clino3d"
    for _ in range(50):
        c.tick(DT)
    assert fields(handle(c, "stop"))["state"] == STOPPING
    assert fields(handle(c, "estop"))["state"] == STOPPED
    assert c.state == STOPPED


def test_start_without_mode_uses_current(c):
    handle(c, "mode clino2d")
    handle(c, "start")
    assert (c.state, c.mode) == (RUNNING, "clino2d")


def test_mode(c):
    assert fields(handle(c, "mode clino3d"))["mode"] == "clino3d"
    assert c.mode == "clino3d"


def test_set_applies_value(c):
    reply = handle(c, "set v_max 55.5")
    assert reply.startswith("ok ")
    assert c.cfg["random"]["v_max_dps"] == 55.5
    handle(c, "set seed 12")
    assert c.cfg["seed"] == 12


def test_pulse_and_pulse_off(c):
    assert handle(c, "pulse outer 1600").startswith("ok ")
    assert c.state == CAL and c.tick(DT) == (None, 1600)
    assert fields(handle(c, "pulse off"))["state"] == STOPPED
    assert c.tick(DT) == (None, None)


def test_execute_takes_parsed_tuple(c):
    assert execute(c, "start", ["clino2d"]).startswith("ok ")
    assert c.mode == "clino2d"


def test_empty_line_gives_no_reply(c):
    before = snapshot(c)
    assert handle(c, "   ") == ""
    assert snapshot(c) == before


# --- execute: errors leave the controller unchanged ----------------------------------------

@pytest.mark.parametrize(
    "line",
    [
        "fly",                      # unknown command
        "start warp",               # unknown mode
        "start random extra",       # too many args
        "mode",                     # missing arg
        "mode hover",
        "set",                      # missing key and value
        "set v_max",                # missing value
        "set v_max abc",            # not a number
        "set v_max 500",            # beyond servo range
        "set v_min 0",
        "set bogus 1",              # unknown key
        "set v_max 50 60",          # too many args
        "set hold_max 0.0004",      # would hang tick()
        "set seed 1.5",
        "pulse",                    # missing args
        "pulse outer",              # missing pulse width
        "pulse middle 1600",        # unknown axis
        "pulse outer 5000",         # outside 700..2300
        "pulse outer fast",
        "pulse outer 1600.0",       # pulse widths are whole us
        "pulse off now",
        "help me",
        "status now",
        "stop now",
        "estop now",
    ],
)
def test_bad_input_is_err_and_changes_nothing(c, line):
    before = snapshot(c)
    reply = handle(c, line)
    assert reply.startswith("err ")
    assert snapshot(c) == before


def test_pulse_rejected_while_running(c):
    handle(c, "start clino3d")
    c.tick(DT)
    before = snapshot(c)
    assert handle(c, "pulse outer 1600").startswith("err ")
    assert snapshot(c) == before


def test_start_rejected_during_calibration(c):
    handle(c, "pulse outer 1600")
    assert handle(c, "start").startswith("err ")
    assert c.state == CAL


def test_err_messages_are_single_line(c):
    for line in ("fly", "set v_max abc", "pulse outer 5000", "start warp"):
        reply = handle(c, line)
        assert "\n" not in reply and "\r" not in reply


# --- robustness ----------------------------------------------------------------------------

def test_random_lines_never_raise(cfg):
    # On the Pico an exception from a command would end the main loop.
    ctrl = Controller(cfg)
    py = random.Random(7)
    words = [
        "help", "status", "start", "stop", "estop", "mode", "set", "pulse", "off",
        "inner", "outer", "random", "clino3d", "clino2d", "v_min", "v_max", "accel",
        "hold_min", "hold_max", "seed", "0", "-1", "1600", "1e309", "nan", "inf", "-inf",
        "55.5", "x", "", "=", "%s", "%d", "\x00", "ü", "1" * 400,
    ]
    for _ in range(5000):
        line = " ".join(py.choice(words) for _ in range(py.randint(0, 4)))
        reply = handle(ctrl, line)
        assert isinstance(reply, str)
        assert not reply.startswith("err internal"), (line, reply)
        ctrl.tick(py.choice([0.0, 0.02]))
        for us in ctrl.status()["inner_us"], ctrl.status()["outer_us"]:
            assert us is None or 700 <= us <= 2300
