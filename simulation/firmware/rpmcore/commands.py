"""Line-based command protocol (docs/PLAN.md 5.6).

The Pico (USB serial) and the simulation viewer (terminal) both feed text lines through
`handle`, so the protocol is defined once. Every reply is one line starting with `ok` or `err`;
`execute` never raises, because an exception here would end the Pico's main loop.
"""

HELP = (
    "ok commands: help | status | start [random|clino3d|clino2d] | stop | estop"
    " | mode <random|clino3d|clino2d> | set <v_min|v_max|accel|hold_min|hold_max|seed> <value>"
    " | pulse <inner|outer> <us> | pulse off"
)

# command -> (min args, max args, usage)
_SPEC = {
    "help": (0, 0, "help"),
    "status": (0, 0, "status"),
    "start": (0, 1, "start [random|clino3d|clino2d]"),
    "stop": (0, 0, "stop"),
    "estop": (0, 0, "estop"),
    "mode": (1, 1, "mode <random|clino3d|clino2d>"),
    "set": (2, 2, "set <key> <value>"),
    "pulse": (1, 2, "pulse <inner|outer> <us> | pulse off"),
}


def parse(line):
    """Split a line into (command, args), lower case, any whitespace. Empty -> ('', [])."""
    tokens = line.strip().lower().split()
    if not tokens:
        return ("", [])
    return (tokens[0], tokens[1:])


def handle(controller, line):
    cmd, args = parse(line)
    return execute(controller, cmd, args)


def execute(controller, cmd, args):
    if cmd == "":
        return ""
    if cmd not in _SPEC:
        return "err unknown command: %s (try help)" % cmd
    lo, hi, usage = _SPEC[cmd]
    if not lo <= len(args) <= hi:
        return "err usage: %s" % usage
    if cmd == "pulse" and (args[0] == "off") != (len(args) == 1):
        return "err usage: %s" % usage
    try:
        return _run(controller, cmd, args)
    except ValueError as e:
        return "err %s" % _one_line(e)
    except Exception as e:
        # A bug must not stop the servo loop; the prefix makes it easy to spot in logs/tests.
        return "err internal %s" % _one_line(e)


def _run(c, cmd, args):
    if cmd == "help":
        return HELP
    if cmd == "status":
        return _status(c)
    if cmd == "start":
        c.start(args[0] if args else None)
        return "ok state=%s mode=%s" % (c.state, c.mode)
    if cmd == "stop":
        c.stop()
        return "ok state=%s" % c.state
    if cmd == "estop":
        c.estop()
        return "ok state=%s" % c.state
    if cmd == "mode":
        c.set_mode(args[0])
        return "ok mode=%s" % c.mode
    if cmd == "set":
        c.set_param(args[0], args[1])
        return "ok %s=%s" % (args[0], args[1])
    # pulse
    if args[0] == "off":
        c.cal_off()
        return "ok state=%s" % c.state
    c.cal_pulse(args[0], args[1])
    return "ok state=%s %s_us=%s" % (c.state, args[0], args[1])


def _status(c):
    s = c.status()
    t_ms = s["t_ms"]
    # Seconds printed from the int-ms counter, not via float, to stay exact on the Pico.
    return "ok state=%s mode=%s inner_dps=%.2f outer_dps=%.2f inner_us=%s outer_us=%s t=%d.%03d" % (
        s["state"],
        s["mode"],
        s["inner_dps"],
        s["outer_dps"],
        _us(s["inner_us"]),
        _us(s["outer_us"]),
        t_ms // 1000,
        t_ms % 1000,
    )


def _us(us):
    return "off" if us is None else "%d" % us


def _one_line(e):
    return str(e).replace("\r", " ").replace("\n", " ")
