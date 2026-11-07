"""Measure the servo curves of the Arduino Nano machine from a PC.

    uv run --group nano rpm-nano-calibrate --port /dev/cu.usbserial-110

The Nano has no REPL, so the PC drives it with the firmware's own ``pulse``
command over USB serial. For each pulse width you time one full revolution
of the *driven axis* (inner frame for the outer servo, platform for the inner
servo) with a stopwatch and type the seconds: positive in the + direction
(right-hand rule, docs/interfaces.md), negative the other way, 0 if it does
not move. The curve is checked with the same rules as ``ServoModel`` and
written to ``firmware/nano/rpm_nano/calibration.h``; flash again to use it.
An invalid curve is never written there.
"""

import argparse
import math
import sys
import time
from collections.abc import Callable
from pathlib import Path

from rpm.core.servo import ServoModel

PULSES_US = (
    1000,
    1150,
    1300,
    1400,
    1440,
    1460,
    1480,
    1500,
    1520,
    1540,
    1560,
    1600,
    1700,
    1850,
    2000,
)
MAX_POINTS = 16  # MAX_SERVO_POINTS in rpm_core.h
HEADER = Path(__file__).resolve().parents[2] / "firmware" / "nano" / "rpm_nano" / "calibration.h"


def parse_seconds(text: str) -> float | None:
    """Stopwatch input -> seconds per revolution, or None if it is not a number."""
    try:
        seconds = float(text.strip().replace(",", "."))
    except ValueError:
        return None
    return seconds if math.isfinite(seconds) else None  # "nan"/"inf" parse, but are typos


def speed_from_seconds(seconds: float) -> float:
    """Seconds per revolution (signed, 0 = stop) -> axis deg/s."""
    return 0.0 if seconds == 0.0 else 360.0 / seconds


def calibration_header(
    outer: list[tuple[int, float]], inner: list[tuple[int, float]], note: str
) -> str:
    """C header with both curves; raises ValueError if one is invalid for the firmware."""
    lines = [
        "// Measured servo curves, written by rpm-nano-calibrate. Do not edit by hand:",
        "// measure again instead. Included by config.h instead of the PLAN 2.5 values.",
        "#pragma once",
        "",
    ]
    for name, points in (("OUTER", outer), ("INNER", inner)):
        if len(points) > MAX_POINTS:
            raise ValueError(f"{name.lower()}: at most {MAX_POINTS} points")
        model = ServoModel([(float(p), v) for p, v in points])  # same checks as ServoModel::set
        pts = model.points
        us = ", ".join(str(int(p)) for p, _ in pts)
        dps = ", ".join(f"{v:.2f}f" for _, v in pts)
        lines += [
            f"const int CAL_{name}_N = {len(pts)};",
            f"const int16_t CAL_{name}_US[] = {{{us}}};",
            f"const float CAL_{name}_DPS[] = {{{dps}}};",
        ]
    note = note.replace("\\", "/").replace('"', "'")
    lines += [f'#define RPM_CALIBRATION "{note}"', ""]
    return "\n".join(lines)


def measure(
    send: Callable[[str], str], ask: Callable[[str], str], axis: str
) -> list[tuple[int, float]]:
    """Step one axis through PULSES_US; ``send`` talks to the Nano, ``ask`` to the user."""
    points = []
    try:
        for us in PULSES_US:
            reply = send(f"pulse {axis} {us}")
            if not reply.startswith("ok"):
                raise RuntimeError(f"Nano refused 'pulse {axis} {us}': {reply}")
            while True:
                seconds = parse_seconds(ask(f"{axis} {us:4d} us: seconds per revolution: "))
                if seconds is not None:
                    break
                print("  please type a number, e.g. 4.8, -5.1 or 0")
            points.append((us, speed_from_seconds(seconds)))
    finally:
        send("pulse off")
    return points


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="rpm-nano-calibrate", description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, e.g. /dev/cu.usbserial-110")
    parser.add_argument("--out", default=str(HEADER))
    args = parser.parse_args(argv)
    try:
        import serial  # type: ignore[import-untyped]
    except ImportError:
        sys.exit("needs pyserial: uv run --group nano rpm-nano-calibrate ...")

    with serial.Serial(args.port, 115200, timeout=2) as link:
        time.sleep(2.0)  # opening the port resets the Nano; wait for the bootloader
        link.reset_input_buffer()

        def send(line: str) -> str:
            link.write((line + "\n").encode())
            while True:
                reply = link.readline().decode(errors="replace").strip()
                if not reply:
                    raise RuntimeError(f"no reply to {line!r}")
                if reply.startswith(("ok", "err")):
                    return reply

        print(send("estop"))
        outer = measure(send, input, "outer")
        inner = measure(send, input, "inner")
    try:
        text = calibration_header(outer, inner, time.strftime("measured %Y-%m-%d %H:%M"))
    except ValueError as e:
        rejected = Path(args.out).with_suffix(".rejected.txt")
        rejected.write_text(f"{e}\nouter={outer}\ninner={inner}\n")
        sys.exit(f"NOT USED: {e}. Measurements kept in {rejected}; measure again.")
    Path(args.out).write_text(text)
    print(f"wrote {args.out}; flash the sketch again to use it")


if __name__ == "__main__":
    main()
