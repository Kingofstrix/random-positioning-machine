"""Measure each drive's speed curve and write calibration.json on the Pico.

Run from the REPL: ``import calibrate``. For each drive the servo steps
through a list of pulse widths. Time one full revolution of the *driven axis*
(inner frame for the outer servo, platform for the inner servo; not the
servo horn) with a stopwatch and type the seconds:

- positive if the axis turns in the + direction (right-hand rule,
  docs/interfaces.md "Axes and frames"),
- negative if it turns the other way,
- 0 if it does not move (dead band).

Keep the whole machine assembled, the gears and slip ring are part of the load.
"""

import json
import time

import config
from machine import PWM, Pin

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
DRIVES = (("outer", config.OUTER_PIN), ("inner", config.INNER_PIN))


def _ask_seconds(prompt: str) -> float:
    while True:
        text = input(prompt).strip().replace(",", ".")
        try:
            return float(text)
        except ValueError:
            print("  please type a number, e.g. 4.8, -5.1 or 0")


def measure(name: str, pin: int) -> "list[list[float]]":
    pwm = PWM(Pin(pin))
    pwm.freq(50)
    points = []
    print("\n=== %s drive on GP%d ===" % (name, pin))
    try:
        for us in PULSES_US:
            pwm.duty_ns(us * 1000)
            time.sleep_ms(1500)  # let it reach speed
            seconds = _ask_seconds(
                "%4d us: seconds per revolution (sign = direction, 0 = stop): " % us
            )
            speed = 0.0 if seconds == 0 else 360.0 / seconds  # deg/s
            points.append([us, speed])
    finally:
        pwm.duty_ns(0)
    return points


def main() -> None:
    data = {}
    for name, pin in DRIVES:
        data[name] = {"points": measure(name, pin)}
        try:
            model = ServoModel.from_dict(data[name])
            print("%s: stop at %d us, max %.0f deg/s" % (name, model.stop_us, model.max_speed))
        except ValueError as e:
            # Typically a typo or a slipped stopwatch: speeds must rise (or fall)
            # steadily with the pulse width. Saved anyway so nothing is lost.
            print("WARNING %s: %s - fix calibration.json by hand or measure again" % (name, e))
    with open(config.CALIBRATION_FILE, "w") as f:
        json.dump(data, f)
    print("\nsaved", config.CALIBRATION_FILE, "- keep a copy: mpremote cp :calibration.json .")
    print(json.dumps(data))


main()
