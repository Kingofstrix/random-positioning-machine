"""Pico firmware: runs the shared controller on the two FS90R servos.

Copy ``rpm/core``, ``config.py``, this file and (optionally) ``calibration.json``
to the Pico; see simulation/README.md. The motion logic lives in rpm.core and
is the same code the simulation tests, this file only does timing and PWM.
"""

import json
import time

import config
from machine import PWM, Pin

from rpm.core.controller import Controller
from rpm.core.servo import ServoModel
from rpm.core.strategies import make_strategy

PWM_FREQ_HZ = 50


class ServoOutput:
    """One FS90R on a PWM pin, driven by pulse width in microseconds."""

    def __init__(self, pin: int) -> None:
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(PWM_FREQ_HZ)

    def write_us(self, pulse_us: int) -> None:
        self.pwm.duty_ns(pulse_us * 1000)

    def off(self) -> None:
        self.pwm.duty_ns(0)  # no pulses: the servo goes limp


def load_models(path: str) -> "tuple[ServoModel, ServoModel]":
    """Servo curves from ``path``, or the datasheet curves if it is missing or broken.

    A bad file (typo, non-monotone curve) must never keep the Pico from booting.
    """
    try:
        with open(path) as f:
            data = json.load(f)
        models = ServoModel.from_dict(data["outer"]), ServoModel.from_dict(data["inner"])
        print("calibration:", path)
        return models
    except OSError:
        print("calibration: none, using FS90R datasheet values")
    except (ValueError, KeyError, TypeError, IndexError) as e:
        print("calibration: %s is invalid (%s), using FS90R datasheet values" % (path, e))
    return ServoModel.nominal(), ServoModel.nominal()


def run(max_ticks: int = 0) -> Controller:
    """Control loop. ``max_ticks`` > 0 stops after that many ticks (for tests)."""
    outer_model, inner_model = load_models(config.CALIBRATION_FILE)
    max_speed = min(config.MAX_SPEED, outer_model.max_speed, inner_model.max_speed)
    strategy = make_strategy(config.STRATEGY, max_speed, config.SEED)
    controller = Controller(strategy, outer_model, inner_model)
    outer = ServoOutput(config.OUTER_PIN)
    inner = ServoOutput(config.INNER_PIN)
    print("strategy:", config.STRATEGY, "max_speed:", max_speed, "deg/s, seed:", config.SEED)

    period_ms = config.CONTROL_PERIOD_MS
    status_ticks = int(config.STATUS_EVERY_S * 1000 / period_ms)
    ticks = 0
    next_ms = time.ticks_ms()
    try:
        while max_ticks <= 0 or ticks < max_ticks:
            pulse_o, pulse_i = controller.tick(period_ms / 1000.0)
            outer.write_us(pulse_o)
            inner.write_us(pulse_i)
            ticks += 1
            if status_ticks and ticks % status_ticks == 0:
                residual = controller.estimate.stats.residual_gravity()
                print("t=%ds residual(est)=%.4f g" % (ticks * period_ms // 1000, residual))
            # Fixed-rate loop: wait for the next slot instead of sleeping a
            # constant time, so computation time does not stretch the period.
            next_ms = time.ticks_add(next_ms, period_ms)
            wait = time.ticks_diff(next_ms, time.ticks_ms())
            if wait > 0:
                time.sleep_ms(wait)
    finally:
        # Always stop the servos, also on Ctrl-C or an exception.
        outer.write_us(outer_model.stop_us)
        inner.write_us(inner_model.stop_us)
        time.sleep_ms(200)
        outer.off()
        inner.off()
    return controller


if __name__ == "__main__":
    run()
