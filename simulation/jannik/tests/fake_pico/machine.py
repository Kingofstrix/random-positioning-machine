"""Stand-in for MicroPython's ``machine`` module so firmware/main.py runs on a PC.

Used by tests/micropython_smoke.py; records every PWM write for assertions.
"""

writes = {}  # pin -> list of duty_ns values
fail_on = {}  # pin -> duty_ns value whose write raises OSError (fault injection)


class Pin:
    def __init__(self, pin):
        self.id = pin


class PWM:
    def __init__(self, pin):
        self.pin = pin.id
        writes[self.pin] = []

    def freq(self, hz):
        assert hz == 50

    def duty_ns(self, ns):
        if fail_on.get(self.pin) == ns:
            raise OSError("injected PWM fault")
        writes[self.pin].append(ns)
