"""Seeded pseudo-random numbers that are identical on the Pico and on a PC.

MicroPython's ``random`` differs between ports and CPython's Mersenne Twister
is not available on the Pico. A tiny xorshift32 gives the same sequence
everywhere, so a seed reproduces a run in simulation and on hardware.
"""

_MASK32 = 0xFFFFFFFF
_DEFAULT_STATE = 0x9E3779B9


class Rng:
    """xorshift32 generator (Marsaglia 2003)."""

    def __init__(self, seed: int = 1) -> None:
        self.seed(seed)

    def seed(self, seed: int) -> None:
        state = seed & _MASK32
        self._state = state if state else _DEFAULT_STATE

    def next_u32(self) -> int:
        x = self._state
        x ^= (x << 13) & _MASK32
        x ^= x >> 17
        x ^= (x << 5) & _MASK32
        self._state = x
        return x

    def random(self) -> float:
        """Uniform float in [0, 1).

        Uses 24 bits so the result stays strictly below 1.0 even with the
        single-precision floats of the Pico port.
        """
        return (self.next_u32() >> 8) / 16777216.0

    def uniform(self, low: float, high: float) -> float:
        return low + (high - low) * self.random()

    def randint(self, low: int, high: int) -> int:
        """Uniform integer in [low, high], exact on every port (no floats)."""
        return low + self.next_u32() % (high - low + 1)

    def sign(self) -> float:
        return 1.0 if self.next_u32() & 1 else -1.0
