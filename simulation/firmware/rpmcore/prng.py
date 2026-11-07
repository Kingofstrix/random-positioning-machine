"""Xorshift32 PRNG (Marsaglia 2003, shifts 13/17/5).

Own generator instead of `random`, so the Pico, CPython and the tests draw the same sequence
for the same seed.
"""

_MASK = 0xFFFFFFFF
# The state must never be 0 (xorshift would stay 0 forever). Any fixed non-zero value works.
_ZERO_SEED_STATE = 0x9E3779B9
_INV_2_24 = 1.0 / 16777216.0


class XorShift32:
    def __init__(self, seed):
        self.seed(seed)

    def seed(self, seed):
        state = int(seed) & _MASK
        if state == 0:
            state = _ZERO_SEED_STATE
        self._state = state

    def next_u32(self):
        x = self._state
        x ^= (x << 13) & _MASK
        x ^= x >> 17
        x ^= (x << 5) & _MASK
        self._state = x
        return x

    def random(self):
        """Float in [0, 1).

        Uses the top 24 bits only: they fit a float32 mantissa exactly, so the Pico gets the
        same value as CPython and the result can never round up to 1.0.
        """
        return (self.next_u32() >> 8) * _INV_2_24

    def uniform(self, a, b):
        return a + (b - a) * self.random()

    def choice_sign(self):
        # Top bit: the low bits of xorshift are the weakest.
        return 1 if self.next_u32() >> 31 else -1
