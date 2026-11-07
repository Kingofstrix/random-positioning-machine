# firmware

Everything in this folder goes onto the Raspberry Pi Pico.

- `rpmcore/`: pure control logic (PRNG, servo mapping, motion profiles, controller, command
  parser). No hardware imports, only `math` and `rpmcore.*`, so it runs under MicroPython and
  CPython. The simulation imports it directly.
- `main.py` (milestone M2): connects `rpmcore` to PWM, USB serial and the onboard LED.

The MicroPython rules are listed in `CLAUDE.md` and checked by
`tests/firmware/test_micropython_compat.py`.
