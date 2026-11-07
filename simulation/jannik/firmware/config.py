"""Machine configuration for the Pico firmware. Edit here, not in main.py.

Pins and wiring are a team interface (docs/interfaces.md, "Wiring"): the
values follow the Core Electronics assembly guide, p. 8. The original
main.py has them swapped, so check them on our machine.
"""

OUTER_PIN = 15  # outer servo, on the fixed outer frame: turns the inner frame
INNER_PIN = 16  # inner servo, on the inner frame (through the slip ring): turns the platform

STRATEGY = "clinostat"  # see rpm/core/strategies.py: clinostat | random_walk | random_setpoints
MAX_SPEED = 120.0  # axis deg/s (20 rpm); capped at what calibration.json says the drives reach
SEED = 1  # same seed + same strategy = same motion as in the simulation

CONTROL_PERIOD_MS = 100
STATUS_EVERY_S = 10  # print the estimated residual gravity this often (0 = never)

CALIBRATION_FILE = "calibration.json"  # written by calibrate.py; datasheet values if missing
