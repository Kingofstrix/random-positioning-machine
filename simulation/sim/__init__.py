"""MuJoCo simulation of the RPM, driven by the same `rpmcore` logic that runs on the Pico."""

import sys
from pathlib import Path

# Lets `python -m sim.…` find rpmcore without an editable install, so the sim never runs a
# stale copy of the firmware logic.
_FIRMWARE_DIR = str(Path(__file__).resolve().parent.parent / "firmware")
if _FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, _FIRMWARE_DIR)
