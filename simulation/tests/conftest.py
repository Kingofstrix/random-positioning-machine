"""Shared test setup. The `slow` marker is registered in pyproject.toml."""

import copy
import sys
from pathlib import Path

import pytest

# Tests must also run without `pip install -e`, e.g. in a fresh checkout or CI.
_SIM_ROOT = Path(__file__).resolve().parent.parent
for _p in (_SIM_ROOT / "firmware", _SIM_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


@pytest.fixture
def cfg():
    """A private copy of CONFIG, so a test that mutates it cannot leak into others."""
    from rpmcore.config import CONFIG

    return copy.deepcopy(CONFIG)
