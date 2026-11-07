"""The Arduino Nano firmware (firmware/nano/rpm_nano) against the Python reference.

The Nano cannot run Python, so its motion logic is a C++ port of rpm/core.
These tests compile the port on the PC (with 32-bit floats, like the AVR) and
check that it moves the machine exactly like the code the simulation tests:
same timer events, same pulses up to float rounding, same estimate. They also
run the state machine and serial protocol tests, and, if arduino-cli is
installed, check that the sketch fits the ATmega328P.
"""

import re
import shutil
import subprocess
from pathlib import Path

import pytest

from rpm.core.controller import make_controller
from rpm.core.servo import ServoModel
from rpm.core.strategies import STRATEGIES
from tests.test_strategies import REFERENCE_RUNS

ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / "firmware" / "nano" / "rpm_nano"
HOST_TESTS = ROOT / "tests" / "nano"
CXX = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
ARDUINO_CLI = shutil.which("arduino-cli")

# The pulse sum over 72000 pulses may differ by a few microseconds where float32
# rounds a pulse the other way; the same bound as the float32 MicroPython check.
PULSE_SUM_TOLERANCE = 72

# ATmega328P: 30720 bytes flash with the bootloader, 2048 bytes RAM. Keep at
# least this much RAM free for the stack (the reply buffer alone is 160 bytes).
MIN_FREE_RAM = 700

needs_cxx = pytest.mark.skipif(CXX is None, reason="no C++ compiler")


def build_and_run(source: str, tmp_path: Path) -> str:
    assert CXX is not None
    exe = tmp_path / Path(source).stem
    flags = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off"]
    subprocess.run(
        [CXX, *flags, f"-I{SKETCH}", str(HOST_TESTS / source), "-o", str(exe)],
        check=True,
        capture_output=True,
        text=True,
    )
    done = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    return done.stdout


@pytest.fixture(scope="module")
def cpp_runs(tmp_path_factory: pytest.TempPathFactory) -> dict[str, tuple[int, int, float]]:
    if CXX is None:
        pytest.skip("no C++ compiler")
    out = build_and_run("reference_runs.cpp", tmp_path_factory.mktemp("nano"))
    runs = {}
    for line in out.splitlines():
        name, events, pulse_sum, residual = line.split()
        runs[name] = (int(events), int(pulse_sum), float(residual))
    return runs


@needs_cxx
@pytest.mark.parametrize("name", sorted(STRATEGIES))
def test_cpp_core_switches_at_the_same_ticks(cpp_runs: dict, name: str) -> None:
    assert cpp_runs[name][0] == REFERENCE_RUNS[name][0]


@needs_cxx
@pytest.mark.parametrize("name", sorted(STRATEGIES))
def test_cpp_core_sends_the_same_pulses(cpp_runs: dict, name: str) -> None:
    assert abs(cpp_runs[name][1] - REFERENCE_RUNS[name][1]) <= PULSE_SUM_TOLERANCE


@needs_cxx
@pytest.mark.parametrize("name", sorted(STRATEGIES))
def test_cpp_estimate_matches_python(cpp_runs: dict, name: str) -> None:
    # The dead-reckoned residual after one hour, which the Nano reports in status.
    model = ServoModel.nominal()
    controller = make_controller(name, 120.0, 1, model, model)
    for _ in range(36000):
        controller.tick(0.1)
    expected = controller.estimate.stats.residual_gravity()
    assert cpp_runs[name][2] == pytest.approx(expected, abs=2e-3)


@needs_cxx
def test_state_machine_and_serial_protocol(tmp_path: Path) -> None:
    assert "machine tests passed" in build_and_run("machine_test.cpp", tmp_path)


@pytest.mark.skipif(ARDUINO_CLI is None, reason="arduino-cli not installed")
@pytest.mark.parametrize("cpu", ["atmega328old", "atmega328"])  # clones mostly have the old one
def test_sketch_fits_the_nano(cpu: str) -> None:
    assert ARDUINO_CLI is not None
    done = subprocess.run(
        [ARDUINO_CLI, "compile", "--fqbn", f"arduino:avr:nano:cpu={cpu}", str(SKETCH)],
        check=True,
        capture_output=True,
        text=True,
        env={"LANG": "C", "LC_ALL": "C", **_path_env()},
    )
    numbers = [int(n) for n in re.findall(r"(\d+) bytes", done.stdout)]
    flash, flash_max, ram, ram_free, ram_max = numbers[:5]
    assert (flash_max, ram_max) == (30720, 2048)
    assert flash <= flash_max
    assert ram_free >= MIN_FREE_RAM, f"only {ram_free} bytes RAM left for the stack"
    assert ram + ram_free == ram_max


def _path_env() -> dict[str, str]:
    import os

    keep = ("PATH", "HOME", "TMPDIR")
    return {k: os.environ[k] for k in keep if k in os.environ}


@pytest.mark.skipif(ARDUINO_CLI is None, reason="arduino-cli not installed")
def test_sketch_builds_with_a_generated_calibration(tmp_path: Path) -> None:
    from rpm.nano.calibrate import calibration_header

    assert ARDUINO_CLI is not None
    sketch = tmp_path / "rpm_nano"
    shutil.copytree(SKETCH, sketch)
    curve = [(1150, -120.0), (1455, 0.0), (1545, 0.0), (1850, 118.0)]
    (sketch / "calibration.h").write_text(calibration_header(curve, curve, "test"))
    done = subprocess.run(
        [ARDUINO_CLI, "compile", "--fqbn", "arduino:avr:nano:cpu=atmega328old", str(sketch)],
        capture_output=True,
        text=True,
        env={"LANG": "C", "LC_ALL": "C", **_path_env()},
    )
    assert done.returncode == 0, done.stderr
