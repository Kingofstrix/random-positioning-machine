"""Guards the project layout that every later milestone builds on."""

import importlib

import pytest

STL_FILES = [
    "01_outer_frame.stl",
    "02_inner_frame.stl",
    "03_experiment_platform.stl",
    "04_spur_gear_large_outer.stl",
    "05_spur_gear_large_inner.stl",
    "06_spur_gear_small.stl",
    "07_end_cap.stl",
    "optional_stand/Stand.stl",
]


@pytest.mark.parametrize("name", ["rpmcore", "sim"])
def test_packages_importable(name):
    importlib.import_module(name)


def test_rpmcore_is_loaded_from_firmware_folder():
    # The sim must run the exact code that gets flashed, not an installed copy elsewhere.
    import rpmcore

    assert rpmcore.__file__ is not None
    assert "firmware" in rpmcore.__file__.replace("\\", "/").split("/")


@pytest.mark.parametrize("rel", STL_FILES)
def test_stl_dir_points_to_printables(rel):
    from sim import geometry

    assert geometry.STL_DIR.name == "rpm_printables"
    assert (geometry.STL_DIR / rel).is_file()


def test_slow_marker_registered(pytestconfig):
    markers = pytestconfig.getini("markers")
    assert any(m.startswith("slow") for m in markers)
