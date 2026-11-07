import importlib.util
import shutil
import sys

import matplotlib.image as mpimg
import mujoco
import numpy as np
import pytest

from sim.mjcf import build_mjcf
from sim.metrics import extra_accel, extra_accel_series, sphere_bin_edges, sphere_bins, sphere_coverage
from sim.params import MachineParams, ReportParams
from sim import report
from sim.report import frame_times, main, replay_qpos
from sim.run import main as run_main

PLOTS = ("residual_g", "sphere", "speed", "extra_accel")
G = 9.81


@pytest.fixture(scope="module")
def results(tmp_path_factory):
    out = tmp_path_factory.mktemp("results")
    run_main(["--mode", "random", "--minutes", "0.05", "--seed", "3", "--out", str(out)])
    return out


def test_report_writes_four_decodable_pngs(results):
    main([str(results)])
    for name in PLOTS:
        png = results / ("random_s3_%s.png" % name)
        assert png.exists() and png.stat().st_size > 0, png
        img = mpimg.imread(png)
        assert img.ndim == 3 and img.shape[0] > 0 and img.shape[1] > 0


def test_report_covers_every_run_and_ignores_other_files(results, tmp_path, monkeypatch):
    shutil.copy(results / "random_s3.npz", tmp_path / "random_s3.npz")
    shutil.copy(results / "random_s3.npz", tmp_path / "clino3d_s7.npz")
    (tmp_path / "notes.npz").write_bytes(b"")  # not <mode>_s<seed>.npz -> must be skipped
    seen = []
    # Plotting itself is covered above; recording the calls keeps this test fast.
    monkeypatch.setattr(report, "write_plots", lambda path, rp: seen.append(path.name) or [])
    main([str(tmp_path)])
    assert sorted(seen) == ["clino3d_s7.npz", "random_s3.npz"]


def test_report_without_runs_fails_clearly(tmp_path):
    with pytest.raises(SystemExit, match="npz"):
        main([str(tmp_path)])


def test_sphere_bins_counts_and_placement():
    dirs = np.array([[0, 0, 1.0], [0, 0, -2.0], [3.0, 0, 0], [0, 0.5, 0]])
    c = sphere_bins(dirs, n_bands=4, n_sectors=8)
    assert c.shape == (4, 8) and c.sum() == 4
    assert c[3].sum() == 1 and c[0].sum() == 1      # +z top band, -z bottom band
    assert c[2, 4] == 1                              # +x: azimuth 0 -> sector n/2, band above z=0
    assert c[2, 6] == 1                              # +y: azimuth pi/2


def test_sphere_bins_agree_with_coverage():
    rng = np.random.default_rng(1)
    v = rng.normal(size=(5000, 3)) * [1.0, 1.0, 0.3]
    c = sphere_bins(v, n_bands=10, n_sectors=20)
    frac, nonuni = sphere_coverage(v, n_bands=10, n_sectors=20)
    assert frac == (c > 0).mean()
    assert nonuni == pytest.approx(c.std() / c.mean())


def test_sphere_bin_edges_are_equal_area():
    lon, lat = sphere_bin_edges(n_bands=6, n_sectors=12)
    assert lon[0] == pytest.approx(-np.pi) and lon[-1] == pytest.approx(np.pi) and len(lon) == 13
    assert lat[0] == pytest.approx(-np.pi / 2) and lat[-1] == pytest.approx(np.pi / 2) and len(lat) == 7
    assert np.allclose(np.diff(np.sin(lat)), 2 / 6)  # equal steps in z = equal area


def test_extra_accel_series_matches_summary():
    g = np.tile([0, 0, -G], (10, 1))
    acc = -g.copy()
    acc[3] += [G, 0, 0]
    e = extra_accel_series(acc, g)
    assert e.shape == (10,) and e[3] == pytest.approx(1.0) and e[0] == pytest.approx(0.0)
    assert extra_accel(acc, g) == (pytest.approx(e.max()), pytest.approx(np.sqrt(np.mean(e**2))))


def test_frame_times_time_lapse():
    rp = ReportParams()
    t = frame_times(rp, t_end=1e9)
    assert len(t) == round(rp.video_seconds * rp.video_fps)
    assert t[1] - t[0] == pytest.approx(rp.video_speedup / rp.video_fps)
    assert frame_times(rp, t_end=2.0)[-1] <= 2.0  # a short run ends the video early


def test_replay_qpos_sets_frames_and_pinions():
    p = MachineParams()
    m = mujoco.MjModel.from_xml_string(build_mjcf(p))
    d = mujoco.MjData(m)
    replay_qpos(m, d, q_outer=0.3, q_inner=-1.2, gear_ratio=p.gear_ratio)
    q = {n: d.qpos[m.jnt_qposadr[m.joint(n).id]] for n in ("outer", "inner", "outer_pinion", "inner_pinion")}
    assert q["outer"] == pytest.approx(0.3) and q["inner"] == pytest.approx(-1.2)
    assert q["outer_pinion"] == pytest.approx(-p.gear_ratio * 0.3)
    assert q["inner_pinion"] == pytest.approx(-p.gear_ratio * -1.2)


def test_video_without_imageio_names_the_extra(results, monkeypatch):
    monkeypatch.setitem(sys.modules, "imageio", None)  # makes `import imageio` fail
    with pytest.raises(SystemExit, match=r"imageio.*\[video\]"):
        main([str(results), "--video"])


@pytest.mark.skipif(importlib.util.find_spec("imageio") is None, reason="imageio ([video] extra) not installed")
def test_video_writes_mp4(results):
    from sim.report import render_video

    out = results / "short.mp4"
    render_video(results / "random_s3.npz", out, ReportParams(video_seconds=0.5))
    assert out.stat().st_size > 0
