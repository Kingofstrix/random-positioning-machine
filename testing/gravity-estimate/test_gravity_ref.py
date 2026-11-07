"""Checks for gravity_ref.py, vectors.csv and (if a C++ compiler exists) gravity_estimate.h.

Run from the repo root:  pytest testing/gravity-estimate
"""

from __future__ import annotations

import csv
import math
import random
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from gravity_ref import (
    GravityEstimate,
    bin_index,
    gravity_in_sample,
    round_half_away,
    run_scenario,
    write_vectors,
)

HERE = Path(__file__).parent
CLINO = (30.0, 18.54)


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def test_formula_matches_rotation_matrix():
    rng = random.Random(1)
    for _ in range(1000):
        ao, ai = rng.uniform(0, 360), rng.uniform(0, 360)
        r = rx(math.radians(ao)) @ ry(math.radians(ai))
        expected = r.T @ np.array([0.0, -1.0, 0.0])
        assert np.allclose(gravity_in_sample(ao, ai), expected, atol=1e-12)


def test_bins_equal_area():
    v = np.random.default_rng(0).normal(size=(400_000, 3))
    v /= np.linalg.norm(v, axis=1)[:, None]
    q = [tuple(round_half_away(c * 1000) for c in row) for row in v]
    counts = np.bincount([bin_index(*p) for p in q], minlength=200)
    assert counts.min() > 0.9 * counts.mean() and counts.max() < 1.1 * counts.mean()


def test_all_200_bins_reachable():
    seen = set()
    for ao in range(0, 360, 2):
        for ai in range(0, 360, 2):
            g = gravity_in_sample(ao, ai)
            seen.add(bin_index(*(round_half_away(c * 1000) for c in g)))
    assert len(seen) == 200


def test_round_half_away_like_c():
    assert [round_half_away(x) for x in (0.5, 1.5, -0.5, -1.5, 0.4, -0.4)] == [1, 2, -1, -2, 0, 0]


def test_before_first_sample():
    g = GravityEstimate()
    assert g.gres() == 1.0 and g.cov() == 0.0
    g.tick(30, 18, 20, True)  # 20 ms < SAMPLE_MS
    assert g.n == 0 and g.gres() == 1.0


def test_still_at_zero():
    *_, (t, gres, cov) = run_scenario(60, lambda t: (0.0, 0.0))
    assert t == 60 and gres == pytest.approx(1.0) and cov == pytest.approx(0.005)


def test_inner_only_does_not_average_gravity():
    # At a_o = 0 the inner axis is vertical: turning it never tilts the sample.
    *_, (_, gres, _) = run_scenario(60, lambda t: (0.0, 30.0))
    assert gres > 0.95


def test_clino3d_reaches_ready_within_10_min():
    rows = list(run_scenario(600, lambda t: CLINO, step_s=1))
    ready = [t for t, gres, cov in rows if gres < 0.05 and cov >= 0.9]
    assert ready and ready[0] <= 600
    assert rows[-1][1] < 0.01 and rows[-1][2] == pytest.approx(1.0)


def test_no_samples_when_not_running():
    g = GravityEstimate()
    for _ in range(100):
        g.tick(30, 18, 20, False)
    assert g.n == 0 and g.a_o == pytest.approx(60.0) and g.a_i == pytest.approx(36.0)


def test_start_resets_stats_not_angles():
    g = GravityEstimate()
    for _ in range(3000):
        g.tick(*CLINO, 20, True)
    ao, ai = g.a_o, g.a_i
    g.start()
    assert (g.n, g.sx, g.sy, g.sz, g.gres(), g.cov()) == (0, 0, 0, 0, 1.0, 0.0)
    assert (g.a_o, g.a_i) == (ao, ai)


def test_negative_speeds_wrap():
    g = GravityEstimate()
    g.tick(-30, -10, 1000, False)
    assert g.a_o == pytest.approx(330.0) and g.a_i == pytest.approx(350.0)


def test_vectors_csv_is_up_to_date(tmp_path):
    fresh = tmp_path / "vectors.csv"
    write_vectors(fresh)
    assert fresh.read_text() == (HERE / "vectors.csv").read_text()


CPP_MAIN = r"""
#include <cstdio>
#include "gravity_estimate.h"
static void run(const char* name, int dur_s, float o, float i, int reset_s) {
  GravityEstimate g; g.start();
  std::printf("%s,0,%.3f,%.3f\n", name, g.gres(), g.cov());
  for (long t = 20; t <= dur_s * 1000L; t += 20) {
    g.tick(o, i, 20, true);
    if (reset_s >= 0 && t == reset_s * 1000L) g.start();
    if (t % 10000 == 0) std::printf("%s,%ld,%.3f,%.3f\n", name, t / 1000, g.gres(), g.cov());
  }
}
int main() {
  std::printf("scenario,t_s,gres,cov\n");
  run("still", 60, 0, 0, -1);
  run("inner_only", 60, 0, 30, -1);
  run("clino3d", 600, 30, 18.54f, -1);
  run("clino3d_reset", 300, 30, 18.54f, 150);
}
"""


def _read(text):
    return list(csv.DictReader(text.splitlines()))


@pytest.mark.skipif(shutil.which("c++") is None, reason="no host C++ compiler")
def test_header_matches_vectors(tmp_path):
    src = tmp_path / "main.cpp"
    src.write_text(CPP_MAIN)
    exe = tmp_path / "grav"
    subprocess.run(["c++", "-std=c++11", "-I", str(HERE), str(src), "-o", str(exe)], check=True)
    got = _read(subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout)
    want = _read((HERE / "vectors.csv").read_text())
    assert [(r["scenario"], r["t_s"]) for r in got] == [(r["scenario"], r["t_s"]) for r in want]
    # float32 angles in C vs float64 here: allow a little drift, no more.
    for g, w in zip(got, want):
        assert abs(float(g["gres"]) - float(w["gres"])) <= 0.005, (g, w)
        assert abs(float(g["cov"]) - float(w["cov"])) <= 0.02, (g, w)  # 4 bins
