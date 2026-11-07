"""Evaluation of a run (docs/PLAN.md 6.6). Inputs are (N, 3) vectors in the sample frame."""

import numpy as np

G0 = 9.81


def residual_g(g_series: np.ndarray) -> float:
    """|time average of gravity in the sample frame| in units of g."""
    return float(np.linalg.norm(np.mean(g_series, axis=0)) / G0)


def residual_g_curve(g_series: np.ndarray) -> np.ndarray:
    """Cumulative residual g after each sample (for the log plot)."""
    cum = np.cumsum(g_series, axis=0) / np.arange(1, len(g_series) + 1)[:, None]
    return np.linalg.norm(cum, axis=1) / G0


def windowed_residual(g_series: np.ndarray, window_s: float, sample_hz: float) -> tuple[float, float]:
    """Residual g per non-overlapping window -> (median, 95th percentile)."""
    n = int(round(window_s * sample_hz))
    k = len(g_series) // n
    if k == 0:
        return float("nan"), float("nan")
    w = g_series[: k * n].reshape(k, n, 3).mean(axis=1)
    r = np.linalg.norm(w, axis=1) / G0
    return float(np.median(r)), float(np.percentile(r, 95))


SPHERE_BANDS = 10    # equal steps in z
SPHERE_SECTORS = 20  # equal steps in azimuth


def sphere_bins(dirs: np.ndarray, n_bands: int = SPHERE_BANDS, n_sectors: int = SPHERE_SECTORS) -> np.ndarray:
    """Hit counts in equal-area bins, shape (n_bands, n_sectors).

    Equal steps in z cut equal areas (Archimedes), so a uniform cover gives equal counts.
    Band 0 is z = -1, sector 0 starts at azimuth -pi.
    """
    u = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
    band = np.clip(((u[:, 2] + 1) / 2 * n_bands).astype(int), 0, n_bands - 1)
    phi = np.arctan2(u[:, 1], u[:, 0])
    sector = np.clip(((phi + np.pi) / (2 * np.pi) * n_sectors).astype(int), 0, n_sectors - 1)
    counts = np.bincount(band * n_sectors + sector, minlength=n_bands * n_sectors)
    return counts.reshape(n_bands, n_sectors)


def sphere_bin_edges(n_bands: int = SPHERE_BANDS, n_sectors: int = SPHERE_SECTORS) -> tuple[np.ndarray, np.ndarray]:
    """Bin edges of sphere_bins as (longitude, latitude) in rad, for map projections."""
    lon = np.linspace(-np.pi, np.pi, n_sectors + 1)
    lat = np.arcsin(np.linspace(-1.0, 1.0, n_bands + 1))
    return lon, lat


def sphere_coverage(dirs: np.ndarray, n_bands: int = SPHERE_BANDS, n_sectors: int = SPHERE_SECTORS) -> tuple[float, float]:
    """Equal-area bins (equal steps in z and in azimuth) -> (share of bins hit, std/mean of counts)."""
    counts = sphere_bins(dirs, n_bands, n_sectors)
    return float((counts > 0).mean()), float(counts.std() / counts.mean())


def extra_accel(acc: np.ndarray, g_series: np.ndarray) -> tuple[float, float]:
    """Acceleration beyond gravity, |acc + g|, in g -> (max, rms).

    An accelerometer reads acc = a - g, so acc + g is the real acceleration of the sample.
    """
    e = extra_accel_series(acc, g_series)
    return float(e.max()), float(np.sqrt(np.mean(e**2)))


def extra_accel_series(acc: np.ndarray, g_series: np.ndarray) -> np.ndarray:
    """|acc + g| per sample in g (see extra_accel)."""
    return np.linalg.norm(acc + g_series, axis=1) / G0


def speed_tracking(cmd: np.ndarray, actual: np.ndarray) -> np.ndarray:
    """RMS of commanded minus actual speed per axis (deg/s)."""
    return np.sqrt(np.mean((cmd - actual) ** 2, axis=0))
