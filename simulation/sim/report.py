"""Plots and an optional video from the runs of sim.run (docs/PLAN.md 6.8).

python -m sim.report results/            # four PNGs next to every <mode>_s<seed>.npz
MUJOCO_GL=egl python -m sim.report results/ --video

Figures are drawn on matplotlib's Agg canvas directly, so the report never needs a display and
never changes the pyplot backend of a process that imports it (e.g. pytest).
"""

import argparse
import json
import os
import re
from pathlib import Path

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

import sim  # noqa: F401  (puts firmware/ on sys.path)
from sim.metrics import extra_accel_series, residual_g_curve, sphere_bin_edges, sphere_bins
from sim.params import MachineParams, ReportParams

RUN_NAME = re.compile(r"^[A-Za-z0-9]+_s\d+$")
AXES = ("inner", "outer")  # column order of w_dps / cmd_dps
VIDEO_HINT = 'The video needs imageio: pip install -e ".[video]"'


def _figure(rp: ReportParams) -> Figure:
    fig = Figure(figsize=rp.fig_size, dpi=rp.dpi)
    FigureCanvasAgg(fig)
    return fig


def plot_residual_g(run: dict, rp: ReportParams) -> Figure:
    fig = _figure(rp)
    ax = fig.add_subplot()
    ax.semilogy(run["t"], residual_g_curve(run["g_sample"]))
    ax.set(xlabel="time [s]", ylabel="residual g [g]", title="Residual gravity (cumulative mean)")
    ax.grid(True, which="both", alpha=0.3)
    return fig


def plot_sphere(run: dict, rp: ReportParams) -> Figure:
    """Gravity directions in the sample frame. Bins are equal-area, so uniform cover is flat 1."""
    counts = sphere_bins(run["g_sample"])
    lon, lat = sphere_bin_edges(*counts.shape)
    fig = _figure(rp)
    ax = fig.add_subplot(projection="hammer")
    mesh = ax.pcolormesh(*np.meshgrid(lon, lat), counts / counts.mean(), cmap="viridis", shading="flat")
    fig.colorbar(mesh, ax=ax, label="hits / uniform mean")
    fig.suptitle("Gravity direction in the sample frame (Hammer, equal-area bins)")
    ax.xaxis.set_ticklabels([])  # azimuth labels overlap the map; latitude is what matters
    ax.grid(True, alpha=0.4)
    return fig


def plot_speed(run: dict, rp: ReportParams) -> Figure:
    fig = _figure(rp)
    axs = fig.subplots(len(AXES), 1, sharex=True)
    for i, (ax, name) in enumerate(zip(axs, AXES)):
        ax.plot(run["t"], run["cmd_dps"][:, i], label="commanded")
        ax.plot(run["t"], run["w_dps"][:, i], label="actual")
        ax.set_ylabel("%s [deg/s]" % name)
        ax.grid(True, alpha=0.3)
    axs[0].legend(loc="upper right")
    axs[0].set_title("Frame speed")
    axs[-1].set_xlabel("time [s]")
    return fig


def plot_extra_accel(run: dict, rp: ReportParams) -> Figure:
    fig = _figure(rp)
    ax = fig.add_subplot()
    ax.plot(run["t"], extra_accel_series(run["acc_sample"], run["g_sample"]))
    ax.set(xlabel="time [s]", ylabel="|acc + g| [g]", title="Extra acceleration of the sample")
    ax.grid(True, alpha=0.3)
    return fig


PLOTS = {"residual_g": plot_residual_g, "sphere": plot_sphere, "speed": plot_speed,
         "extra_accel": plot_extra_accel}


def load_run(path: Path) -> dict:
    with np.load(path) as z:
        run = {k: z[k] for k in z.files if k != "meta"}
        run["meta"] = json.loads(str(z["meta"]))
    return run


def write_plots(path: Path, rp: ReportParams) -> list[Path]:
    run = load_run(path)
    out = []
    for name, plot in PLOTS.items():
        png = path.with_name("%s_%s.png" % (path.stem, name))
        plot(run, rp).savefig(png)
        out.append(png)
    return out


def frame_times(rp: ReportParams, t_end: float) -> np.ndarray:
    """Sim times shown by the video frames; a run shorter than the time-lapse ends it early."""
    n = int(round(rp.video_seconds * rp.video_fps))
    t = np.arange(n) * rp.video_speedup / rp.video_fps
    return t[t <= t_end]


def replay_qpos(model, data, q_outer: float, q_inner: float, gear_ratio: float) -> None:
    """Pose the model from logged frame angles. Pinions follow the gear constraint (-ratio x frame)."""
    import mujoco

    for joint, q in (("outer", q_outer), ("inner", q_inner),
                     ("outer_pinion", -gear_ratio * q_outer), ("inner_pinion", -gear_ratio * q_inner)):
        data.qpos[model.jnt_qposadr[model.joint(joint).id]] = q
    mujoco.mj_kinematics(model, data)


def _import_imageio():
    try:
        import imageio.v2 as iio
    except ImportError as e:
        raise SystemExit(VIDEO_HINT) from e
    return iio


def _machine_params(meta: dict) -> MachineParams:
    """The run's own geometry (e.g. payload offset); JSON turned its tuples into lists."""
    known = MachineParams.__dataclass_fields__
    kw = {k: tuple(v) if isinstance(v, list) else v for k, v in meta.get("params", {}).items() if k in known}
    return MachineParams(**kw)


def render_video(path: Path, out: Path, rp: ReportParams) -> Path:
    """Replay the logged angles offscreen (no physics) and write an MP4 time-lapse."""
    iio = _import_imageio()
    # Only takes effect if mujoco is not imported yet; the CLI imports it no earlier than here.
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco

    from sim.machine import SimMachine

    run = load_run(path)
    p = _machine_params(run["meta"])
    machine = SimMachine(p)
    m, d = machine.model, machine.data
    t = frame_times(rp, float(run["t"][-1]))
    q_outer = np.interp(t, run["t"], run["q_outer"])
    q_inner = np.interp(t, run["t"], run["q_inner"])
    cam = mujoco.MjvCamera()
    cam.lookat[:] = m.stat.center
    cam.distance = rp.cam_distance * m.stat.extent
    cam.azimuth, cam.elevation = rp.cam_azimuth, rp.cam_elevation
    with mujoco.Renderer(m, rp.video_height, rp.video_width) as renderer:
        with iio.get_writer(str(out), fps=rp.video_fps) as writer:
            for qo, qi in zip(q_outer, q_inner):
                replay_qpos(m, d, qo, qi, p.gear_ratio)
                renderer.update_scene(d, camera=cam)
                writer.append_data(renderer.render())
    return out


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results", type=Path, help="folder with <mode>_s<seed>.npz files from sim.run")
    ap.add_argument("--video", action="store_true", help="also render an MP4 time-lapse per run")
    a = ap.parse_args(argv)
    runs = sorted(p for p in a.results.glob("*.npz") if RUN_NAME.match(p.stem))
    if not runs:
        raise SystemExit("no <mode>_s<seed>.npz in %s (run python -m sim.run first)" % a.results)
    if a.video:
        _import_imageio()  # fail before the slow part, not after the plots
    rp = ReportParams()
    for path in runs:
        for png in write_plots(path, rp):
            print(png)
        if a.video:
            print(render_video(path, path.with_suffix(".mp4"), rp))


if __name__ == "__main__":
    main()
