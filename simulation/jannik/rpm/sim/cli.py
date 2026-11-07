"""Command line: ``rpm-sim run`` for one strategy, ``rpm-sim compare`` for the report."""

import argparse
from pathlib import Path

from rpm.core.strategies import STRATEGIES
from rpm.sim.simulate import ServoPlant, run


def _cmd_run(args: argparse.Namespace) -> None:
    plant = ServoPlant(speed_error=args.speed_error, noise=args.noise)
    result = run(args.strategy, args.duration, seed=args.seed, outer_plant=plant, inner_plant=plant)
    lo, hi = result.coverage_range
    print(f"strategy          {result.strategy}")
    print(f"simulated         {args.duration:.0f} s, seed {args.seed}")
    print(f"residual gravity  {result.residual_gravity:.4f} g (true)")
    print(f"                  {result.estimated_stats.residual_gravity():.4f} g (Pico's estimate)")
    print(f"coverage          {lo:.2f} .. {hi:.2f} (1.0 = uniform)")


def _cmd_compare(args: argparse.Namespace) -> None:
    from rpm.sim.report import compare, write_report  # matplotlib only when needed

    summaries = compare(args.duration, args.seeds)
    path = write_report(summaries, Path(args.out), args.duration)
    for s in summaries:
        print(f"{s.name:18s} worst {s.residuals.max():.4f} g  {'PASS' if s.passes else 'fail'}")
    print(f"report written to {path}")


VIEWER_STRATEGIES = ("clinostat", "random_walk", "random_setpoints")


def _cmd_export(args: argparse.Namespace) -> None:
    from rpm.sim.export import export

    out = Path(args.out) if args.out else None
    path = export(args.strategy, args.minutes, args.seed, args.speed_error, args.noise, out)
    print(f"{args.strategy}: {args.minutes:g} min written to {path}")


def _cmd_viewer(args: argparse.Namespace) -> None:
    """Export the default strategies if missing, serve the repo root and open the viewer."""
    import functools
    import http.server
    import threading
    import webbrowser

    from rpm.sim.export import DATA_DIR, PROJECT_DIR, export, update_index

    viewer = PROJECT_DIR / "viewer" / "index.html"
    if not viewer.is_file():
        raise SystemExit(f"viewer not found at {viewer} (run from a checkout of the repo)")
    for name in VIEWER_STRATEGIES:
        if not (DATA_DIR / f"{name}.json").is_file():
            print(f"exporting {name} ...")
            export(name)
    update_index(DATA_DIR)
    # Serve the repo root: the viewer loads the printed parts from rpm_printables/.
    root = PROJECT_DIR.parents[1]
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(root))
    server = http.server.ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    url = f"http://127.0.0.1:{args.port}/{viewer.relative_to(root).as_posix()}"
    print(f"serving {root} at {url} (Ctrl-C to stop)")
    if not args.no_browser:
        threading.Timer(0.5, webbrowser.open, (url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.server_close()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="rpm-sim", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_run = sub.add_parser("run", help="simulate one strategy and print its metrics")
    p_run.add_argument("strategy", choices=sorted(STRATEGIES))
    p_run.add_argument("--duration", type=float, default=600.0, help="seconds (default 600)")
    p_run.add_argument("--seed", type=int, default=1)
    p_run.add_argument(
        "--speed-error", type=float, default=0.0, help="real servo speed error, e.g. 0.2 for +20 %%"
    )
    p_run.add_argument("--noise", type=float, default=0.0, help="relative speed noise")
    p_run.set_defaults(func=_cmd_run)

    p_cmp = sub.add_parser("compare", help="compare all strategies and write a report")
    p_cmp.add_argument("--duration", type=float, default=600.0)
    p_cmp.add_argument("--seeds", type=int, default=5, help="seeds per scenario")
    p_cmp.add_argument("--out", default="results", help="output folder (default results/)")
    p_cmp.set_defaults(func=_cmd_compare)

    p_exp = sub.add_parser("export", help="simulate one strategy and write JSON for the viewer")
    p_exp.add_argument("strategy", choices=sorted(STRATEGIES))
    p_exp.add_argument("--minutes", type=float, default=10.0, help="simulated time (default 10)")
    p_exp.add_argument("--seed", type=int, default=1)
    p_exp.add_argument("--speed-error", type=float, default=0.0, help="real servo speed error")
    p_exp.add_argument("--noise", type=float, default=0.0, help="relative speed noise")
    p_exp.add_argument("--out", help="output file (default viewer/data/<strategy>.json)")
    p_exp.set_defaults(func=_cmd_export)

    p_view = sub.add_parser("viewer", help="export the strategies and open the 3D viewer")
    p_view.add_argument("--port", type=int, default=8000)
    p_view.add_argument("--no-browser", action="store_true", help="only start the server")
    p_view.set_defaults(func=_cmd_viewer)

    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
