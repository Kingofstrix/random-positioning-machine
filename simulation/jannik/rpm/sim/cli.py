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

    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
