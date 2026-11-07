#!/usr/bin/env python3
"""Interactive test runner for the Random Positioning Machine.

Walks the tester through the steps defined in testing/steps/<topic>.yaml,
prints a coloured summary and writes a Markdown report to testing/results/.

Usage:
    python testing/run_tests.py --topic hardware
    python testing/run_tests.py            # all topics

Exit code: 0 if no step failed, 1 if at least one failed, 2 on a setup
error (bad step file, missing PyYAML), 130 if the run was aborted.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

try:
    import yaml
except ImportError:
    # Exit 2 (setup error), not 1, so it is never mistaken for a failed test.
    print("PyYAML is missing. Install it with: pip install pyyaml", file=sys.stderr)
    sys.exit(2)

TESTING_DIR = Path(__file__).resolve().parent
STEPS_DIR = TESTING_DIR / "steps"
RESULTS_DIR = TESTING_DIR / "results"

# Order matters: a run without --topic goes bench-first, system last.
TOPICS = ("hardware", "software", "test")

PASS, FAIL, SKIP, NOT_RUN = "passed", "failed", "skipped", "not run"
STEP_TYPES = ("manual",)  # "auto" (serial) steps are planned, not supported yet
REQUIRED_FIELDS = ("id", "title", "do", "expect")


@dataclass
class Step:
    topic: str
    id: str
    title: str
    do: str
    expect: str
    status: str = NOT_RUN
    comment: str = ""


class StepFileError(Exception):
    pass


# --- colours ---------------------------------------------------------------

def _use_colour() -> bool:
    # Respect https://no-color.org and don't print escape codes into pipes.
    return sys.stdout.isatty() and "NO_COLOR" not in os.environ


COLOURS = {PASS: "32", FAIL: "31", SKIP: "33", NOT_RUN: "90"}


def colour(text: str, status: str) -> str:
    if not _use_colour():
        return text
    return f"\033[{COLOURS[status]}m{text}\033[0m"


def bold(text: str) -> str:
    return f"\033[1m{text}\033[0m" if _use_colour() else text


# --- loading steps ---------------------------------------------------------

def load_steps(topic: str, steps_dir: Path = STEPS_DIR) -> list[Step]:
    """Read and validate the step file for one topic."""
    path = steps_dir / f"{topic}.yaml"
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise StepFileError(f"{path}: file not found")
    except yaml.YAMLError as e:
        raise StepFileError(f"{path}: invalid YAML: {e}")

    if not isinstance(data, dict) or not isinstance(data.get("steps"), list):
        raise StepFileError(f"{path}: expected a mapping with a 'steps' list")

    steps, seen = [], set()
    for i, raw in enumerate(data["steps"], start=1):
        where = f"{path}, step {i}"
        if not isinstance(raw, dict):
            raise StepFileError(f"{where}: expected a mapping")
        missing = [f for f in REQUIRED_FIELDS if not raw.get(f)]
        if missing:
            raise StepFileError(f"{where}: missing {', '.join(missing)}")
        step_type = raw.get("type", "manual")
        if step_type not in STEP_TYPES:
            raise StepFileError(f"{where}: type '{step_type}' is not supported yet")
        sid = str(raw["id"])
        if sid in seen:
            raise StepFileError(f"{where}: duplicate id {sid}")
        seen.add(sid)
        steps.append(Step(topic, sid, str(raw["title"]).strip(),
                          str(raw["do"]).strip(), str(raw["expect"]).strip()))
    if not steps:
        raise StepFileError(f"{path}: no steps defined")
    return steps


# --- running ---------------------------------------------------------------

ANSWERS = {"p": PASS, "f": FAIL, "s": SKIP}


def ask_status() -> str:
    while True:
        answer = input("  Result? [p]ass / [f]ail / [s]kip: ").strip().lower()
        if answer[:1] in ANSWERS:
            return ANSWERS[answer[:1]]
        print("  Please type p, f or s.")


def run_step(step: Step, number: int, total: int) -> None:
    print()
    print(bold(f"[{number}/{total}] {step.id} – {step.title}"))
    print(f"  Do:     {step.do}")
    print(f"  Expect: {step.expect}")
    step.status = ask_status()
    step.comment = input("  Comment (optional, Enter to skip): ").strip()
    print("  -> " + colour(step.status.upper(), step.status))


def run(steps: list[Step]) -> bool:
    """Run all steps. Returns False if the tester aborted with Ctrl-C/Ctrl-D."""
    current_topic = None
    for n, step in enumerate(steps, start=1):
        if step.topic != current_topic:
            current_topic = step.topic
            print("\n" + bold(f"=== Topic: {current_topic} ==="))
        try:
            run_step(step, n, len(steps))
        except (KeyboardInterrupt, EOFError):
            # Keep what was answered so far; the rest stays "not run".
            step.status, step.comment = NOT_RUN, ""
            print("\n\nRun aborted.")
            return False
    return True


# --- reporting -------------------------------------------------------------

def git_info() -> dict[str, str]:
    def git(*args: str) -> str:
        try:
            out = subprocess.run(["git", *args], cwd=TESTING_DIR, check=True,
                                 capture_output=True, text=True)
            return out.stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            return ""

    commit = git("rev-parse", "--short", "HEAD") or "unknown"
    # Untracked results from earlier runs don't make the tested code dirty.
    dirty = git("status", "--porcelain", "--untracked-files=no")
    return {
        "branch": git("rev-parse", "--abbrev-ref", "HEAD") or "unknown",
        "commit": commit + (" (dirty: uncommitted changes)" if dirty else ""),
    }


def counts(steps: list[Step]) -> dict[str, int]:
    return {s: sum(1 for st in steps if st.status == s)
            for s in (PASS, FAIL, SKIP, NOT_RUN)}


def print_summary(steps: list[Step]) -> None:
    print("\n" + bold("=== Summary ==="))
    width = max(len(s.id) for s in steps)
    for s in steps:
        line = f"  {colour(s.status.upper().ljust(8), s.status)} {s.id.ljust(width)}  {s.title}"
        print(line + (f"  ({s.comment})" if s.comment else ""))
    c = counts(steps)
    print()
    print("  " + ", ".join(colour(f"{c[k]} {k}", k) for k in (PASS, FAIL, SKIP, NOT_RUN) if c[k]))


def _cell(text: str) -> str:
    # Keep Markdown table cells on one line and unbroken.
    return text.replace("|", "\\|").replace("\n", " ")


ICONS = {PASS: "✅ passed", FAIL: "❌ failed", SKIP: "⚠️ skipped", NOT_RUN: "⏸️ not run"}


def render_report(steps: list[Step], label: str, started: datetime,
                  finished: datetime, completed: bool, git: dict[str, str]) -> str:
    c = counts(steps)
    result = "FAILED" if c[FAIL] else ("INCOMPLETE" if not completed or c[NOT_RUN] else "PASSED")
    lines = [
        f"# Test report: {label}",
        "",
        f"- **Result:** {result}",
        f"- **Branch:** `{git['branch']}`",
        f"- **Commit:** `{git['commit']}`",
        f"- **Started:** {started:%Y-%m-%d %H:%M}",
        f"- **Finished:** {finished:%Y-%m-%d %H:%M}" + ("" if completed else " (aborted)"),
        f"- **Totals:** {c[PASS]} passed, {c[FAIL]} failed, {c[SKIP]} skipped, {c[NOT_RUN]} not run",
    ]
    for topic in dict.fromkeys(s.topic for s in steps):
        lines += ["", f"## {topic}", "", "| ID | Step | Status | Comment |", "|---|---|---|---|"]
        lines += [f"| {s.id} | {_cell(s.title)} | {ICONS[s.status]} | {_cell(s.comment)} |"
                  for s in steps if s.topic == topic]
    return "\n".join(lines) + "\n"


def write_report(text: str, label: str, started: datetime,
                 results_dir: Path = RESULTS_DIR) -> Path:
    results_dir.mkdir(parents=True, exist_ok=True)
    base = f"{started:%Y-%m-%d_%H%M}_{label}"
    path, n = results_dir / f"{base}.md", 2
    while path.exists():  # two runs in the same minute must not overwrite each other
        path, n = results_dir / f"{base}-{n}.md", n + 1
    path.write_text(text, encoding="utf-8")
    return path


# --- main ------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the RPM test path.")
    parser.add_argument("--topic", choices=TOPICS,
                        help="run only this topic (default: all topics)")
    args = parser.parse_args(argv)

    topics = [args.topic] if args.topic else list(TOPICS)
    label = args.topic or "all"
    try:
        steps = [step for t in topics for step in load_steps(t)]
    except StepFileError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    started = datetime.now()
    print(bold(f"RPM test run – topic: {label} – {len(steps)} steps"))
    completed = run(steps)
    finished = datetime.now()

    print_summary(steps)
    report = write_report(render_report(steps, label, started, finished, completed, git_info()),
                          label, started)
    print(f"\n  Report written to {report.relative_to(TESTING_DIR.parent)}")
    print("  Commit it on a test/chore/... branch and open a PR.")

    if not completed:
        return 130
    return 1 if counts(steps)[FAIL] else 0


if __name__ == "__main__":
    sys.exit(main())
