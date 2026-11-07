# testing/

Test path for the Random Positioning Machine. Owner: Testing team.

| Path | Content |
|---|---|
| `run_tests.py` | Interactive test runner |
| `steps/<topic>.yaml` | Test steps per topic |
| `results/` | One Markdown report per test run |
| `gravity-estimate/` | Spec, reference code and test values for the Arduino's `gres`/`cov` status fields |

## Setup
Python 3.9 or newer and PyYAML:

```sh
pip install pyyaml
```

## Run
From the repository root:

```sh
python testing/run_tests.py --topic hardware   # only one topic
python testing/run_tests.py                    # all topics in order
```

| Topic | Covers | Owner |
|---|---|---|
| `hardware` | Printed parts, assembly, wiring, power, slip ring | Hardware |
| `software` | Pico firmware and PC simulation | Simulation |
| `test` | Full-system checks on the assembled machine | Testing |

Hardware and software teams run their own topic while developing. The testing team runs all topics, e.g. before a demo.

For each step the runner shows what to do and what to expect. Answer `p` (passed), `f` (failed) or `s` (skipped, e.g. a part is not built yet) and optionally add a comment such as a measured value. A failed step does not stop the run. `Ctrl-C` aborts the run; remaining steps are reported as *not run*.

At the end you get a coloured summary in the terminal: green passed, red failed, yellow skipped.

Exit code: `0` nothing failed, `1` at least one step failed, `2` setup error (bad step file, PyYAML missing), `130` aborted.

## Reports
Every run writes `results/<YYYY-MM-DD_HHMM>_<topic|all>.md` with the git branch and commit, totals and a table of all steps. The commit is marked *dirty* if there were uncommitted changes, so the report shows exactly what was tested.

`main` is protected, so commit the report on a branch and open a PR:

```sh
git checkout -b test/chore/results-2026-11-07
git add testing/results/
git commit -m "chore(test): add hardware test results"
```

## Adding or changing steps
Edit the YAML file of your topic and open a PR. Each step needs:

```yaml
- id: HW-09            # unique, prefix HW / SW / SYS
  title: Short name
  do: What the tester does
  expect: What counts as passed
```

`type` is optional and defaults to `manual`. Automatic steps over the serial interface are planned once the firmware exists. The runner checks the file on start and reports missing fields, duplicate IDs and unsupported types.
