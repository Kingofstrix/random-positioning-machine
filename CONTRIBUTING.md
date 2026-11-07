# Contributing

Three teams work in this repo: **hardware**, **simulation** (logic/simulation) and **testing**.
These rules keep `main` stable and the history readable.

## 1. Never push to `main`
`main` is protected. All changes go through a branch and a pull request (PR).

## 2. Branch naming
`<team>/<type>/<short-description>` in lowercase with hyphens.

| team | type |
|---|---|
| `hw` · `sim` · `test` · `docs` | `feat` · `fix` · `refactor` · `chore` |

Examples: `hw/feat/servo-mount`, `sim/fix/gravity-vector-sign`, `test/feat/imu-logging`

## 3. Folder ownership
| Folder | Owner |
|---|---|
| `hardware/` (STL, CAD, wiring, BOM) | Hardware |
| `simulation/` (control logic, models) | Simulation |
| `testing/` (test plans, scripts, results) | Testing |
| `docs/` | everyone |

Only touch another team's folder if you tag that team in the PR.

## 4. Commits
- Small and focused, one logical change per commit.
- Message format: `<type>(<scope>): <what changed>`, e.g. `feat(sim): add 3D clinostat mode`
- No generated files, logs or large binaries unless they are deliverables (STL, PDF).

## 5. Pull requests
- Fill out the PR template.
- At least **1 approval**, ideally from another team when interfaces are affected.
- All review comments resolved before merging.
- Merge with **squash merge**, then delete the branch.
- Keep PRs small. Over ~400 changed lines (excluding binaries) usually means it should be split.

## 6. Documentation
- Every folder has a `README.md` explaining what is inside and how to use it.
- Code: docstrings for public functions, comments for *why* rather than *what*.
- Hardware changes: update the BOM and note the change in `hardware/README.md`.
- Interfaces between teams (pins, units, data formats, coordinate frames) are documented in `docs/interfaces.md`. Changing them needs a PR approved by every affected team.

## 7. Issues
- Track work as GitHub Issues with a team label (`hardware`, `simulation`, `testing`).
- Reference issues in PRs: `Closes #12`.

## 8. Licence
The printable parts derive from the Core Electronics design (CC BY-NC-SA 4.0). Keep the attribution in `rpm_printables/README.txt` and do not use the design commercially.
