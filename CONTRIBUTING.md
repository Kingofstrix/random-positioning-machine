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
- Keep PRs small: one feature or fix per PR.

## 6. Documentation
- Every folder has a `README.md` explaining what is inside and how to use it.
- Code: docstrings for public functions, comments for *why* rather than *what*.
- Hardware changes: update the BOM and note the change in `hardware/README.md`.
- Interfaces between teams (pins, units, data formats, coordinate frames) are documented in `docs/interfaces.md`. Changing them needs a PR approved by every affected team.
- If behaviour, pins, units or data formats change, update that folder's `README.md` in the same PR.

## 7. Working with AI agents
Everyone on this project codes with AI agents (Claude Code, Copilot, Cursor, …). The rules above apply to agents too, plus:
- Tell your agent to read this file at the start of every session. Most agents don't pick it up on their own.
- The agent can open and review PRs, but it never approves or merges them. A person does that.
- It asks you before changing a shared interface (`docs/interfaces.md`) or restructuring folders.
- If a task touches several teams' folders, it splits the work into separate PRs.
- You're responsible for everything it commits under your name. Read the diff before you open the PR.

## 8. Reviewing a PR
1. Check the rules first: branch name, commit format, folder ownership, README updated, interface changes recorded in `docs/interfaces.md` with the affected teams tagged, no stray files (logs, build output, large binaries).
2. Check that the description matches the diff. Agents sometimes change more than they say.
3. Then check correctness: does the PR do what it says, and how was it tested? Ask for evidence, not just "works".
4. List your findings briefly, most serious first, each with its file and line.

## 9. Issues
- Track work as GitHub Issues with a team label (`hardware`, `simulation`, `testing`).
- Reference issues in PRs: `Closes #12`.

## 10. Licence
The printable parts derive from the Core Electronics design (CC BY-NC-SA 4.0). Keep the attribution in `rpm_printables/README.txt` and do not use the design commercially.
