# tests

pytest suite. `tests/firmware/` covers the pure logic in `rpmcore`, `tests/sim/` the MuJoCo model
and closed-loop runs. Long runs are marked `@pytest.mark.slow`.

```bash
pytest -m "not slow"
```

The test plan is in `docs/PLAN.md`, section 8.
