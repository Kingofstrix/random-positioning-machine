# ADR 0002: Constant-speed clinostat as the default strategy

- Status: accepted
- Date: 2026-11-07
- Author: Jannik Greiner

## Context

Candidates, all behind the same `Strategy.step()` interface: the original Core Electronics
behaviour (`random_setpoints`), independent random speeds and directions per axis (`random_walk`),
and a two-axis clinostat with constant speeds (`clinostat`). Judged by the acceptance criteria in
[api.md](../api.md#acceptance-criteria) over the robustness envelope.

## Findings

From `rpm-sim compare` (25 runs of 10 min per strategy, see [`results/`](../../results/README.md)):

| Strategy | median residual | worst residual | coverage |
|---|---:|---:|---|
| `clinostat` | 0.0011 g | 0.0030 g | 0.60–2.10 |
| `random_setpoints` | 0.041 g | 0.094 g | 0.43–2.33 |
| `random_walk` | 0.042 g | 0.098 g | 0.41–2.41 |

Speed matters less than the strategy (30 runs each: envelope corners plus nominal, seeds 1–5):

| max speed | `clinostat` worst | `random_walk` worst |
|---:|---:|---:|
| 30 °/s | 0.011 g | 0.19 g |
| 60 °/s | 0.0046 g | 0.10 g |
| 90 °/s | 0.0025 g | 0.090 g |
| 120 °/s | 0.0030 g | 0.098 g |

So the clinostat would also pass at 60 °/s, half the servo's maximum, if the team prefers to
spare the gears; random motion fails at every speed.

1. **Random motion averages slowly.** Full revolutions at constant speed cancel almost exactly
   every turn; random speed and direction changes leave a residual that shrinks only slowly
   (roughly like 1/√t).
2. **One resonance.** With `g = (-sin i·sin o, -cos i·sin o, -cos o)`, the time average vanishes for
   every speed ratio inner/outer except |ratio| = 1. There the inner angle keeps a fixed phase to
   the outer one and **0.5 g remains** (`test_equal_axis_speeds_do_not_average_out`). The ratio
   1/φ² ≈ 0.382 is irrational and stays below 0.6 even when both servos err by 20 % in opposite
   directions; the golden ratio 0.618 could reach 0.93 and failed with 0.021 g. Whole-microsecond pulses make
   the real ratio rational (8/21 nominally); that is harmless, only a ratio of 1 resonates.
3. **Coverage cannot be uniform without position feedback.** The z component `-cos o` lingers near
   ±1 whenever the outer angle is uniform in time, which it is for every strategy that does not
   know `o`. Gravity then sits along the inner axis about twice as long as it should, for *all*
   strategies (the four high bars in `results/coverage.png`). Uniform coverage needs an outer speed
   that depends on the true outer angle, see the index-sensor note in
   [ADR 0001](0001-open-loop-dead-reckoning.md).

## Decision

- Default: `clinostat`, outer at 120 deg/s (20 rpm), inner at 0.382 × that.
- Coverage criterion [0.5, 2.5]: every direction visited, none dominating, not uniform.
- `random_setpoints` stays as the baseline, `random_walk` for comparison.

## Consequences

- Deterministic motion, identical on every run: easy to test and to reproduce on the machine.
- Both gears always turn the same way at a steady speed: no reversals, little wear.
- If an experiment needs uniform coverage, that is a hardware change first (index sensor).
