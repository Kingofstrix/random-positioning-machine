// Prints the reference runs of the C++ core, to compare with the Python ones
// (tests/test_strategies.py REFERENCE_RUNS). Built and run by tests/test_nano_core.py.
//
// Output, one line per strategy: <name> <timer event hash> <pulse sum> <residual(est)>

#include <stdio.h>

#include "rpm_core.h"

using namespace rpm;

int main() {
  ServoModel model;
  if (!model.set_nominal()) return 1;
  for (int k = 0; k < N_STRATEGIES; ++k) {
    Controller c;
    c.init((StrategyKind)k, 120.0f, 1, model, model);
    uint32_t events = 0;
    unsigned long long pulse_sum = 0;
    unsigned long last0 = 0, last1 = 0;
    bool first = true;
    for (uint32_t tick = 0; tick < 36000; ++tick) {
      int po, pi;
      c.tick(0.1f, &po, &pi);
      pulse_sum += (unsigned long long)(po + pi);
      if (c.strategy.kind == CLINOSTAT) continue;  // no timers: Python's key stays None
      unsigned long t0 = c.strategy.next_change_ms[0];
      unsigned long t1 = c.strategy.kind == RANDOM_WALK ? c.strategy.next_change_ms[1] : 0;
      if (first || t0 != last0 || t1 != last1) {
        events = events * 31u + tick;
        last0 = t0;
        last1 = t1;
        first = false;
      }
    }
    printf("%s %lu %llu %.6f\n", strategy_name((StrategyKind)k), (unsigned long)events, pulse_sum,
           (double)c.estimate.stats.residual());
  }
  return 0;
}
