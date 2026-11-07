// Run states and the serial protocol of the Nano firmware, on a PC.
// Built and run by tests/test_nano_core.py; prints "machine tests passed".

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "machine.h"

using namespace rpm;

static char reply[Machine::REPLY_SIZE];

static bool starts_with(const char* s, const char* prefix) {
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

static const char* run(Machine& m, const char* line) {
  m.execute(line, reply);
  return reply;
}

#define CHECK(cond)                                         \
  do {                                                      \
    if (!(cond)) {                                          \
      printf("FAILED line %d: %s | %s\n", __LINE__, #cond, reply); \
      return 1;                                             \
    }                                                       \
  } while (0)

int main() {
  ServoModel model;
  CHECK(model.set_nominal());

  // Number formatting without %f.
  char num[16];
  format_fixed(num, sizeof(num), 0.0123f, 4);
  CHECK(!strcmp(num, "0.0123"));
  format_fixed(num, sizeof(num), -120.0f, 1);
  CHECK(!strcmp(num, "-120.0"));
  format_fixed(num, sizeof(num), 0.00004f, 4);
  CHECK(!strcmp(num, "0.0000"));
  format_fixed(num, sizeof(num), 45.84f, 1);
  CHECK(!strcmp(num, "45.8"));

  Machine m;
  m.init(model, model, CLINOSTAT, 120.0f, 1);

  // The testing team's SW-05 sequence.
  CHECK(starts_with(run(m, "status"), "ok state=STOPPED"));
  CHECK(strstr(reply, "outer_us=off"));
  CHECK(!strcmp(run(m, "start random"), "ok start random_walk"));
  for (int i = 0; i < 50; ++i) {
    ServoCommand c = m.tick(0.1f);
    CHECK(c.on && c.outer_us >= 700 && c.outer_us <= 2300);
  }
  CHECK(starts_with(run(m, "status"), "ok state=RUNNING mode=random_walk"));
  CHECK(strstr(reply, "t=5000"));
  CHECK(!strcmp(run(m, "stop"), "ok stop"));
  CHECK(starts_with(run(m, "status"), "ok state=STOPPING"));
  int ticks = 0;
  float last = 1e9f;
  while (m.state == STOPPING && ticks < 100) {
    ServoCommand c = m.tick(0.1f);
    float speed = fabsf(m.controller.speed_outer) + fabsf(m.controller.speed_inner);
    CHECK(speed <= last);  // ramps down, never up
    last = speed;
    if (m.state == STOPPING) CHECK(c.on);
    ++ticks;
  }
  CHECK(m.state == STOPPED);
  CHECK(ticks <= 50);  // SYS-07: still within about 5 s
  CHECK(!m.tick(0.1f).on);
  CHECK(starts_with(run(m, "status"), "ok state=STOPPED"));
  CHECK(!strcmp(run(m, "estop"), "ok estop"));
  CHECK(starts_with(run(m, "foo"), "err"));
  CHECK(starts_with(run(m, ""), "err"));
  CHECK(starts_with(run(m, "help"), "ok help"));

  // estop cuts the pulses at once, from any state.
  run(m, "start clino3d");
  CHECK(m.tick(0.1f).on);
  run(m, "estop");
  CHECK(m.state == STOPPED && !m.tick(0.1f).on);

  // Unknown modes and parameters are refused without side effects.
  CHECK(starts_with(run(m, "start warp"), "err"));
  CHECK(m.state == STOPPED);
  CHECK(starts_with(run(m, "set speed abc"), "err"));
  CHECK(starts_with(run(m, "set speed -5"), "err"));
  CHECK(starts_with(run(m, "set speed nan"), "err"));
  CHECK(starts_with(run(m, "set seed 1.5"), "err"));
  CHECK(starts_with(run(m, "set color 3"), "err"));
  CHECK(starts_with(run(m, "set speed 60"), "ok"));
  CHECK(starts_with(run(m, "set seed 7"), "ok"));
  run(m, "start clinostat");
  m.tick(0.1f);
  CHECK(m.controller.strategy.max_speed == 60.0f);
  run(m, "set speed 500");  // capped at what the servo curve reaches
  run(m, "start");
  m.tick(0.1f);
  CHECK(m.controller.strategy.max_speed == model.max_speed());

  // mode switches and restarts while running; start without mode keeps it.
  CHECK(!strcmp(run(m, "mode random_setpoints"), "ok mode random_setpoints"));
  CHECK(m.state == RUNNING && m.controller.strategy.kind == RANDOM_SETPOINTS);
  CHECK(starts_with(run(m, "mode"), "err"));

  // Raw pulses only when stopped; the other servo holds at its stop pulse.
  CHECK(starts_with(run(m, "pulse outer 1600"), "err stop first"));
  run(m, "estop");
  CHECK(starts_with(run(m, "pulse outer 2400"), "err"));
  CHECK(starts_with(run(m, "pulse outer 16x0"), "err"));
  CHECK(starts_with(run(m, "pulse middle 1600"), "err"));
  CHECK(!strcmp(run(m, "pulse outer 1600"), "ok pulse outer 1600"));
  ServoCommand c = m.tick(0.1f);
  CHECK(m.state == CAL && c.on && c.outer_us == 1600 && c.inner_us == model.stop_us);
  run(m, "pulse inner 1400");
  c = m.tick(0.1f);
  CHECK(c.outer_us == 1600 && c.inner_us == 1400);
  CHECK(starts_with(run(m, "status"), "ok state=CAL"));
  CHECK(!strcmp(run(m, "pulse off"), "ok pulse off"));
  CHECK(m.state == STOPPED && !m.tick(0.1f).on);

  // A restart reseeds: the same motion as the first run and as the simulation.
  run(m, "set speed 120");
  run(m, "set seed 1");
  run(m, "start random_walk");
  int first[20];
  for (int i = 0; i < 20; ++i) first[i] = m.tick(0.1f).outer_us;
  run(m, "estop");
  run(m, "start random_walk");
  for (int i = 0; i < 20; ++i) CHECK(m.tick(0.1f).outer_us == first[i]);

  // Overlong lines are cut, not overflowed.
  char longline[200];
  memset(longline, 'a', sizeof(longline) - 1);
  longline[sizeof(longline) - 1] = '\0';
  CHECK(starts_with(run(m, longline), "err"));

  // Invalid servo curves are rejected (as in Python's ServoModel).
  ServoModel bad;
  int16_t us[3] = {2400, 2500, 2600};
  float v[3] = {-100.0f, 0.0f, 100.0f};
  CHECK(!bad.set(us, v, 3));  // stop pulse outside 700..2300
  int16_t us2[3] = {1000, 1500, 2000};
  float v2[3] = {-100.0f, NAN, 100.0f};
  CHECK(!bad.set(us2, v2, 3));
  float v3[3] = {-5.0f, 3.0f, 1.0f};
  CHECK(!bad.set(us2, v3, 3));  // not monotone
  CHECK(model.pulse_for(1000.0f) == 1850 && model.pulse_for(-1000.0f) == 1150);
  CHECK(model.pulse_for(0.0f) == 1500);

  printf("machine tests passed\n");
  return 0;
}
