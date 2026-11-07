// Run states and the serial command protocol of the Nano firmware.
//
// Protocol as in docs/PLAN.md 5.6 (line based, replies start with "ok" or
// "err"), so the testing team's steps in testing/steps/ work unchanged:
//
//   help                        list commands
//   status                      state, mode, speeds, pulses, time, residual(est)
//   start [mode]                start (or restart) a strategy
//   stop                        ramp both axes down, then switch the servos off
//   estop                       switch the servos off at once
//   mode <name>                 strategy for the next start (restarts if running)
//   set speed <deg/s> | set seed <n>
//   pulse <outer|inner> <us>    raw pulse for calibration (only when stopped)
//   pulse off                   end calibration
//
// Mode names: clinostat, random_walk, random_setpoints, and PLAN's aliases
// clino3d (= clinostat) and random (= random_walk).
//
// No Arduino headers here either: tests/nano/machine_test.cpp runs it on a PC.

#ifndef RPM_MACHINE_H
#define RPM_MACHINE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rpm_core.h"

// Format strings live in flash on the AVR: in RAM they took ~700 of its 2048 bytes.
#ifdef __AVR__
#include <avr/pgmspace.h>
#define RPM_FMT(s) PSTR(s)
#define RPM_SNPRINTF snprintf_P
#else
#define RPM_FMT(s) (s)
#define RPM_SNPRINTF snprintf
#endif

namespace rpm {

enum RunState { STOPPED = 0, RUNNING = 1, STOPPING = 2, CAL = 3 };

inline const char* state_name(RunState s) {
  switch (s) {
    case RUNNING:
      return "RUNNING";
    case STOPPING:
      return "STOPPING";
    case CAL:
      return "CAL";
    default:
      return "STOPPED";
  }
}

// Parses a mode name; returns false if unknown.
inline bool parse_mode(const char* name, StrategyKind* kind) {
  if (!strcmp(name, "clinostat") || !strcmp(name, "clino3d")) {
    *kind = CLINOSTAT;
  } else if (!strcmp(name, "random_walk") || !strcmp(name, "random")) {
    *kind = RANDOM_WALK;
  } else if (!strcmp(name, "random_setpoints")) {
    *kind = RANDOM_SETPOINTS;
  } else {
    return false;
  }
  return true;
}

// Fixed-point text for a float, because avr-libc's printf has no %f. Uses
// only %ld, which every printf has (no width from '*').
inline void format_fixed(char* buf, int size, float v, int decimals) {
  long scale = 1;
  for (int i = 0; i < decimals; ++i) scale *= 10;
  bool negative = v < 0.0f;
  long scaled = (long)floorf((negative ? -v : v) * scale + 0.5f);
  int n = RPM_SNPRINTF(buf, size, RPM_FMT("%s%ld"), negative && scaled ? "-" : "", scaled / scale);
  if (decimals <= 0 || n < 0 || n + decimals + 2 > size) return;
  buf[n++] = '.';
  long frac = scaled % scale;
  for (int i = decimals - 1; i >= 0; --i) {
    buf[n + i] = (char)('0' + frac % 10);
    frac /= 10;
  }
  buf[n + decimals] = '\0';
}

// What the servo pins should do this tick.
struct ServoCommand {
  bool on;  // false: no pulses at all (servo limp, PWM off)
  int outer_us;
  int inner_us;
};

class Machine {
 public:
  static constexpr float STOP_DECEL = 60.0f;  // deg/s^2: 120 deg/s ramps down in 2 s
  static const int REPLY_SIZE = 160;

  void init(const ServoModel& outer, const ServoModel& inner, StrategyKind mode, float max_speed,
            uint32_t seed) {
    outer_model_ = &outer;
    inner_model_ = &inner;
    mode_ = mode;
    max_speed_ = max_speed;
    seed_ = seed;
    state = STOPPED;
    elapsed_ms = 0;
    command_.on = false;
    command_.outer_us = outer.stop_us;
    command_.inner_us = inner.stop_us;
  }

  void start() {
    controller.init(mode_, max_speed_, seed_, *outer_model_, *inner_model_);
    controller.speed_outer = controller.speed_inner = 0.0f;
    state = RUNNING;
    elapsed_ms = 0;
  }
  void stop() {
    if (state == RUNNING) state = STOPPING;
  }
  void estop() {
    state = STOPPED;
    command_.on = false;
  }

  // Advance one control period; returns what to write to the servos.
  ServoCommand tick(float dt) {
    if (state == RUNNING) {
      controller.tick(dt, &command_.outer_us, &command_.inner_us);
      command_.on = true;
      elapsed_ms += (unsigned long)(dt * 1000.0f + 0.5f);
    } else if (state == STOPPING) {
      float step = STOP_DECEL * dt;
      controller.speed_outer = toward_zero(controller.speed_outer, step);
      controller.speed_inner = toward_zero(controller.speed_inner, step);
      command_.outer_us = outer_model_->pulse_for(controller.speed_outer);
      command_.inner_us = inner_model_->pulse_for(controller.speed_inner);
      controller.estimate.advance(outer_model_->speed_at(command_.outer_us),
                                  inner_model_->speed_at(command_.inner_us), dt);
      command_.on = true;
      elapsed_ms += (unsigned long)(dt * 1000.0f + 0.5f);
      if (controller.speed_outer == 0.0f && controller.speed_inner == 0.0f) {
        state = STOPPED;
        command_.on = false;
      }
    } else if (state == STOPPED) {
      command_.on = false;
    }
    return command_;  // CAL: keeps the raw pulses set by the command
  }

  // Executes one command line; writes the reply (without newline) into reply.
  void execute(const char* line, char* reply) {
    char buf[48];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* words[4] = {0, 0, 0, 0};
    int n = 0;
    for (char* tok = strtok(buf, " \t\r\n"); tok && n < 4; tok = strtok(0, " \t\r\n")) {
      words[n++] = tok;
    }
    if (n == 0) {
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err empty line"));
      return;
    }
    const char* cmd = words[0];
    if (!strcmp(cmd, "help")) {
      RPM_SNPRINTF(reply, REPLY_SIZE,
               RPM_FMT("ok help status start [mode] stop estop mode <name> set speed|seed <v> "
               "pulse <outer|inner> <us> pulse off"));
    } else if (!strcmp(cmd, "status")) {
      status(reply);
    } else if (!strcmp(cmd, "start")) {
      if (n >= 2 && !parse_mode(words[1], &mode_)) {
        RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err unknown mode %s"), words[1]);
        return;
      }
      start();
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("ok start %s"), strategy_name(mode_));
    } else if (!strcmp(cmd, "stop")) {
      stop();
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("ok stop"));
    } else if (!strcmp(cmd, "estop")) {
      estop();
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("ok estop"));
    } else if (!strcmp(cmd, "mode")) {
      if (n < 2 || !parse_mode(words[1], &mode_)) {
        RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err usage: mode clinostat|random_walk|random_setpoints"));
        return;
      }
      if (state == RUNNING || state == STOPPING) start();
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("ok mode %s"), strategy_name(mode_));
    } else if (!strcmp(cmd, "set")) {
      set(n, words, reply);
    } else if (!strcmp(cmd, "pulse")) {
      pulse(n, words, reply);
    } else {
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err unknown command %s, try help"), cmd);
    }
  }

  RunState state;
  Controller controller;
  unsigned long elapsed_ms;

 private:
  static float toward_zero(float v, float step) {
    if (v > step) return v - step;
    if (v < -step) return v + step;
    return 0.0f;
  }

  void status(char* reply) {
    char o[12], i[12], r[12];
    bool moving = state == RUNNING || state == STOPPING;
    format_fixed(o, sizeof(o), moving ? controller.speed_outer : 0.0f, 1);
    format_fixed(i, sizeof(i), moving ? controller.speed_inner : 0.0f, 1);
    format_fixed(r, sizeof(r), controller.estimate.stats.residual(), 4);
    if (command_.on) {
      RPM_SNPRINTF(reply, REPLY_SIZE,
               RPM_FMT("ok state=%s mode=%s outer_dps=%s inner_dps=%s outer_us=%d inner_us=%d t=%lu "
               "residual_est=%s"),
               state_name(state), strategy_name(mode_), o, i, command_.outer_us, command_.inner_us,
               elapsed_ms, r);
    } else {
      RPM_SNPRINTF(reply, REPLY_SIZE,
               RPM_FMT("ok state=%s mode=%s outer_dps=%s inner_dps=%s outer_us=off inner_us=off t=%lu "
               "residual_est=%s"),
               state_name(state), strategy_name(mode_), o, i, elapsed_ms, r);
    }
  }

  void set(int n, char** words, char* reply) {
    if (n < 3) {
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err usage: set speed <deg/s> | set seed <n>"));
      return;
    }
    char* end = 0;
    double v = strtod(words[2], &end);
    if (end == words[2] || *end != '\0' || !isfinite((float)v)) {
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err not a number: %s"), words[2]);
    } else if (!strcmp(words[1], "speed")) {
      if (v <= 0.0 || v > 1000.0) {
        RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err speed must be in (0, 1000] deg/s"));
        return;
      }
      max_speed_ = (float)v;  // capped at the drives' maximum on start
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("ok speed applies on next start"));
    } else if (!strcmp(words[1], "seed")) {
      if (v < 0.0 || v > 4294967295.0 || v != floor(v)) {
        RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err seed must be a whole number >= 0"));
        return;
      }
      seed_ = (uint32_t)v;
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("ok seed applies on next start"));
    } else {
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err unknown parameter %s"), words[1]);
    }
  }

  void pulse(int n, char** words, char* reply) {
    if (n == 2 && !strcmp(words[1], "off")) {
      if (state == CAL) estop();
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("ok pulse off"));
      return;
    }
    if (state == RUNNING || state == STOPPING) {
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err stop first (stop or estop)"));
      return;
    }
    if (n < 3 || (strcmp(words[1], "outer") && strcmp(words[1], "inner"))) {
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err usage: pulse <outer|inner> <us> | pulse off"));
      return;
    }
    char* end = 0;
    long us = strtol(words[2], &end, 10);
    if (end == words[2] || *end != '\0' || us < MIN_PULSE_US || us > MAX_PULSE_US) {
      RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("err pulse must be %d..%d us"), MIN_PULSE_US, MAX_PULSE_US);
      return;
    }
    if (state != CAL) {  // entering calibration: the other servo holds still
      command_.outer_us = outer_model_->stop_us;
      command_.inner_us = inner_model_->stop_us;
    }
    state = CAL;
    command_.on = true;
    if (!strcmp(words[1], "outer")) {
      command_.outer_us = (int)us;
    } else {
      command_.inner_us = (int)us;
    }
    RPM_SNPRINTF(reply, REPLY_SIZE, RPM_FMT("ok pulse %s %ld"), words[1], us);
  }

  const ServoModel* outer_model_;
  const ServoModel* inner_model_;
  StrategyKind mode_;
  float max_speed_;
  uint32_t seed_;
  ServoCommand command_;
};

}  // namespace rpm

#endif  // RPM_MACHINE_H
