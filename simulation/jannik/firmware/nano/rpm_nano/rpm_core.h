// Motion logic of the RPM for the Arduino Nano (ATmega328P), ported from rpm/core.
//
// This is a line-by-line port of the Python package rpm.core, which stays the
// reference: the simulation (rpm.sim) tests the Python version, and
// tests/test_nano_core.py compiles this header on the PC and checks that it
// produces the same motion (timer events exactly, pulses within rounding) as
// the Python reference runs in tests/test_strategies.py.
//
// Rules for this file:
// - No Arduino headers: it must also compile on a PC for the tests.
// - float only (on the AVR, double is float anyway), so the PC build sees the
//   same 32-bit arithmetic as the Nano.
// - No heap: everything is statically sized.
//
// Conventions as in docs/interfaces.md: axis outer turns the inner frame about
// x, axis inner turns the platform about the inner frame's z; speeds in deg/s,
// pulses in whole microseconds.

#ifndef RPM_CORE_H
#define RPM_CORE_H

#include <math.h>
#include <stdint.h>

namespace rpm {

const float PI_F = 3.14159265358979f;
const float TWO_PI_F = 6.28318530717959f;
const float DEG = PI_F / 180.0f;

// ---------------------------------------------------------------- kinematics

struct Vec3 {
  float x, y, z;
};

// Unit gravity in the sample frame for R = Rx(outer) * Rz(inner).
inline Vec3 gravity_in_sample(float theta_outer, float theta_inner) {
  float s_o = sinf(theta_outer);
  Vec3 g = {-sinf(theta_inner) * s_o, -cosf(theta_inner) * s_o, -cosf(theta_outer)};
  return g;
}

// Map to [0, 2 pi), like Python's ``theta % TWO_PI``.
inline float wrap_angle(float theta) {
  float r = fmodf(theta, TWO_PI_F);
  return r < 0.0f ? r + TWO_PI_F : r;
}

// Python int(round(x)) for the values we meet (halfway cases are vanishingly
// rare with real-valued speeds; tests allow for them).
inline long round_to_long(float x) { return (long)floorf(x + 0.5f); }

// ----------------------------------------------------------------------- rng

// xorshift32, identical sequence to rpm.core.rng.Rng.
class Rng {
 public:
  explicit Rng(uint32_t seed = 1) { reseed(seed); }
  void reseed(uint32_t seed) { state_ = seed ? seed : 0x9E3779B9UL; }
  uint32_t next_u32() {
    uint32_t x = state_;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state_ = x;
    return x;
  }
  float random() { return (float)(next_u32() >> 8) / 16777216.0f; }  // 24 bits, < 1
  float uniform(float low, float high) { return low + (high - low) * random(); }
  long randint(long low, long high) {
    return low + (long)(next_u32() % (uint32_t)(high - low + 1));
  }
  float sign() { return (next_u32() & 1UL) ? 1.0f : -1.0f; }

 private:
  uint32_t state_;
};

// -------------------------------------------------------------- servo model

const int MIN_PULSE_US = 700;
const int MAX_PULSE_US = 2300;
const int STOP_US = 1500;
const int MAX_SERVO_POINTS = 16;

// Monotone curve (pulse_us, axis_deg_s) of one drive, see rpm/core/servo.py.
class ServoModel {
 public:
  ServoModel() : stop_us(STOP_US), max_forward(0), max_reverse(0), n_(0), increasing_(true) {}

  // Points must be sorted by pulse. Returns false (and keeps the model empty)
  // if they are invalid; same rules as the Python ServoModel.
  bool set(const int16_t* pulses, const float* speeds, int n) {
    n_ = 0;
    if (n < 2 || n > MAX_SERVO_POINTS) return false;
    for (int i = 0; i < n; ++i) {
      if (!isfinite(speeds[i])) return false;
      if (i > 0 && pulses[i] <= pulses[i - 1]) return false;
    }
    bool inc = speeds[n - 1] > speeds[0];
    for (int i = 1; i < n; ++i) {
      if ((inc && speeds[i] < speeds[i - 1]) || (!inc && speeds[i] > speeds[i - 1])) return false;
    }
    if (speeds[0] * speeds[n - 1] >= 0.0f) return false;
    for (int i = 0; i < n; ++i) {
      us_[i] = pulses[i];
      v_[i] = speeds[i];
    }
    n_ = n;
    increasing_ = inc;
    int lo = -1, hi = -1;
    for (int i = 0; i < n; ++i) {
      if (v_[i] == 0.0f) {
        if (lo < 0) lo = i;
        hi = i;
      }
    }
    float stop = lo >= 0 ? (us_[lo] + us_[hi]) / 2.0f : interp_pulse(0.0f);
    long s = round_to_long(stop);
    if (s < MIN_PULSE_US || s > MAX_PULSE_US) {
      n_ = 0;
      return false;
    }
    stop_us = (int)s;
    max_forward = v_[0] > v_[n - 1] ? v_[0] : v_[n - 1];
    max_reverse = v_[0] < v_[n - 1] ? v_[0] : v_[n - 1];
    return true;
  }

  // PLAN 2.5: linear outside +-45 us around the stop pulse, saturated beyond +-350 us.
  bool set_nominal(float servo_rpm = 83.0f, float direction = 1.0f, int stop = STOP_US) {
    float axis = servo_rpm * 6.0f / 4.0f * direction;
    int16_t us[4] = {(int16_t)(stop - 350), (int16_t)(stop - 45), (int16_t)(stop + 45),
                     (int16_t)(stop + 350)};
    float v[4] = {-axis, 0.0f, 0.0f, axis};
    return set(us, v, 4);
  }

  bool valid() const { return n_ >= 2; }

  float max_speed() const { return max_forward < -max_reverse ? max_forward : -max_reverse; }

  float speed_at(float pulse_us) const {
    if (pulse_us <= us_[0]) return v_[0];
    if (pulse_us >= us_[n_ - 1]) return v_[n_ - 1];
    for (int i = 1; i < n_; ++i) {
      if (pulse_us <= us_[i]) {
        return v_[i - 1] + (v_[i] - v_[i - 1]) * (pulse_us - us_[i - 1]) / (us_[i] - us_[i - 1]);
      }
    }
    return v_[n_ - 1];
  }

  int pulse_for(float speed) const {
    long pulse;
    if (speed == 0.0f) {
      pulse = stop_us;
    } else {
      if (speed > max_forward) speed = max_forward;
      if (speed < max_reverse) speed = max_reverse;
      pulse = round_to_long(interp_pulse(speed));
    }
    if (pulse < MIN_PULSE_US) pulse = MIN_PULSE_US;
    if (pulse > MAX_PULSE_US) pulse = MAX_PULSE_US;
    return (int)pulse;
  }

  int stop_us;
  float max_forward;
  float max_reverse;

 private:
  float interp_pulse(float speed) const {
    for (int i = 1; i < n_; ++i) {
      float v0 = v_[i - 1], v1 = v_[i];
      float lo = v0 <= v1 ? v0 : v1, hi = v0 <= v1 ? v1 : v0;
      if (lo <= speed && speed <= hi && v1 != v0) {
        return us_[i - 1] + (us_[i] - us_[i - 1]) * (speed - v0) / (v1 - v0);
      }
    }
    return ((speed < 0.0f) == increasing_) ? us_[0] : us_[n_ - 1];
  }

  int n_;
  int16_t us_[MAX_SERVO_POINTS];
  float v_[MAX_SERVO_POINTS];
  bool increasing_;
};

// ------------------------------------------------------------------ metrics

// Time-weighted mean gravity with Kahan sums (float32 would lose ~1 s/h).
// Unlike Python's GravityStats there are no coverage cells: 2 KB of RAM.
class GravityStats {
 public:
  GravityStats() { reset(); }
  void reset() {
    for (int i = 0; i < 4; ++i) sum_[i] = carry_[i] = 0.0f;
  }
  void add(const Vec3& g, float dt) {
    acc(0, dt);
    acc(1, g.x * dt);
    acc(2, g.y * dt);
    acc(3, g.z * dt);
  }
  float total_time() const { return sum_[0]; }
  Vec3 mean() const {
    Vec3 m = {0.0f, 0.0f, 0.0f};
    if (sum_[0] > 0.0f) {
      m.x = sum_[1] / sum_[0];
      m.y = sum_[2] / sum_[0];
      m.z = sum_[3] / sum_[0];
    }
    return m;
  }
  float residual() const {
    Vec3 m = mean();
    return sqrtf(m.x * m.x + m.y * m.y + m.z * m.z);
  }

 private:
  void acc(int i, float v) {
    float y = v - carry_[i];
    float s = sum_[i] + y;
    carry_[i] = (s - sum_[i]) - y;
    sum_[i] = s;
  }
  float sum_[4];
  float carry_[4];
};

// --------------------------------------------------------------- controller

const float RESPONSE_LAG_S = 0.1f;

// Exact first-order lag: speed after t seconds, and distance covered (speed * s).
inline void first_order(float speed, float target, float t, float lag, float* speed_out,
                        float* dist_out) {
  if (lag <= 0.0f) {
    *speed_out = target;
    *dist_out = target * t;
    return;
  }
  float decay = expf(-t / lag);
  *speed_out = target + (speed - target) * decay;
  *dist_out = target * t + (speed - target) * lag * (1.0f - decay);
}

// Dead-reckoned orientation (no position feedback, docs/adr/0001).
class OrientationEstimate {
 public:
  OrientationEstimate() { reset(); }
  void reset() {
    theta_outer = theta_inner = speed_outer = speed_inner = 0.0f;
    stats.reset();
  }
  void advance(float target_outer, float target_inner, float dt) {
    float unused, mid_o, mid_i, step_o, step_i;
    first_order(speed_outer, target_outer, 0.5f * dt, RESPONSE_LAG_S, &unused, &mid_o);
    first_order(speed_inner, target_inner, 0.5f * dt, RESPONSE_LAG_S, &unused, &mid_i);
    first_order(speed_outer, target_outer, dt, RESPONSE_LAG_S, &speed_outer, &step_o);
    first_order(speed_inner, target_inner, dt, RESPONSE_LAG_S, &speed_inner, &step_i);
    stats.add(gravity_in_sample(theta_outer + mid_o * DEG, theta_inner + mid_i * DEG), dt);
    theta_outer = wrap_angle(theta_outer + step_o * DEG);
    theta_inner = wrap_angle(theta_inner + step_i * DEG);
  }
  float theta_outer, theta_inner;  // rad
  float speed_outer, speed_inner;  // deg/s
  GravityStats stats;
};

// --------------------------------------------------------------- strategies

enum StrategyKind { CLINOSTAT = 0, RANDOM_WALK = 1, RANDOM_SETPOINTS = 2, N_STRATEGIES = 3 };

inline const char* strategy_name(StrategyKind k) {
  switch (k) {
    case CLINOSTAT:
      return "clinostat";
    case RANDOM_WALK:
      return "random_walk";
    default:
      return "random_setpoints";
  }
}

// Motion strategy, see rpm/core/strategies.py. One tagged struct instead of
// virtual classes: no heap, and the three share their few fields.
class Strategy {
 public:
  void init(StrategyKind kind, float max_speed, uint32_t seed) {
    this->kind = kind;
    this->max_speed = max_speed;
    rng.reseed(seed);
    elapsed_ms = 0;
    for (int i = 0; i < 2; ++i) {
      next_change_ms[i] = 0;
      target[i] = current[i] = 0.0f;
      speed[i] = setpoint[i] = 0.5f;
    }
  }

  // Target axis speeds (deg/s) for this control period of dt seconds.
  void step(float dt, float* out_outer, float* out_inner) {
    unsigned long now = elapsed_ms;
    elapsed_ms = now + (unsigned long)(dt * 1000.0f + 0.5f);
    if (kind == CLINOSTAT) {
      *out_outer = max_speed;
      *out_inner = max_speed * CLINOSTAT_RATIO;
    } else if (kind == RANDOM_WALK) {
      float max_delta = WALK_ACCEL * dt;
      for (int i = 0; i < 2; ++i) {
        if (now >= next_change_ms[i]) {
          next_change_ms[i] = now + (unsigned long)rng.randint(WALK_HOLD_MIN_MS, WALK_HOLD_MAX_MS);
          float magnitude = rng.uniform(WALK_MIN_FRACTION, 1.0f) * max_speed;
          target[i] = rng.sign() * magnitude;
        }
        float delta = target[i] - current[i];
        if (delta > max_delta) delta = max_delta;
        if (delta < -max_delta) delta = -max_delta;
        current[i] += delta;
      }
      *out_outer = current[0];
      *out_inner = current[1];
    } else {
      if (now >= next_change_ms[0]) {
        next_change_ms[0] = now + SETPOINT_HOLD_MS;
        setpoint[0] = rng.random();
        setpoint[1] = rng.random();
      }
      float k = dt / SETPOINT_SMOOTHING_S;
      if (k > 1.0f) k = 1.0f;
      for (int i = 0; i < 2; ++i) speed[i] += k * (setpoint[i] - speed[i]);
      *out_outer = max_speed * drive(speed[0]);
      *out_inner = max_speed * drive(speed[1]);
    }
  }

  StrategyKind kind;
  float max_speed;
  unsigned long elapsed_ms;
  unsigned long next_change_ms[2];

  static constexpr float CLINOSTAT_RATIO = 0.3819660113f;
  static const long WALK_HOLD_MIN_MS = 2000;
  static const long WALK_HOLD_MAX_MS = 10000;
  static constexpr float WALK_MIN_FRACTION = 0.3f;
  static constexpr float WALK_ACCEL = 60.0f;
  static const unsigned long SETPOINT_HOLD_MS = 10000;
  static constexpr float SETPOINT_SMOOTHING_S = 10.0f;
  static constexpr float SETPOINT_DEAD_BAND = 0.48f;

 private:
  float drive(float s) const {
    float v = sinf(2.0f * PI_F * s);
    if (fabsf(v) < SETPOINT_DEAD_BAND) v = v >= 0.0f ? SETPOINT_DEAD_BAND : -SETPOINT_DEAD_BAND;
    return v;
  }
  Rng rng;
  float target[2], current[2];  // random walk
  float speed[2], setpoint[2];  // random setpoints
};

// Strategy + servo models + estimate, one tick per control period.
class Controller {
 public:
  // max_speed is capped at what both drives reach (rpm.core.controller.make_controller).
  void init(StrategyKind kind, float max_speed, uint32_t seed, const ServoModel& outer,
            const ServoModel& inner) {
    outer_model = &outer;
    inner_model = &inner;
    float cap = outer.max_speed() < inner.max_speed() ? outer.max_speed() : inner.max_speed();
    strategy.init(kind, max_speed < cap ? max_speed : cap, seed);
    estimate.reset();
    ticks = 0;
  }

  void tick(float dt, int* pulse_outer, int* pulse_inner) {
    strategy.step(dt, &speed_outer, &speed_inner);
    *pulse_outer = outer_model->pulse_for(speed_outer);
    *pulse_inner = inner_model->pulse_for(speed_inner);
    estimate.advance(outer_model->speed_at(*pulse_outer), inner_model->speed_at(*pulse_inner), dt);
    ++ticks;
  }

  Strategy strategy;
  OrientationEstimate estimate;
  const ServoModel* outer_model;
  const ServoModel* inner_model;
  float speed_outer, speed_inner;  // last commanded, deg/s
  unsigned long ticks;
};

}  // namespace rpm

#endif  // RPM_CORE_H
