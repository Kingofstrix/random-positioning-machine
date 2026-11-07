// gravity_estimate.h - estimate of the gravity the sample "felt", for the RPM Arduino sketch.
//
// What it does
//   The machine has no acceleration sensor. This file integrates the frame angles from the
//   speeds you command, works out where gravity points in the sample (platform) frame, and
//   keeps two numbers for the status line:
//     gres  |average gravity vector| since `start`, in g. 1.000 = no averaging yet,
//           near 0 = gravity averaged out (simulated microgravity).
//     cov   share of 200 equal-area orientation bins visited since `start`, 0..1.
//   Reference + test vectors: gravity_ref.py / vectors.csv in the same folder.
//
// How to use (see the example at the bottom)
//   GravityEstimate grav;                     // one global object, ~32 bytes RAM
//   on the `start` command:    grav.start();
//   every control tick:        grav.tick(outer_dps, inner_dps, dt_ms, running);
//       outer_dps/inner_dps = the RAMPED frame speeds in deg/s (what the servos really get),
//       dt_ms = milliseconds since the last tick, running = state is RUNNING or STOPPING.
//       Call it in every state (also STOPPED), so the angles stay correct.
//   in the status reply:       grav.gres(), grav.cov()
//
// No dynamic memory, no String, only <math.h>. On the Nano `float` and `double` are both
// 32 bit; the sums are `long` (32 bit) and exact, enough for about 59 hours of running.

#ifndef GRAVITY_ESTIMATE_H
#define GRAVITY_ESTIMATE_H

#include <math.h>
#include <stdint.h>

class GravityEstimate {
 public:
  static const uint16_t SAMPLE_MS = 100;  // one sample every 100 ms
  static const uint8_t N_BANDS = 10;      // bands in gy, equal area on the sphere
  static const uint8_t N_SECTORS = 20;    // sectors in azimuth atan2(gz, gx)
  static const uint8_t N_BINS = 200;      // N_BANDS * N_SECTORS

  GravityEstimate() : a_o(0), a_i(0) { start(); }

  // Reset the statistics (NOT the angles: the frames do not jump back to 0 on `start`).
  void start() {
    sx = sy = sz = 0;
    n = 0;
    ms_since_sample = 0;
    for (uint8_t k = 0; k < sizeof(bits); k++) bits[k] = 0;
  }

  void tick(float outer_dps, float inner_dps, uint16_t dt_ms, bool running) {
    a_o = wrap360(a_o + outer_dps * dt_ms / 1000.0f);
    a_i = wrap360(a_i + inner_dps * dt_ms / 1000.0f);
    if (!running) return;
    ms_since_sample += dt_ms;
    while (ms_since_sample >= SAMPLE_MS) {
      ms_since_sample -= SAMPLE_MS;
      sample();
    }
  }

  // |average gravity| in g; 1.000 before the first sample.
  float gres() const {
    if (n == 0) return 1.0f;
    // sx^2 would overflow a long after a few minutes, so square in float.
    float fx = sx, fy = sy, fz = sz;
    return sqrt(fx * fx + fy * fy + fz * fz) / (1000.0f * n);
  }

  // Share of the 200 bins visited, 0..1.
  float cov() const {
    uint8_t count = 0;
    for (uint8_t k = 0; k < sizeof(bits); k++) {
      uint8_t b = bits[k];
      while (b) { count += b & 1; b >>= 1; }
    }
    return count / (float)N_BINS;
  }

 private:
  float a_o, a_i;            // frame angles in degrees, kept in [0, 360)
  long sx, sy, sz;           // sums of gravity components, in units of 0.001 g
  long n;                    // number of samples
  uint16_t ms_since_sample;  // time since the last sample
  uint8_t bits[N_BINS / 8];  // 1 bit per orientation bin = 25 bytes

  static float wrap360(float a) {
    a = fmod(a, 360.0f);
    if (a < 0) a += 360.0f;
    return a;
  }

  static long roundHalfAway(float x) { return (long)(x >= 0 ? x + 0.5f : x - 0.5f); }

  void sample() {
    const float DEG = M_PI / 180.0f;
    float so = sin(a_o * DEG), co = cos(a_o * DEG);
    float si = sin(a_i * DEG), ci = cos(a_i * DEG);
    // Gravity (0,-1,0) seen from the platform: g = R^T * g_world with R = Rx(a_o) * Ry(a_i).
    long ix = roundHalfAway(-so * si * 1000.0f);
    long iy = roundHalfAway(-co * 1000.0f);
    long iz = roundHalfAway(so * ci * 1000.0f);
    sx += ix; sy += iy; sz += iz; n++;

    // Bin from the rounded integers (no -0.0 trouble, same result as gravity_ref.py).
    long band = (iy + 1000) * N_BANDS / 2000;  // floor((gy + 1) / 2 * 10)
    if (band > N_BANDS - 1) band = N_BANDS - 1;
    float phi = atan2((float)iz, (float)ix);   // -pi..pi
    long sector = (long)((phi + M_PI) / (2 * M_PI) * N_SECTORS);
    if (sector > N_SECTORS - 1) sector = N_SECTORS - 1;
    uint8_t b = band * N_SECTORS + sector;
    bits[b >> 3] |= (uint8_t)(1 << (b & 7));
  }
};

// ---------------------------------------------------------------------------------------
// Example: append the two fields to the status reply.
// Arduino's printf/sprintf has no %f, so print floats with Serial.print(x, 3)
// (or dtostrf(x, 1, 3, buf) if you build the line in a char buffer).
//
//   GravityEstimate grav;
//
//   void printStatus() {
//     Serial.print(F("ok state="));  Serial.print(stateName());
//     // ... mode, inner_dps, outer_dps, inner_us, outer_us, t as before ...
//     Serial.print(F(" gres="));     Serial.print(grav.gres(), 3);
//     Serial.print(F(" cov="));      Serial.print(grav.cov(), 3);
//     Serial.print('\n');
//   }
//
//   // in loop(), once per control tick (e.g. every 20 ms):
//   grav.tick(outer_dps_ramped, inner_dps_ramped, dt_ms,
//             state == RUNNING || state == STOPPING);
//
//   // in the `start` command handler:
//   grav.start();
// ---------------------------------------------------------------------------------------

#endif  // GRAVITY_ESTIMATE_H
