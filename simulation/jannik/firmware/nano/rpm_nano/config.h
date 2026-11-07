// Machine configuration of the Nano firmware. Edit here, not in rpm_nano.ino.
//
// Pins are a team interface (docs/interfaces.md, "Wiring"). D9 and D10 are the
// two outputs of Timer1, which the Servo library drives anyway.

#ifndef RPM_CONFIG_H
#define RPM_CONFIG_H

#include "rpm_core.h"

const int OUTER_PIN = 9;   // outer servo, on the fixed outer frame: turns the inner frame
const int INNER_PIN = 10;  // inner servo, on the inner frame (through the slip ring): turns the platform

const rpm::StrategyKind DEFAULT_MODE = rpm::CLINOSTAT;  // see docs/adr/0002
const bool AUTOSTART = true;   // start DEFAULT_MODE on power-up, without a PC
const float MAX_SPEED = 120.0f;  // axis deg/s, capped at what the servo curves reach
const uint32_t SEED = 1;       // same seed + same mode = same motion as in the simulation

const unsigned long CONTROL_PERIOD_MS = 100;
const unsigned long STATUS_EVERY_MS = 10000;  // print status this often while running (0 = never)
const long SERIAL_BAUD = 115200;

// Servo curves (pulse_us, axis_deg_s), ascending pulse, signed by the right-hand
// rule. tools/calibrate_nano.py measures them and writes calibration.h, which
// replaces these PLAN 2.5 assumptions (83 rpm at 5 V, linear from +-45 to +-350 us).
#if __has_include("calibration.h")
#include "calibration.h"
#else
const int CAL_OUTER_N = 4;
const int16_t CAL_OUTER_US[] = {1150, 1455, 1545, 1850};
const float CAL_OUTER_DPS[] = {-124.5f, 0.0f, 0.0f, 124.5f};
const int CAL_INNER_N = 4;
const int16_t CAL_INNER_US[] = {1150, 1455, 1545, 1850};
const float CAL_INNER_DPS[] = {-124.5f, 0.0f, 0.0f, 124.5f};
#define RPM_CALIBRATION "PLAN 2.5 assumptions (not measured)"
#endif

#endif  // RPM_CONFIG_H
