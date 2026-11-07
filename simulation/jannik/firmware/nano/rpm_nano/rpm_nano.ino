// RPM firmware for the Arduino Nano (ATmega328P, also CH340 clones).
//
// Runs the motion logic of rpm_core.h (a port of rpm/core, checked against
// the Python simulation) on two FS90R continuous servos and speaks the line
// protocol of machine.h over USB serial (115200 baud). Board settings and
// flashing: simulation/jannik/README.md, "Run on the machine".

#include <Servo.h>

#include "config.h"
#include "machine.h"

rpm::ServoModel outer_model;
rpm::ServoModel inner_model;
rpm::Machine machine;
Servo outer_servo;
Servo inner_servo;

char line[48];
uint8_t line_len = 0;
bool line_overflow = false;
char reply[rpm::Machine::REPLY_SIZE];
unsigned long next_tick_ms;
unsigned long last_status_ms;

// Attach/detach so that "off" really means no pulses: an FS90R without pulses
// stops and goes limp, and nothing can drive it by accident.
void apply(const rpm::ServoCommand& cmd) {
  if (!cmd.on) {
    if (outer_servo.attached()) outer_servo.detach();
    if (inner_servo.attached()) inner_servo.detach();
    return;
  }
  if (!outer_servo.attached()) outer_servo.attach(OUTER_PIN, rpm::MIN_PULSE_US, rpm::MAX_PULSE_US);
  if (!inner_servo.attached()) inner_servo.attach(INNER_PIN, rpm::MIN_PULSE_US, rpm::MAX_PULSE_US);
  // writeMicroseconds clamps to the attach() limits as well: 700..2300 us.
  outer_servo.writeMicroseconds(cmd.outer_us);
  inner_servo.writeMicroseconds(cmd.inner_us);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  bool ok = outer_model.set(CAL_OUTER_US, CAL_OUTER_DPS, CAL_OUTER_N) &&
            inner_model.set(CAL_INNER_US, CAL_INNER_DPS, CAL_INNER_N);
  if (!ok) {  // a broken calibration.h must not keep the machine from running
    outer_model.set_nominal();
    inner_model.set_nominal();
    Serial.println(F("calibration: invalid, using PLAN 2.5 assumptions"));
  } else {
    Serial.print(F("calibration: "));
    Serial.println(F(RPM_CALIBRATION));
  }
  machine.init(outer_model, inner_model, DEFAULT_MODE, MAX_SPEED, SEED);
  if (AUTOSTART) machine.start();
  Serial.println(F("rpm ready, type help"));
  next_tick_ms = millis();
  last_status_ms = next_tick_ms;
}

void read_commands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line_overflow) {
        Serial.println(F("err line too long"));
      } else if (line_len > 0) {
        line[line_len] = '\0';
        machine.execute(line, reply);
        Serial.println(reply);
      }
      line_len = 0;
      line_overflow = false;
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = c;
    } else {
      line_overflow = true;
    }
  }
}

void loop() {
  read_commands();
  unsigned long now = millis();
  // Fixed rate: unsigned difference stays right across the millis() wrap (49 days).
  if ((long)(now - next_tick_ms) < 0) return;
  next_tick_ms += CONTROL_PERIOD_MS;
  apply(machine.tick(CONTROL_PERIOD_MS / 1000.0f));
  bool running = machine.state == rpm::RUNNING;
  digitalWrite(LED_BUILTIN, running && (now / 500) % 2 ? HIGH : LOW);  // blinks while running
  if (running && STATUS_EVERY_MS && now - last_status_ms >= STATUS_EVERY_MS) {
    last_status_ms = now;
    machine.execute("status", reply);
    Serial.println(reply);
  }
}
