# PLAN – RPM Firmware + MuJoCo-Simulation

Stand: 07.11.2026. Grundlage: `rpm_printables/` (STLs + Aufbauanleitung), Vorlesungsfolien
„Disciplined Vibe Coding“, Original-Repo `CoreElectronics/CE-Random-Positioning-Machine`,
FS90R-Datenblatt (Feetech, via Pololu).

Markierungen: **[gemessen]** = aus STL/Datenblatt bestimmt · **[Annahme]** = Schätzung, auf der
echten Maschine kalibrieren · **[prüfen]** = unsicher, im Viewer/am Gerät verifizieren.

**Anpassungen an die Repo-Regeln (`CONTRIBUTING.md`), beschlossen 07.11.2026:**
- Der gesamte Code liegt unter `simulation/` (Ordner des Simulation-Teams), nicht im Repo-Root.
  Alle Pfade in diesem Plan (`firmware/`, `sim/`, `tests/`, `tools/`) sind relativ zu `simulation/`.
- Die STLs werden **nicht** kopiert, die Sim lädt sie direkt aus `rpm_printables/`.
- Jeder Meilenstein geht über Branch + PR (Squash-Merge). Große Meilensteine werden auf mehrere
  PRs unter ~400 Zeilen verteilt (M1: prng + servo_map + AST-Test / motion + controller / commands).
- `mujoco>=3.2,<4`: 4.0.0 gibt es auf PyPI nur als Quellpaket, 3.9 hat Wheels bis Python 3.14.

---

## 1. Ziel

1. **Steuerlogik** für den Pico: zwei Endlos-Servos so fahren, dass der zeitlich gemittelte
   Schwerkraftvektor im Probenkoordinatensystem gegen 0 geht. Dazu mehrere Fahrmodi,
   sanftes Anfahren/Stoppen, ein serielles Befehlsinterface und ein Kalibriermodus.
2. **Physik-Simulation in MuJoCo** mit den echten STL-Teilen, einem Servo-/DC-Motor-Modell,
   Getriebe 4:1, Reibung und Sensoren an der Probe. Die Simulation führt **denselben**
   Controller-Code aus wie der Pico.
3. **Auswertung**: Rest-Schwerkraft über Zeit, Richtungsabdeckung auf der Kugel,
   Zusatzbeschleunigungen an der Probe, Soll/Ist-Drehzahl.
4. **Tests** für Logik, Modell und geschlossenen Regelkreis.

Nicht im Scope (erstmal): IMU auf der Plattform und geschlossene Regelung, Web-UI, eigene
CAD-Änderungen.

---

## 2. Hardware-Fakten

### 2.1 Kinematik [gemessen aus den STLs]

- Alle STLs außer `Stand.stl` liegen im **gemeinsamen Baugruppen-Koordinatensystem** (mm).
  Lagerbohrungen und Wellenzapfen fluchten ohne Verschiebung.
- **Oben = +Y** (STL). Boden = Unterkante der Standfüße bei **y = −91,19 mm**.
- **Achse outer**: Gerade entlang **X** durch **(0, 0, 2,5) mm**. Der Außenrahmen steht fest, der
  Innenrahmen dreht darin. Rechts (+X) steckt ein Zapfen Ø 5 mm des Innenrahmens
  (x 55,2…80,8) in einer Sacklochbohrung Ø 5,3 des Außenrahmens; links (−X) ist eine
  Durchgangsbohrung für Schleifring und großes Zahnrad.
- **Achse inner**: Gerade entlang **Y** durch **(0, 0, 2,5) mm** im Innenrahmen. Die Plattform
  hat eine Welle Ø 5 mm, die Bohrungen im Innenrahmen haben Ø 6 mm. Die Welle ist auf der Achse
  massiv bei y −63,5…−22,2 und 22,2…66,8 (in der Mitte offener Rahmen).
- **Beide Achsen schneiden sich** in (0, 0, 2,5) mm. Bei Winkel outer = inner = 0 steht die
  inner-Achse **senkrecht**. Das ist wichtig für die Tests (siehe 8.3).
- Rahmengrößen: Außenrahmen 169 × 180 × 24 mm, Innenrahmen 157 × 120 × 21 mm,
  Plattform 62 × 130,5 × 7 mm.

### 2.2 Getriebe [gemessen]

| Teil | Zähne | Kopfkreis-Ø | Rolle |
|---|---|---|---|
| `04_spur_gear_large_outer` | 60 | 63,0 mm | sitzt am Innenrahmen (−X-Seite) |
| `05_spur_gear_large_inner` | 60 | 63,0 mm | sitzt auf der Plattformwelle (+Y-Seite) [prüfen] |
| `06_spur_gear_small` (2×) | 15 | 17,3 mm | Ritzel auf den Servos |

→ Modul ≈ 1, **Übersetzung 4:1 auf beiden Achsen**, Achsabstand ≈ 38 mm.
Rahmen-Drehzahl = Servo-Drehzahl / 4, Moment am Rahmen = 4 × Servo-Moment.

### 2.3 Servo-Positionen [gemessen, Zuordnung prüfen]

Servo-Schraublöcher Ø 2 mm im Abstand 27,5 mm (passt zur 9-g-Servo-Bauform):
- **outer-Servo** am linken Außenrahmen-Holm (x ≈ −84…−79), Löcher bei (y, z) = (27,7; 14,5)
  und (55,5; 14,5), Schrauben entlang X.
- **inner-Servo** am Innenrahmen auf der +Y-Seite (y ≈ 55…59), Löcher bei (x, z) = (29,1; 14,5)
  und (56,6; 14,5), Schrauben entlang Y.
- Die genaue Lage der großen Zahnräder **entlang** ihrer Achse ist aus den Daten nicht eindeutig
  → im Viewer gegen Anleitung/Foto prüfen. Für die Dynamik ist das egal (nur wenige Gramm).

### 2.4 Teile, Volumen, Massen

| Datei | Volumen [gemessen] | Masse bei 60 % Füllgrad PLA [Annahme] | Körper in MuJoCo |
|---|---|---|---|
| 01_outer_frame | 57,2 cm³ | (fest, egal) | world |
| 02_inner_frame | 25,5 cm³ | ≈ 19 g | inner_frame |
| 03_experiment_platform | 14,2 cm³ | ≈ 11 g | platform |
| 04_spur_gear_large_outer | 4,8 cm³ | ≈ 3,6 g | inner_frame |
| 05_spur_gear_large_inner | 4,1 cm³ | ≈ 3,1 g | platform |
| 06_spur_gear_small | 0,8 cm³ | ≈ 0,6 g | Ritzel-Körper |
| 07_end_cap | 0,1 cm³ | ≈ 0,1 g | platform |
| Stand (2×) | 7,8 cm³ | (fest) | world |
| FS90R (2×) | – | 9 g, Box 22,5 × 12,1 × 27,7 mm | inner-Servo am inner_frame, outer-Servo an world |

Alle STLs sind wasserdicht, MuJoCo kann Trägheiten daraus berechnen. Dichte als Parameter:
`1240 kg/m³ × fill_factor`.

`Stand.stl` liegt **nicht** im Baugruppensystem: y/z passen (Löcher bei y = −49,8 und −74,8 wie
die neuen Eckbohrungen im Außenrahmen), aber x = 0…3 muss an die Holme verschoben werden:
rechts x = +84,61…+87,61, links gespiegelt x = −87,61…−84,61.

### 2.5 FS90R-Servo [Datenblatt]

| Parameter | Wert |
|---|---|
| Neutral (Stopp) | 1500 µs, per Trimmpoti an der Unterseite justierbar |
| Totband | 90 µs (also ±45 µs um Neutral) |
| Pulsbereich | 700…2300 µs, > 1500 = CCW, < 1500 = CW |
| Leerlaufdrehzahl | 80 RPM @ 4,8 V, 100 RPM @ 6 V (Pololu-Shopseite sagt 100/130, Datenblatt hat Vorrang) |
| Haltemoment | 1,3 kg·cm @ 4,8 V, 1,5 kg·cm @ 6 V |
| Strom | 110 mA Leerlauf, 700 mA blockiert (4,8 V) |
| PWM | 50 Hz |

Für 5 V (Pico VBUS) interpoliert: **≈ 83 RPM, ≈ 0,13 N·m** [Annahme].
→ Rahmen max. ≈ 21 RPM ≈ 125 °/s, Moment am Rahmen max. ≈ 0,5 N·m.

**Kennlinie Puls → Drehzahl [Annahme]:** innerhalb ±45 µs Stillstand, danach ungefähr linear bis
zur Sättigung bei ca. ±350 µs. Der Original-Code-Kommentar bestätigt, dass sich das meiste im
unteren Bereich abspielt. **Muss am Gerät gemessen werden (Meilenstein M7).**

### 2.6 Elektrik [Aufbauanleitung]

| Signal | Pico-Pin | Kabel |
|---|---|---|
| outer-Servo Signal | **GP15** (Pin 20) | Orange |
| inner-Servo Signal (über Schleifring) | **GP16** (Pin 21) | Orange → Gelb |
| 5 V | VBUS (Pin 40) | Rot |
| GND | GND (Pin 38) | Braun → Schwarz |

GP15 = PWM-Slice 7B, GP16 = Slice 0A → unabhängig, beide auf 50 Hz.
Zwei blockierte Servos ziehen bis zu 1,6 A über VBUS. Das kann USB-Ports überlasten, deshalb
in der Doku auf ein ausreichendes Netzteil hinweisen.

### 2.7 Probleme im Original-Code (nicht übernehmen, nur Referenz)

1. **Pins vertauscht** gegenüber der Anleitung (Code: outer = 16, inner = 15).
2. `random()` liefert 0…1, der Kommentar sagt −1…1. Die Abbildung über `sin(2π·s)` ist nicht
   monoton, Drehzahl und Richtung hängen also willkürlich vom Setpoint ab.
3. Die Servo-Library mappt 0…180° auf 544…2400 µs, d. h. 90° = 1472 µs statt 1500.
4. `frequency` wird nie benutzt, kein Seed, keine Rampe beim Stoppen, keine Kalibrierung.

---

## 3. Architektur-Entscheidungen

| # | Entscheidung | Begründung |
|---|---|---|
| A1 | `firmware/rpmcore/` = reine Logik, MicroPython-kompatibel, von der Sim importiert | Eine Quelle der Wahrheit: getestet = geflasht |
| A2 | Eigener PRNG (xorshift32) statt `random` | Gleiche Zufallsfolge auf Pico, CPython und in Tests |
| A3 | Setpoints in **Rahmen-°/s**, Umrechnung in µs über eine Kalibriertabelle (stückweise linear, getrennt für ± Richtung) | Physikalisch lesbar, asymmetrische Servos abbildbar, echte Messpunkte eintragbar |
| A4 | Control-Tick 20 ms (50 Hz) | Passt zur Servo-PWM, reicht für die langsamen Rahmen |
| A5 | MuJoCo mit **echten STL-Meshes** direkt aus `rpm_printables/`, `scale = 0.001`, **Kollisionen aus** (`contype=conaffinity=0`) | Maschine ist nur über Gelenke gekoppelt, Kontakte kosten nur Zeit; keine Kopie der Binärdateien |
| A6 | Servo als MuJoCo-`velocity`-Aktuator am Rahmengelenk mit `gear=4`: `kv = τ_stall/ω0`, `ctrl = u·ω0`, `forcerange = ±τ_stall` | Ergibt genau die DC-Motor-Kennlinie τ = τ_stall·(u − ω/ω0). Die Motordämpfung am Rahmen (≈ 0,24 N·m·s/rad) gegen die kleine Rahmenträgheit ergibt ohne Armature eine Zeitkonstante von ≈ 1 ms, explizit mit 1-ms-Schritt also **instabil**. `implicitfast` behandelt den Geschwindigkeitsterm implizit und bleibt stabil, egal wie die Armature-Schätzung ausfällt |
| A7 | Puls → Ansteuerung `u ∈ [−1, 1]` in `sim/servo_model.py` (Totband, Sättigung, Neutral-Offset) | Hier sitzt die „echte“ Servo-Kennlinie der Sim; die Firmware hat ihre eigene Kalibrierung, beide dürfen abweichen (realistisch) |
| A8 | Reflektierte Rotorträgheit als `armature` am Gelenk | Bei Getriebemotoren dominiert sie die Rahmenträgheit [Annahme: 8e-4 kg·m² am Rahmen] |
| A9 | Ritzel als eigene Körper, an die Rahmengelenke gekoppelt per `<equality><joint polycoef="0 -4 0 0 0">` | Zahnräder drehen sich im Viewer sichtbar richtig; physikalisch vernachlässigbar |
| A10 | Sensoren am Site `sample`: `accelerometer`, `gyro`, `framequat`; dazu `jointpos`/`jointvel`/`actuatorfrc` | Der Beschleunigungssensor zeigt, was eine IMU auf der Plattform messen würde |
| A11 | Gravitationsmetriken aus `R_sampleᵀ · g_world` (exakt); Accelerometer-Mittel zusätzlich | Trennt reine Schwerkraftmittelung von Zentrifugaleffekten |

---

## 4. Repo-Struktur

```
random-positioning-machine/
├── CLAUDE.md
├── CONTRIBUTING.md
├── docs/
│   ├── PLAN.md
│   └── interfaces.md            # Pins, Einheiten, Koordinatensysteme (Team-Schnittstellen)
├── rpm_printables/              # STLs + Aufbauanleitung + Lizenz (nicht ändern, nicht kopieren)
└── simulation/
    ├── README.md                # Aufbau, Flashen, Sim starten, Kalibrieren
    ├── pyproject.toml           # deps: mujoco>=3.2,<4, numpy, matplotlib; dev: pytest; opt: imageio[ffmpeg]
    ├── firmware/                # Inhalt kommt 1:1 auf den Pico
    │   ├── main.py              # Hardware-Adapter: PWM, USB-Seriell, Loop, LED
    │   └── rpmcore/
    │       ├── __init__.py
    │       ├── config.py        # Pins, Grenzen, Kalibriertabelle, Modus-Parameter
    │       ├── prng.py          # XorShift32
    │       ├── servo_map.py     # Rahmen-°/s ↔ Puls-µs
    │       ├── motion.py        # SlewLimiter + Profile (random, clino3d, clino2d)
    │       ├── controller.py    # Zustandsautomat, tick(dt) → (inner_us, outer_us)
    │       └── commands.py      # Parser für serielle Befehle
    ├── sim/
    │   ├── __init__.py          # fügt firmware/ zu sys.path hinzu
    │   ├── params.py            # MachineParams (dataclass): Massen, Reibung, Servo, Payload, dt
    │   ├── geometry.py          # Konstanten aus Abschnitt 2 (Achsen, Offsets, Teileliste, STL-Pfad)
    │   ├── mjcf.py              # build_mjcf(params) → XML-String
    │   ├── servo_model.py       # FS90R: pulse_us → u; Referenz-Formel torque(u, ω)
    │   ├── machine.py           # SimMachine: Modell + Daten + Servos, step(inner_us, outer_us)
    │   ├── run.py               # simulate(...) → SimLog; CLI
    │   ├── metrics.py           # Rest-g, Fenster-Rest-g, Kugelabdeckung, Zusatzbeschl., Tracking
    │   ├── viewer.py            # mujoco.viewer.launch_passive + Terminal-Befehle + Pfeile
    │   └── report.py            # Plots, optional Video (mujoco.Renderer, MUJOCO_GL=egl)
    ├── tools/
    │   └── pico_selftest.py     # läuft auf dem Pico: importiert rpmcore, prüft Kernfunktionen
    └── tests/
        ├── conftest.py          # sys.path, Marker "slow", Fixtures (cfg, machine)
        ├── firmware/
        │   ├── test_prng.py
        │   ├── test_servo_map.py
        │   ├── test_motion.py
        │   ├── test_controller.py
        │   ├── test_commands.py
        │   ├── test_main_fake_hw.py
        │   └── test_micropython_compat.py
        └── sim/
            ├── test_servo_model.py
            ├── test_model.py
            ├── test_actuator_dynamics.py
            ├── test_metrics.py
            └── test_closed_loop.py
```

---

## 5. Firmware im Detail (`firmware/rpmcore`)

### 5.1 `config.py`

Ein einziges Dict `CONFIG` (keine Klassen nötig), Startwerte:

```python
CONFIG = {
    "pins": {"outer": 15, "inner": 16},
    "tick_ms": 20,
    "gear_ratio": 4.0,
    "autostart": True,
    "default_mode": "random",
    "seed": 1,
    "accel_dps2": 20.0,          # Rampe für Rahmen-Drehzahl [Annahme]
    "random": {"v_min_dps": 10.0, "v_max_dps": 60.0, "hold_min_s": 5.0, "hold_max_s": 20.0},
    "clino3d": {"outer_dps": 30.0, "inner_dps": 18.54},   # Verhältnis ≈ Goldener Schnitt
    "clino2d": {"outer_dps": 30.0},
    "servo": {
        "outer": {"neutral_us": 1500, "min_us": 700, "max_us": 2300,
                  # Kalibriertabelle: (Rahmen-°/s, Puls-Offset µs), je Richtung, aufsteigend
                  "pos": [(0.0, 45), (125.0, 350)], "neg": [(0.0, -45), (125.0, -350)]},
        "inner": {...gleich...},
    },
}
```

### 5.2 `prng.py`

`XorShift32(seed)`: `next_u32()`, `random()` ∈ [0, 1), `uniform(a, b)`, `choice_sign()` → ±1.
Seed 0 wird auf eine feste Konstante abgebildet (xorshift darf nicht 0 sein). Immer
`& 0xFFFFFFFF` maskieren.

### 5.3 `servo_map.py`

`ServoMap(cal)`:
- `pulse_for(frame_dps) -> int µs`: 0 → `neutral_us` exakt. Sonst Offset aus der Tabelle für
  die Richtung interpolieren (der erste Tabellenpunkt überspringt das Totband), **vom Neutralwert
  weg runden** (positive Richtung aufrunden, negative abrunden, damit jede Drehzahl ≠ 0 sicher
  außerhalb des Totbands landet) und auf `[min_us, max_us]` clampen.
- `max_dps() -> float`: kleinster Endwert beider Richtungen (Profile clampen darauf).
- `dps_for(pulse_us)`: Umkehrung (für Status und Tests).

### 5.4 `motion.py`

- `SlewLimiter(rate)`: `update(target, dt)`: Änderung pro Tick maximal `rate·dt`.
- Profile mit gemeinsamer Schnittstelle `targets(dt) -> (inner_dps, outer_dps)`:
  - `RandomWalkProfile(cfg, rng)`: hält Zielwerte für eine zufällige Dauer
    `uniform(hold_min, hold_max)`, danach für **jede Achse unabhängig**
    Betrag `uniform(v_min, v_max)` × zufälliges Vorzeichen. `v_min > 0` sorgt dafür, dass nie im
    Totband stehen geblieben wird. Der Halte-Timer läuft intern in int-ms.
  - `Clinostat3DProfile(cfg)`: konstante Drehzahlen (nicht ganzzahliges Verhältnis).
  - `Clinostat2DProfile(cfg)`: nur outer dreht, inner = 0.
- Registry: `PROFILES = {"random": ..., "clino3d": ..., "clino2d": ...}`.

### 5.5 `controller.py`

Zustände: `STOPPED`, `RUNNING`, `STOPPING` (Rampe auf 0, dann STOPPED), `CAL` (Rohpuls).

```python
c = Controller(CONFIG)
c.start(mode=None)        # STOPPED → RUNNING, Rampe ab aktueller Drehzahl
c.stop()                  # RUNNING → STOPPING
c.estop()                 # sofort STOPPED, Pulse = None (PWM aus)
c.set_mode(name)          # wechselt Profil, Rampe sorgt für weichen Übergang
c.set_param(key, value)   # v_min, v_max, accel, hold_min, hold_max, seed (validiert)
c.cal_pulse(axis, us)     # nur aus STOPPED/CAL, setzt Rohpuls
inner_us, outer_us = c.tick(dt)   # None = PWM aus
c.status() -> dict        # state, mode, Ist-Setpoints °/s, Pulse, Laufzeit (int-ms)
```

Invarianten (werden getestet): Pulse immer in [700, 2300] oder `None`; Setpoint-Änderung pro
Tick ≤ accel·dt; STOPPED liefert `None`; STOPPING erreicht 0 und dann STOPPED.

### 5.6 `commands.py`

Zeilenbasiert, Antwort `ok …` oder `err …`:

| Befehl | Wirkung |
|---|---|
| `help` | Befehlsliste |
| `status` | `state=… mode=… inner_dps=… outer_dps=… inner_us=… outer_us=… t=…` |
| `start [random\|clino3d\|clino2d]` | starten |
| `stop` / `estop` | weich stoppen / sofort aus |
| `mode <name>` | Modus wechseln |
| `set <key> <value>` | Parameter ändern |
| `pulse <inner\|outer> <µs>` / `pulse off` | Kalibrierung mit Rohpuls |

`parse(line) -> (cmd, args)` und `execute(controller, cmd, args) -> str`. Beides ist reine Logik.
Dieselben Funktionen nutzt `sim/viewer.py` für Terminal-Eingaben.

### 5.7 `main.py` (Pico)

- `PwmServo(pin)`: `PWM(Pin(pin))`, `freq(50)`, `write_us(us)` → `duty_ns(us*1000)`,
  `off()` → `duty_ns(0)`.
- Nicht-blockierendes Lesen von `sys.stdin` über `select.poll`, Zeilenpuffer.
- Loop: `ticks_ms`/`ticks_diff` → `dt` → `controller.tick(dt)` → Pulse schreiben → Befehle →
  `sleep_ms` bis zum nächsten Tick. Onboard-LED blinkt im Zustand RUNNING.
- `try/finally`: bei jeder Exception beide Servos `off()`.
- Logik in `def main(hw=None)` kapseln, damit `test_main_fake_hw.py` Fake-`machine`/`time`
  injizieren kann. Unten `if __name__ == "__main__": main()`.

---

## 6. Simulation im Detail (`sim/`)

### 6.1 Koordinaten und Körperbaum (`mjcf.py`)

- Welt: Z oben. Alle STL-Geoms hängen unter einem Body `base` mit
  `quat="0.7071068 0.7071068 0 0"` (+90° um X: STL-Y → Welt-Z) und `pos="0 0 0.09119"`
  (Boden bei z = 0). Mesh-`scale="0.001 0.001 0.001"`, Mesh-Dateien aus `rpm_printables/`
  (Pfad in `geometry.py`).
- Optionen: `timestep=0.001`, `integrator="implicitfast"`, `gravity="0 0 -9.81"`.
- Default: `contype=0 conaffinity=0` für alle Geoms.

```
worldbody
 └─ base (fest)                   outer_frame, 2× stand, outer_servo (Box), Schleifring-Stator
     ├─ outer_pinion  [hinge, equality −4 × outer]
     └─ inner_frame   [hinge "outer", axis 1 0 0, pos 0 0 0.0025]
         │                         inner_frame, gear_large_outer, inner_servo (Box, 9 g)
         ├─ inner_pinion [hinge, equality −4 × inner]
         └─ platform  [hinge "inner", axis 0 1 0, pos 0 0 0.0025]
                                   platform, gear_large_inner, end_cap, payload (Zylinder)
                                   site "sample" (Achsschnittpunkt + payload_offset)
```

(Achsen und Positionen in STL-Koordinaten des `base`-Frames, in m.)

### 6.2 Aktuatoren

```xml
<actuator>
  <velocity name="outer_servo" joint="outer" gear="4" kv="{tau_stall/omega0}"
            forcelimited="true" forcerange="{-tau_stall} {tau_stall}"/>
  <velocity name="inner_servo" joint="inner" gear="4" .../>
</actuator>
```

Mit `gear=4` ist die Aktuatorgeschwindigkeit = Servowellen-Drehzahl und die Kraft = Servomoment.
Pro Control-Tick (Puls ist innerhalb eines Ticks konstant): `data.ctrl[i] = servo_model.u(pulse_us) * omega0`.
Bei u = 0 bremst der Aktuator wie ein kurzgeschlossener Motor (τ = −τ_stall·ω/ω0). Das
Festhalten im Stillstand übernimmt `frictionloss` (FS90R-Getriebe ist schwer rückdrehbar).
Gelenke: `armature`, `damping`, `frictionloss` aus `params.py`.

### 6.3 `params.py` – Startwerte

| Parameter | Wert | Status |
|---|---|---|
| `gear_ratio` | 4.0 | gemessen |
| `servo_noload_rpm` | 83 | Annahme (5 V) |
| `servo_stall_torque` | 0.13 N·m | Annahme (5 V) |
| `servo_neutral_us` / `deadband_half_us` / `sat_offset_us` | 1500 / 45 / 350 | Datenblatt / Datenblatt / Annahme |
| `pla_density × fill_factor` | 1240 × 0.6 kg/m³ | Annahme |
| `armature_outer`, `armature_inner` | 8e-4 kg·m² | Annahme |
| `frictionloss_outer` (Schleifring + PLA-Lager) | 0.01 N·m | Annahme |
| `frictionloss_inner` | 0.005 N·m | Annahme |
| `damping` | 1e-4 N·m·s/rad | Annahme |
| `payload` | 50-mm-Petrischale, 15 g, Offset (0, 0, 0) m | Annahme |
| `timestep` / `control_dt` | 0.001 / 0.020 s | Entscheidung |

### 6.4 `machine.py`

```python
m = SimMachine(params)
m.step(inner_us, outer_us, duration=0.02)   # None = Servo aus (u = 0)
m.state() -> dict(t, q_outer, q_inner, w_outer, w_inner,
                  g_sample (3,), acc_sample (3,), gyro (3,), tau_outer, tau_inner)
```

### 6.5 `run.py`

`simulate(mode, minutes, seed, params, cfg, log_hz=50) -> SimLog` (numpy-Arrays + Meta).
Schleife: `controller.tick(control_dt)` → `machine.step(...)` → loggen.
CLI: `python -m sim.run --mode random --minutes 30 --seed 1 --out results/` schreibt
`results/<mode>_s<seed>.npz` + `summary.json`.

### 6.6 `metrics.py`

| Funktion | Definition |
|---|---|
| `residual_g(g_series)` | \|mittel(g_s)\| / 9,81 über den ganzen Lauf |
| `residual_g_curve(g_series)` | kumulatives Mittel über die Zeit (für den Log-Plot) |
| `windowed_residual(g_series, window_s)` | Rest-g in Fenstern (60 s, 600 s) → Median, P95 |
| `sphere_coverage(dirs, n_bands=10, n_sectors=20)` | flächengleiche Bins (gleiche Δz, gleiche Δφ): Anteil belegter Bins + Ungleichförmigkeit (std/mean) |
| `extra_accel(acc, g_s)` | \|acc + g_s\| (Accelerometer minus Schwerkraftanteil) in g, Max und RMS |
| `speed_tracking(cmd, actual)` | RMS-Fehler Soll/Ist pro Achse in °/s |

### 6.7 `viewer.py`

`mujoco.viewer.launch_passive` in Echtzeit, Controller läuft live mit. Ein Thread liest
Terminal-Zeilen → `commands.parse/execute` (gleiches Protokoll wie am Pico). Über
`viewer.user_scn` zwei Pfeile an der Probe zeichnen: aktuelle Schwerkraftrichtung und das
kumulative Mittel (skaliert, sollte schrumpfen). Ins Terminal regelmäßig Zustand und Rest-g
ausgeben.

### 6.8 `report.py`

Aus einer `.npz`: (1) Rest-g über Zeit, log-y; (2) Schwerkraftrichtungen auf der Kugel
(Hammer-Projektion, Heatmap der Bins); (3) Soll/Ist-Drehzahl beider Achsen; (4)
Zusatzbeschleunigung. Optional `--video`: Offscreen-Render 20 s mit 4× Zeitraffer als MP4.

---

## 7. Fahrmodi – was wir erwarten

| Modus | Erwartung | Zweck |
|---|---|---|
| `random` | Rest-g fällt mit der Zeit, Kugel gut abgedeckt | Hauptmodus RPM |
| `clino3d` | Rest-g fällt, aber Abdeckung mit Mustern | deterministische Referenz |
| `clino2d` | Nur die waagerechte outer-Achse dreht → g kreist in einer Ebene der Probe: Mittel → 0, aber nur ein Großkreis der Kugel abgedeckt | Vergleich 2D-Klinostat |
| nur inner dreht, outer bei 0° | inner-Achse senkrecht → **Rest-g bleibt 1 g** | Plausibilitätstest der Geometrie |

---

## 8. Tests

`pytest -m "not slow"` muss unter 30 s bleiben. Lange Läufe bekommen `@pytest.mark.slow`.

### 8.1 Firmware (rein, schnell)

- **prng**: gleicher Seed → gleiche Folge; `random()` in [0, 1); Seed 0 erlaubt; 10⁴ Werte
  haben Mittel 0,5 ± 0,02.
- **servo_map**: 0 °/s → 1500 µs; kleinste positive Drehzahl → > 1545 µs (Totband
  übersprungen); monoton steigend; Clamping auf [700, 2300]; `dps_for(pulse_for(x)) ≈ x`
  (Toleranz ±0,3 °/s wegen Rundung auf ganze µs); asymmetrische Tabellen werden respektiert.
- **motion**: SlewLimiter überschreitet nie `rate·dt`; RandomWalk-Ziele haben Betrag in
  [v_min, v_max]; beide Vorzeichen kommen vor (über 100 Wechsel); Haltezeiten in
  [hold_min, hold_max]; gleicher Seed → gleiche Ziele; clino2d liefert inner = 0.
- **controller**: Start aus STOPPED fährt rampenförmig hoch; `stop()` endet in STOPPED mit
  `None`-Pulsen; `estop()` sofort `None`; Pulse immer im gültigen Bereich (Property-Test über
  10⁴ Ticks mit zufälligen Befehlen); `cal_pulse` nur aus STOPPED; ungültige Parameter werfen
  `ValueError` und ändern nichts.
- **commands**: jede Zeile aus 5.6 wird korrekt geparst; Groß/Klein und Leerzeichen egal;
  Unsinn → `err …`, Controller unverändert.
- **main mit Fake-HW**: Fake-`machine.PWM/Pin`, Fake-Uhr, Fake-stdin mit `"stop\n"` → nach N
  Ticks wurden Pulse geschrieben, nach `stop` gehen sie auf neutral/aus, bei Exception wird
  `off()` aufgerufen.
- **MicroPython-Kompatibilität (AST)**: in `rpmcore/*.py` nur erlaubte Imports (`math`,
  `rpmcore.*`), keine f-Strings (`JoinedStr`), keine Annotationen, keine Decorators außer
  `@staticmethod`/`@property`, keine `dataclass`-/`typing`-/`enum`-Namen. Ersetzt keinen Test
  auf dem echten Pico (siehe 10.1).

### 8.2 Sim-Bausteine

- **servo_model**: u = 0 im Totband; u = ±1 ab Sättigung; Vorzeichen stimmt (>1500 → +);
  Referenzformel: τ(u=1, ω=0) = τ_stall, τ(u=1, ω=ω0) = 0.
- **model**: MJCF kompiliert; 2 Hinge-Gelenke + 2 Ritzel-Gelenke; Achse outer ⟂ Schwerkraft;
  Achse inner ⟂ Achse outer; Achsen schneiden sich (Abstand < 0,1 mm); bei q = 0 ist die
  inner-Achse parallel zur Schwerkraft; Gesamtmasse der drehenden Teile im erwarteten Bereich
  (30–80 g); Boden bei z ≈ 0.
- **actuator_dynamics**: Konstanter Puls = Sättigung, ohne Last → Rahmen erreicht
  ω0/4 ± 5 % innerhalb 0,5 s; Gelenk festgehalten (sehr hohe Reibung) → Aktuatormoment ≈
  τ_stall; Puls im Totband → keine Drehung; Ritzel dreht −4× schneller als der Rahmen.

### 8.3 Geschlossener Kreis (Controller + Physik)

- **Vorzeichen**: `set` positiver outer-Setpoint → `q_outer` steigt; dasselbe für inner.
- **Geometrie-Plausibilität**: nur inner dreht, outer = 0 → Rest-g nach 60 s > 0,95 g.
- **Determinismus**: zweimal gleicher Seed → identische Logs.
- **Tracking**: im random-Modus RMS-Fehler Soll/Ist < 10 °/s (nach Kalibrierungsabgleich
  Sim = Firmware) [Schwelle nach dem ersten Lauf festzurren].
- **slow – Mikrogravitation**: random, 30 min, Seed 1–3 → Rest-g < 0,05 g und
  Kugelabdeckung > 90 % der Bins. **Startwerte, nach dem ersten echten Lauf mit gemessenem
  Wert + Sicherheitsmarge festschreiben** (einziger Fall, in dem eine Schwelle angepasst werden
  darf, Begründung im Commit).
- **Sensorkonvention**: Payload-Offset 0 → |acc + g_s| < 0,001 g (die Probe sitzt im festen
  Achsschnittpunkt, also darf außer Schwerkraft nichts gemessen werden).
- **slow – Zusatzbeschleunigung**: Offset 10 mm, random, 10 min → max. und RMS von
  |acc + g_s| nur in `summary.json` reporten, keine Schwelle.

---

## 9. Meilensteine (eine Claude-Code-Session pro Meilenstein)

Jeder Meilenstein ist fertig, wenn seine Tests grün sind **und** `pytest -m "not slow"`
insgesamt grün ist. Danach commit, push und PR (bei Bedarf mehrere PRs, siehe oben).

| M | Inhalt | Fertig wenn |
|---|---|---|
| **M0** | Gerüst unter `simulation/`: Ordner, `pyproject.toml`, `conftest.py`, README-Skelette, venv; STL-Pfad zeigt auf `rpm_printables/` | `pytest` läuft (Gerüst-Test grün), `pip install -e ".[dev]"` klappt |
| **M1** | `rpmcore`: prng, servo_map, motion, controller, commands (3 PRs) | alle Tests aus 8.1 außer main grün, AST-Kompatibilitätstest grün |
| **M2** | `firmware/main.py` + `tools/pico_selftest.py` | Fake-HW-Test grün |
| **M3** | `sim/geometry.py`, `params.py`, `mjcf.py`: statisches Modell | Tests „model“ grün; ein Offscreen-PNG vom Modell sieht aus wie die Maschine (Teile sitzen, Zahnräder greifen) |
| **M4** | `servo_model.py`, Aktuatoren, Ritzel-Kopplung, `machine.py` | Tests „servo_model“ + „actuator_dynamics“ grün |
| **M5** | `run.py`, `metrics.py` | Tests „metrics“ + „closed_loop“ (schnelle) grün; 30-min-Lauf liefert `summary.json` |
| **M6** | `viewer.py`, `report.py`; slow-Schwellen festschreiben | Viewer läuft lokal, Report-PNGs plausibel, `pytest` komplett grün |
| **M7** | Hardware: Flashen, Pin-Check, Kalibrierung, Werte zurück in `config.py` und `params.py` | siehe 10 |

---

## 10. Hardware-Inbetriebnahme und Kalibrierung (M7)

Alle Befehle im Ordner `simulation/`.

1. **10.1 Echter MicroPython-Check**: `mpremote cp -r firmware/rpmcore :` und
   `mpremote cp firmware/main.py :`, dann `mpremote run tools/pico_selftest.py`. Der Selftest
   prüft auf dem Pico: PRNG-Folge für Seed 1 = dieselben Werte wie in CPython (in den Test
   hartkodiert), `servo_map` an 5 Stützstellen, 500 Controller-Ticks ohne Exception.
2. **Pin-Check**: `pulse outer 1600` → dreht der Außen-/Innenrahmen? Bei Bedarf `pins` in
   `config.py` tauschen.
3. **Neutral**: `pulse <axis> 1500` → steht der Servo still? Sonst Trimmpoti nachstellen oder
   `neutral_us` anpassen.
4. **Kennlinie**: für Pulse 1500 ± {50, 75, 100, 150, 200, 300, 400} µs je 10 Rahmenumdrehungen
   stoppen → °/s. In die `pos`/`neg`-Tabellen in `config.py` eintragen, `sat_offset_us`,
   `servo_noload_rpm` in `params.py` nachziehen.
5. **Reibung**: kleinsten Puls suchen, bei dem der Rahmen gerade noch anläuft → daraus
   `frictionloss` in `params.py` abschätzen.
6. Sim mit den neuen Werten neu laufen lassen; `random`-Parameter (v_min ≥ Anlaufgrenze)
   anpassen.

---

## 11. Offene Punkte / Risiken

- Kennlinie, Reibung, Rotorträgheit sind **Annahmen**, bis M7 gemessen ist. Die Sim-Ergebnisse
  davor sind qualitativ.
- Lage der großen Zahnräder entlang der Achse und genaue Servo-Box-Positionen **[prüfen]** im
  Viewer.
- Ohne Sensor ist die Steuerung offen: Der Pico kennt die echten Winkel nicht. Nächster Schritt
  wäre eine IMU auf der Plattform (Kabel über den Schleifring, der hat 6 Adern, 3 sind frei) und
  ein Modus, der gezielt die gemittelte Schwerkraft gegen 0 regelt. Die Sim hat den Sensor
  schon (`accelerometer` am `sample`).
- Stromversorgung über USB-VBUS kann bei Blockade einbrechen.
- Lizenz: STLs CC BY-NC-SA 4.0 (Core Electronics / Unconvengineering, geändert von
  M. Schutera). Attribution in `rpm_printables/README.txt` behalten, nicht kommerziell.

---

## 12. Prompts für Claude Code

Erste Session (Plan Mode, Shift+Tab):
> Lies CLAUDE.md und docs/PLAN.md. Fass in 5 Punkten zusammen, wie du Meilenstein M0 und M1
> umsetzt, und stell mir Fragen, falls etwas im Plan unklar oder widersprüchlich ist. Noch keinen
> Code.

Danach pro Meilenstein:
> Setze Meilenstein M<n> aus docs/PLAN.md um. Schreib zuerst die Tests aus Abschnitt 8, lass sie
> laufen und zeig mir, dass sie rot sind. Dann implementiere, bis `pytest -m "not slow"` grün
> ist. Halte dich an die Regeln in CLAUDE.md. Commit, push und PR am Ende.

Bei M3 zusätzlich:
> Render ein PNG des Modells von schräg vorne und eins von der Seite und schau sie dir an.
> Sitzen Zahnräder und Servos plausibel? Wenn nicht, korrigiere `geometry.py`, nicht die STLs.

Bei Bugs:
> Reproduziere den Fehler zuerst mit einem Test, dann fix, dann Test dauerhaft in die Suite.
