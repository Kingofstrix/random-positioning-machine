# Random Positioning Machine – Firmware + MuJoCo-Simulation

Zweiachsige Random Positioning Machine (RPM) für simulierte Mikrogravitation (DHBW Ravensburg,
Kurs „Disciplined Vibe Coding“). Ein Raspberry Pi Pico steuert zwei FS90R-Endlos-Servos. Die
Steuerlogik läuft identisch auf dem Pico (MicroPython) und in einer MuJoCo-Simulation (CPython).

**Der vollständige Plan steht in `docs/PLAN.md`. Lies ihn vor jeder größeren Änderung.** Arbeite
die Meilensteine dort der Reihe nach ab, einen pro Session. Repo-Regeln (Branches, PRs, Ordner der
Teams) stehen in `CONTRIBUTING.md`.

## Befehle

Alle Befehle laufen im Ordner `simulation/` mit aktivem venv (`simulation/.venv`).

```bash
python3 -m venv .venv && . .venv/bin/activate   # einmalig (fish: . .venv/bin/activate.fish)
pip install -e ".[dev]"            # mujoco, numpy, matplotlib, pytest
pytest -m "not slow"               # schnelle Tests (< 30 s), nach jeder Änderung
pytest                             # alles inkl. lange Sim-Läufe
python -m sim.run --mode random --minutes 30 --seed 1 --out results/
python -m sim.viewer --mode random # interaktiver MuJoCo-Viewer, Befehle im Terminal tippen
python -m sim.report results/      # Plots (+ optional Video)
MUJOCO_GL=egl python -m sim.report results/ --video   # Offscreen-Render ohne Bildschirm (Linux/Container)
mpremote cp -r firmware/rpmcore : && mpremote cp firmware/main.py :   # auf den Pico
```

## Architektur in einem Satz

`simulation/firmware/rpmcore/` ist reine Logik ohne Hardware-Imports und die einzige Quelle der
Wahrheit: `simulation/firmware/main.py` verbindet sie mit PWM/USB auf dem Pico,
`simulation/sim/` verbindet dieselbe Logik mit MuJoCo.

## Harte Regeln

1. **`firmware/rpmcore/` muss unter MicroPython laufen (RP2040).**
   - Erlaubte Imports: nur `math` und Module aus `rpmcore` selbst. Kein `random` (eigener
     PRNG in `prng.py`), kein `time`, kein `machine`.
   - Gibt es in MicroPython nicht: `dataclasses`, `typing`, `enum`, `abc`.
   - Zur Sicherheit vermeiden: Type-Hints und f-Strings. Strings mit `%` formatieren.
   - Floats sind auf dem Pico 32 bit, also keine Zeit in Float aufsummieren (Zeit kommt als
     `dt` in Sekunden pro Tick herein, Timer und absolute Zeit intern als int-ms).
   - `tests/firmware/test_micropython_compat.py` prüft das per AST und muss grün bleiben.
2. **Hardware nur in `firmware/main.py`.** Alles dort Testbare kommt nach `rpmcore`.
3. **Die Simulation importiert die Firmware-Logik, sie kopiert sie nicht.** Kein zweiter Controller
   in `sim/`.
4. **Tests zuerst.** Schreib für jede Funktion zuerst den Test, sieh ihn rot, dann implementiere.
   Wenn ein Test fehlschlägt: den Code reparieren, nie den Test aufweichen. Schwellwerte nur
   ändern, wenn PLAN.md das ausdrücklich erlaubt, und den Grund im Commit nennen.
5. **STL-Dateien in `rpm_printables/` nicht verändern und nicht kopieren.** Die Sim lädt sie direkt
   von dort (Lizenz CC BY-NC-SA 4.0, Attribution in `rpm_printables/README.txt` beibehalten).
6. Neue Parameter kommen in `rpmcore/config.py` (Firmware) bzw. `sim/params.py` (Physik), nie als
   Magic Numbers in den Code.
7. **Nie direkt auf `main`.** Branch `sim/<type>/<beschreibung>`, PR mit ausgefülltem Template,
   Squash-Merge. PRs möglichst unter ~400 Zeilen, größere Meilensteine auf mehrere PRs aufteilen.
8. Pins, Einheiten und Koordinatensysteme sind Team-Schnittstellen und stehen in
   `docs/interfaces.md`. Änderungen dort brauchen die Zustimmung der betroffenen Teams.

## Einheiten und Konventionen

| Größe | Einheit | Wo |
|---|---|---|
| Rahmen-Drehzahl (Setpoints, Profile) | °/s | Firmware |
| Servo-Pulsbreite | µs (int) | Firmware → PWM |
| Zeit pro Tick | s (float `dt`) | Firmware |
| Alles in MuJoCo | SI (m, kg, s, rad, N·m) | `sim/` |
| STL-Koordinaten | mm, +Y = oben | `rpm_printables/` |

- Achse **outer** = Innenrahmen dreht im feststehenden Außenrahmen (waagerecht, STL-X).
- Achse **inner** = Plattform dreht im Innenrahmen (STL-Y, steht bei 0°/0° senkrecht).
- Positive Drehzahl = positiver Drehsinn um die jeweilige Achse (Rechte-Hand-Regel).
- Pins: outer-Servo GP15, inner-Servo (über Schleifring) GP16, wie in der Aufbauanleitung.
  **Achtung: der Original-Code von Core Electronics hat das vertauscht.**

## Stil

- Python 3.11+ in `sim/` und `tests/` (Type-Hints und dataclasses dort ok).
- Kurze Module, ein Thema pro Datei. Docstrings erklären das *Warum*, nicht das *Was*.
- Commits klein, ein Meilenstein-Schritt pro Commit, Message auf Englisch im Format
  `<type>(<scope>): <what changed>` (siehe `CONTRIBUTING.md`).
