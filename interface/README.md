# Control page and launcher

A minimal page to run the Random Positioning Machine from a laptop: **Start**, **Stop**,
**Emergency stop**, the average gravity on the sample, an initialization curve and the machine log.
Owned by the **Testing** team.

The machine is an Arduino Nano with a CH340 USB chip. A small launcher on the laptop talks to the
Arduino over USB and opens the page in a browser window. There is no web version any more.

## Starting it

1. Plug the Arduino into the laptop.
2. Start the launcher:
   - **macOS:** double-click `start-mac.command`. If macOS blocks it the first time, right-click → Open.
   - **Windows:** double-click `start-windows.bat`.
   - **Any system:** `pip install -r requirements.txt`, then `python3 launcher.py`.
3. The page opens in its own window (Chrome or Edge in app mode, otherwise the default browser).

The first start creates `interface/.venv` and installs `pyserial`, which needs Python 3.9 or newer
and an internet connection once. After that it works offline.

The launcher finds the Arduino by the CH340's USB ID and reconnects when it is unplugged and plugged
in again. It waits 2 s after connecting because opening the port restarts the Nano.

Options: `--port /dev/cu.wchusbserial1410` (pick the port by hand), `--baud 115200`,
`--http-port 8765`, `--no-browser`.

**Windows:** if no port shows up, install the CH340 driver from the chip maker (WCH). Recent macOS
and Linux versions have it built in.

## The page

| Part | What it shows |
|---|---|
| Controls | State, run time, Start, Stop (slows down gently), Emergency stop (also the `Esc` key) |
| Average gravity | How much gravity the sample has felt on average since Start, in g. Target below 0.05 g. |
| Orientations visited | Share of 200 directions the sample has been turned through. Target 90 %. |
| Initialization | "Initializing…" with a rough forecast, then "Ready" once both targets are met |
| Initialization curve | Both values over time, with the targets as dashed lines and a marker when ready |
| Log | Every command and reply. Status replies are hidden unless you tick the box. |

**Download run (CSV)** saves the curve of the current run: `run_id, t_s, avg_gravity_g,
orientations_visited, outer_dps, inner_dps`.

Both gravity values are calculated by the Arduino from the speeds it commands. The servos give no
feedback, so this is an estimate. The targets come from `docs/PLAN.md` §8.3 and are provisional.

Without the launcher (for example when you open `index.html` straight from the disk), the page runs a
**simulated machine** with the same commands and the same gravity calculation, so you can try it
without hardware. You can speed the simulation up to 60×.

## How it talks to the machine

USB serial at 115200 baud, one text line per command. Replies start with `ok` or `err`
(see `docs/interfaces.md`).

| Command | When |
|---|---|
| `start` | Start |
| `stop` | Stop |
| `estop` | Emergency stop or `Esc` |
| `status` | Every 0.5 s while connected |

From the `status` reply the page reads `state`, `t`, `outer_dps`, `inner_dps`, `gres` and `cov`.
Unknown fields are ignored. If `gres` and `cov` are missing, the page still works and says that the
firmware does not report gravity yet. How the Arduino calculates them, with reference code and test
values: `testing/gravity-estimate/`.

Between the page and the launcher (127.0.0.1 only):

| Route | Purpose |
|---|---|
| `GET /` | the page |
| `GET /events` | Server-Sent Events: `port` (`{"connected": …, "port": …}`) and `line` (one serial line) |
| `POST /cmd` | one command line as text; 503 when no Arduino is connected |

## Developing

`index.html` is a single file with no build step and no external resources. Launcher tests:

```sh
cd interface
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt pytest
pytest tests
```
