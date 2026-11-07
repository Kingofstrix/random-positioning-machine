# Web interface

One-page control hub for the Random Positioning Machine. Experimenters start and stop a run, note
their sample and export the results to Excel. Owned by the **Testing** team.

Live page (after the first deploy): https://kingofstrix.github.io/random-positioning-machine/

## Browsers

| Browser | Works |
|---|---|
| **Chrome or Edge on a desktop or laptop** (version 114 or newer) | Everything, including connecting the machine. **Use one of these.** |
| Firefox, Safari | Page, practice mode and export work. Connecting a machine does **not** work (no Web Serial). |
| Phones and tablets | Page and export work. Connecting a machine does not work. |

Connecting needs the [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API),
which only Chromium browsers on desktop support. It also needs a secure page: the GitHub Pages
address (https) or `localhost`. Opening `index.html` straight from the disk does not allow it.

## Using it

1. Plug the machine into the computer with USB and open the live page in Chrome or Edge.
2. Press **Connect machine** and pick the machine's port in the browser's list.
3. Type the sample name (or keep the prefilled ID), press **Start turning**.
4. Press **Stop gently** when done. **Emergency stop** (or the Esc key) switches the motors off at once.
5. **Export for Excel**: *Copy table* and paste into Excel, or *Download file* (CSV). Choose German
   or English Excel so separators and decimal commas come out right.

Without a machine the page runs in **practice mode**: the buttons work and runs are recorded as
"practice". If connecting fails, the page offers a **simulated machine** for demos.

Help for newcomers sits behind the **?** buttons and on the tab **The machine**.

### Expert mode

The switch in the top bar adds, on the same page:

- live values from the machine (state, ring speeds, machine time, log size),
- the machine messages (every command and reply), a command field and the baud rate,
- a **Mode** column in the results (practice, simulated, machine),
- a **machine log** export: one row per second with the ring speeds and motor signals the machine
  was told to run. These are commanded values, not measurements (the servos give no feedback and
  there are no sensors yet).

## Data

- Results are saved in the browser's local storage on this computer only. Clearing browser data
  deletes them, and another computer does not see them. Export after each experiment.
- The machine log is kept in memory for the latest run and is lost when the page closes.
- CSV files are UTF-8 with BOM. German: `;` and decimal comma. English: `,` and decimal point.

Every run gets an ID `RPM-YYMMDD-NN` (NN counts the runs of that day). The sample name is prefilled
with the ID and falls back to it when left empty, so no run is ever without a name.

Results columns: `Date, ID, Start, End, Duration (minutes), How it ended, Sample, Notes` (+ `Mode` in
expert mode). Machine log columns: `Date, Time, Seconds since start, State, Outer ring speed (°/s,
commanded), Inner ring speed (°/s, commanded), Outer motor signal (µs), Inner motor signal (µs), ID, Sample`.

## How it talks to the machine

USB serial, one text line per command, replies start with `ok` or `err` (see `docs/interfaces.md`).
The page uses only these commands from the simulation team's `rpmcore/commands.py`:

| Command | When |
|---|---|
| `start` | Start turning |
| `stop` | Stop gently |
| `estop` | Emergency stop |
| `status` | Every 0.5 s while connected |

From the `status` reply it reads `state` (`RUNNING`, `STOPPING`, `STOPPED`), `inner_dps`,
`outer_dps`, `inner_us`, `outer_us` and `t` (seconds). If the firmware changes this format, the
page has to change too.

## Forward to the simulation (hardware in the loop)

Expert mode → *Machine messages* → **Forward to simulation** sends a copy of every line the page
sends to the machine and every line it receives to `python -m sim.hil` on the same computer
(default `ws://localhost:8765`). The page keeps the serial port. Only a real machine is
forwarded, not the preview.

`sim.hil` then runs the 3D simulation with the machine's real motor signals and checks the Nano
firmware against the simulation team's logic (rpmcore). At the end it writes a report for the
simulation team. Details and how to flash the needed **telemetry build**:
`simulation/arduino/README.md` (on the branch `sim/feat/arduino-nano-firmware` until merged).

1. Flash the telemetry build onto the Nano.
2. On the laptop: `cd simulation && python -m sim.hil`.
3. On the page: expert mode on, tick *Forward to simulation* (the status says *connected to the
   simulation*), then **Connect machine**. Connecting resets the Nano, so the check starts at its boot.
4. Use the page as usual. End with Ctrl-C in the terminal: the report is in `simulation/results/`.

The telemetry build also sends a line per tick (`T …`) and per command (`C …`). The page forwards
them but does not show them. If the Nano runs the normal firmware while forwarding is on, the
page says so in the machine messages.

If the browser blocks the connection from the GitHub Pages address to `localhost` (some Chrome
versions ask for permission to access devices on the local network: allow it), open the page
locally instead: `cd interface && python3 -m http.server 8000`, then http://localhost:8000.

## Not decided yet

- **Baud rate.** The page defaults to 115200, the Nano firmware uses 115200.
- **Nano firmware.** Written (`simulation/arduino/`, branch `sim/feat/arduino-nano-firmware`), not merged yet.
- **Moon and Mars gravity.** Shown as "later"; needs a motion strategy from the simulation team.
- **Measurements.** No sensors yet. Columns will be added once the teams decide what to measure.
- **Build details** on the tab *The machine* stay "Not decided yet" until the hardware team fills them in.

## Developing

It is a single static file, `index.html`, with no build step and no dependencies apart from Google Fonts.

```sh
cd interface
python3 -m http.server 8000   # then open http://localhost:8000 in Chrome
```

For a demo without hardware, set `PREVIEW_SIMULATED_MACHINE = true` near the top of the script.
Keep it `false` in the committed file.

Pushing to `main` with changes under `interface/` deploys the page through
`.github/workflows/pages.yml`.

## Licence

The machine drawing is based on the Random Positioning Machine by Core Electronics and
Unconvengineering (CC BY-NC-SA 4.0), modified by Mark Schutera, DHBW Ravensburg. See
`rpm_printables/README.txt`.
