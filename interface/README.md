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

## Not decided yet

- **Baud rate.** The page defaults to 115200 and lets experts pick another. It must match the Nano firmware.
- **Nano firmware.** The Arduino Nano script that answers these commands is not written yet.
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
