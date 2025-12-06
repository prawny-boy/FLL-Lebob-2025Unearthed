# Unearthed 2025 Robotics Codebase

PrimeHub-ready mission software plus desktop tooling for the LEBOB FIRST LEGO League *SUBMERGED / UNEARTHED 2025* season.

The tournament program is kept in a **single hub-friendly file** (`src/main.py`) so it can be flashed to the robot quickly, while desktop simulators help us tune PID profiles, plan obstacle routes, and document field experiments.

## Highlights

- **Mission menu on-hub:** Slot-based decorator system (for example `@mission("1")`) keeps attachments organized and allows rapid iteration without touching the firmware scaffolding.
- **Safety-first drive helpers:** Smart PID drive/turn routines, torque caps, and a low-voltage alert help prevent brownouts during long practice blocks.
- **Desktop experiments:** PID motor simulator plus a grid-based path planner/visualiser cut down table time when exploring new routes or attachments.
- **Field resource pack:** Updated maps, sketches, and captured paths live in `resources/` so pit crews can sync the same reference set.

## Quick start for drivers

You only need the hub and this repository to run missions; a full Python environment is optional.

1. Open `src/main.py` in the SPIKE Prime app or PyBricks IDE.
2. Download / run the program on the PrimeHub.
3. On the hub:
   - Use the left / right buttons to choose a **mission slot number** shown on the display.
   - Press the centre button to start the selected mission.
4. Swap attachments so they match the mission selected (see team documentation on the table).

Mission slots are defined at the bottom of `src/main.py` using the `@mission("<slot>")` decorator. Keeping attachments and slots in sync is critical for competition day.

## Requirements (for developers)

- LEGO PrimeHub (SPIKE Prime / Robot Inventor) running PyBricks firmware ≥ 3.4, or the LEGO SPIKE App for USB/BLE deployment.
- Python 3.10+ with `matplotlib` for PID plots and optional `pygame` for visualising planner output.
- `pybricksdev` CLI (or the SPIKE/PyBricks IDE) if you prefer terminal-based deployment from the command line.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `src/main.py` | Deployable PrimeHub program containing missions, helpers, and the low-voltage battery monitor. |
| `src/testing/PIDTesting/` | Standalone PID models (`pidcontroller.py`, `motorsimulation.py`) that export plots for arm/drive tuning. |
| `src/testing/PathFinding/` | A* planner, mission CSV exporter, and optional pygame visualiser for obstacle-aware paths. |
| `resources/` | Field maps, attachment sketches, and historical route captures (`PathV*.png`, `Map.pdf`). |
| `AGENTS.md` | Contributor guide covering coding style, command cheatsheets, and pull-request expectations. |
| `LICENSE` | Apache 2.0 licensing information for distribution. |

## Setup & Deployment (developers)

1. Clone or download this repository next to your SPIKE / PrimeHub project folder.
2. (Optional) Create a virtual environment:

   ```bash
   python -m venv .venv
   source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
   pip install matplotlib pygame  # pygame is optional but needed for visualisation.py
   ```

3. Update the missions inside `src/main.py` to match the attachment combo currently installed on the robot.
4. Deploy via GUI or command line. For BLE / USB CLI users:

   ```bash
   pybricksdev run ble --name "MyPrimeHub" src/main.py
   ```

5. On the hub, use the PrimeHub screen and buttons to pick the mission slot, confirm battery status, and run the selected routine.

## Desktop Tooling & Testing

All of these commands assume you are in the repository root and (optionally) have your virtual environment activated.

- **PID tuning:**

  ```bash
  python src/testing/PIDTesting/motorsimulation.py 90 --kp 0.6 --ki 0.05 --kd 0.2 --output out/pid.png
  ```

  Produces angle / speed / duty plots for candidate constants.

- **Path planning & CSV export:**

  ```bash
  python src/testing/PathFinding/pathfinding.py MissionA 150 150 --csv out/MissionA.csv
  ```

  Prints the smoothed waypoint list plus tank-drive instructions and writes a CSV for sharing.

- **Path visualisation (requires pygame):**

  ```bash
  python src/testing/PathFinding/visualisation.py MissionB 160 200 --scale 0.2
  ```

## Field logging checklist

When you change missions or drive constants, try to record:

- Mission name / slot number.
- Attachment combination and any special setup (e.g. starting tile offset, arm preload).
- Battery percentage and whether the low-voltage guard (`LOW_VOLTAGE`) triggered.
- Any new plots, CSVs, or updated field images saved to `resources/` or an `out/` folder.

These notes make it easier for other drivers to reproduce your results.

## Contributing

- Follow the guidance in `AGENTS.md` for coding style, naming, and required test artifacts.
- Use Conventional Commit prefixes seen in history (`fix:`, `chore(main):`, etc.) and scope missions or subsystems when possible.
- Include field resources (updated PNG/PDF, CSV outputs, PID plots) whenever a change impacts routing or attachments.
- Open an issue or pull-request discussion before large refactors—`src/main.py` must stay hub-friendly (single file, limited imports).

## License

Licensed under the [Apache License 2.0](LICENSE). See the license file for permissions and limitations.
