# Unearthed 2025 Robotics Codebase

PrimeHub-ready mission software plus desktop tooling for the LEBOB FIRST LEGO League *SUBMERGED / UNEARTHED 2025* season. The repository keeps the tournament program in a single hub-friendly file, while notebooks and simulators help us tune PID profiles, plan obstacle routes, and document field experiments.

## Highlights
- **Mission menu on-hub:** Slot-based decorator system (`@mission("1")`, etc.) keeps attachments organized and allows rapid iteration without editing firmware scaffolding.
- **Safety-first drive helpers:** Smart PID drive/turn routines, torque caps, and a low-voltage alert prevent brownouts during long practice blocks.
- **Desktop experiments:** PID motor simulator plus a grid-based path planner/visualiser cut down table time when exploring new routes or attachments.
- **Field resource pack:** Updated maps, sketches, and captured paths live in `resources/` so pit crews can sync the same reference set.

## Requirements
- LEGO PrimeHub (SPIKE Prime/Robot Inventor) running PyBricks firmware ≥ 3.4, or the LEGO SPIKE App for USB/BLE deployment.
- Python 3.10+ with `matplotlib` for PID plots and optional `pygame` for visualising planner output.
- `pybricksdev` CLI (or the SPIKE/PyBricks IDE) if you prefer terminal-based deployment.

## Repository Layout
| Path | Purpose |
| --- | --- |
| `src/main.py` | Deployable PrimeHub program containing missions, helpers, and the low-voltage battery monitor. |
| `src/testing/PIDTesting/` | Standalone PID models (`pidcontroller.py`, `motorsimulation.py`) that export plots for arm/drive tuning. |
| `src/testing/PathFinding/` | A* planner, mission CSV exporter, and optional pygame visualiser for obstacle-aware paths. |
| `resources/` | Field maps, attachment sketches, and historical route captures (`PathV*.png`, `Map.pdf`). |
| `AGENTS.md` | Contributor guide covering coding style, command cheatsheets, and PR expectations. |
| `LICENSE` | Apache 2.0 licensing information for distribution. |

## Setup & Deployment
1. Clone or download the repo next to your SPIKE/PrimeHub project folder.
2. (Optional) Create a virtual environment:  
   ```bash
   python -m venv .venv && source .venv/bin/activate
   pip install matplotlib pygame  # pygame is optional but needed for visualisation.py
   ```
3. Update missions inside `src/main.py` to match the attachment combo currently installed on the robot.
4. Deploy via GUI or command line. For BLE/USB CLI users:  
   ```bash
   pybricksdev run ble --name "MyPrimeHub" src/main.py
   ```
5. On the hub, use the PrimeHub screen/buttons to pick the mission slot, confirm battery status, and run the selected routine.

## Desktop Tooling & Testing
- **PID tuning:**  
  ```bash
  python src/testing/PIDTesting/motorsimulation.py 90 --kp 0.6 --ki 0.05 --kd 0.2 --output out/pid.png
  ```  
  Produces angle/speed/duty plots for candidate constants.
- **Path planning & CSV export:**  
  ```bash
  python src/testing/PathFinding/pathfinding.py MissionA 150 150 --csv out/MissionA.csv
  ```  
  Prints the smoothed waypoint list plus tank-drive instructions and writes a CSV for sharing.
- **Path visualisation (requires pygame):**  
  ```bash
  python src/testing/PathFinding/visualisation.py MissionB 160 200 --scale 0.2
  ```

Log notable runs (mission, attachment, battery %) in your PR description so other drivers can reproduce table conditions.

## Contributing
- Follow the guidance in `AGENTS.md` for coding style, naming, and required test artifacts.
- Use Conventional Commit prefixes seen in history (`fix:`, `chore(main):`, etc.) and scope missions or subsystems when possible.
- Include field resources (updated PNG/PDF, CSV outputs, PID plots) whenever a change impacts routing or attachments.
- Open an issue or PR discussion before large refactors—`src/main.py` must stay hub-friendly (single file, limited imports).

## License
Licensed under the [Apache License 2.0](LICENSE). See the license file for permissions and limitations.
