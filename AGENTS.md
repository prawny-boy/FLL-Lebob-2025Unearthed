# Repository Guidelines

## Project Structure & Module Organization
- `src/main.py` is the singular, deployable PrimeHub program; keep runtime logic, menu handlers, and motion helpers consolidated here so SPIKE/PyBricks can sync the file directly.
- `src/testing/PIDTesting/` holds desktop-only PID prototypes (`pidcontroller.py`, `motorsimulation.py`) for tuning turn and arm profiles.
- `src/testing/PathFinding/` provides the waypoint planner plus optional pygame visualiser; use it to pre-plan obstacle routing before encoding missions in `main.py`.
- `resources/` stores supporting field maps (`Map.pdf`, `PathV*.png`); update these whenever missions or attachments change.

## Build, Test, and Development Commands
- Deploy to a hub by opening `src/main.py` in the SPIKE Prime app or PyBricks IDE, or via CLI: `pybricksdev run ble --name "<HubName>" src/main.py`.
- Run a virtual PID experiment and emit a plot: `python src/testing/PIDTesting/motorsimulation.py 90 --kp 0.6 --ki 0.05 --kd 0.2 --output out/pid.png`.
- Generate waypoint CSV plus tank-drive instructions: `python src/testing/PathFinding/pathfinding.py MissionA 150 150 --csv out/missionA.csv`.
- Visualise a mission path (pygame required): `python src/testing/PathFinding/visualisation.py MissionB 160 200 --scale 0.2`.

## Coding Style & Naming Conventions
- Follow PEP 8 defaults: 4-space indentation, `snake_case` for functions/helpers, and `CAPS` for module-level constants such as `LOW_VOLTAGE`.
- Keep the menu flow readable—group mission helpers logically and annotate complex maneuvers with short docstrings.
- Prefer dataclasses and typed signatures in testing utilities; prime-hub code can use lightweight type hints where feasible but avoid runtime-heavy dependencies.
- Keep assets lightweight; imports beyond `pybricks.*` and `pybricks.tools` belong only in desktop experiments.

## Testing Guidelines
- Treat `src/testing` scripts as the regression suite: capture plots/CSVs from PID runs and planners whenever you change drive constants or obstacle maps, and attach them to reviews.
- On-hub verification is still mandatory—log mission/attachment combinations you exercised plus whether the low-voltage guard (`LOW_VOLTAGE = 7000`) triggered.
- Name exported files with the mission and date (e.g., `out/missionC-2024-09-15.csv`) so field coaches can trace the tuning lineage.

## Commit & Pull Request Guidelines
- Follow the existing Conventional Commit format observed in history (`fix:`, `chore(main):`, etc.); include an optional scope describing the touched subsystem.
- Each PR should describe the mission(s) touched, how to reproduce the change (commands above), and include screenshots or files from `resources`/`out` when visual context helps.
- Link issues or note field observations prompting the change, and call out any manual steps (e.g., recalibrating gyro, swapping attachments) reviewers must know before testing.

