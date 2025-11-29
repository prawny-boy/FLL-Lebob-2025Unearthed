# Repository Guidelines

This file is aimed at **contributors** (students and mentors) who edit the code or documentation. For how to simply run missions on the hub, see the quick-start section in `README.md`.

## Project Structure & Module Organization

- `src/main.py` is the **single deployable PrimeHub program**. Keep runtime logic, menu handlers, and motion helpers consolidated here so the SPIKE / PyBricks tools can sync one file directly.
- `src/testing/PIDTesting/` holds desktop-only PID prototypes (`pidcontroller.py`, `motorsimulation.py`) for tuning turn and arm profiles.
- `src/testing/PathFinding/` provides the waypoint planner plus optional `pygame` visualiser; use it to pre-plan obstacle routing before encoding missions in `src/main.py`.
- `resources/` stores supporting field maps (`Map.pdf`, `PathV*.png`) and other reference material; update these whenever missions or attachments change.

Keep `src/main.py` **hub-friendly**:

- Avoid heavy third-party dependencies; stick to `pybricks.*` and `pybricks.tools` on-hub.
- Favour simple, readable helpers over deep module trees.

## Build, Test, and Development Commands

Most commands below are also listed (briefly) in `README.md`; this section is the contributor-focused reference.

- **Deploy to a hub**
  - GUI: open `src/main.py` in the SPIKE Prime app or PyBricks IDE and download it to the hub.
  - CLI (BLE example):
    - `pybricksdev run ble --name "<HubName>" src/main.py`

- **Run a virtual PID experiment and emit a plot**
  - `python src/testing/PIDTesting/motorsimulation.py 90 --kp 0.6 --ki 0.05 --kd 0.2 --output out/pid.png`

- **Generate waypoint CSV plus tank-drive instructions**
  - `python src/testing/PathFinding/pathfinding.py MissionA 150 150 --csv out/MissionA.csv`

- **Visualise a mission path (requires pygame)**
  - `python src/testing/PathFinding/visualisation.py MissionB 160 200 --scale 0.2`

If you add new helper scripts, follow the same style and document them here.

## Coding Style & Naming Conventions

- Follow PEP 8 defaults: 4-space indentation, `snake_case` for functions and helpers, and `CAPS` for module-level constants such as `LOW_VOLTAGE`.
- Keep the mission menu flow readable—group mission helpers logically, and give complex manoeuvres short docstrings or comments.
- Prefer dataclasses and typed signatures in desktop testing utilities; PrimeHub code can use lightweight type hints, but avoid runtime-heavy or large dependencies.
- Keep assets lightweight; imports beyond `pybricks.*` and `pybricks.tools` belong only in desktop experiments under `src/testing`.

When editing `src/main.py`:

- Keep mission slots and attachments in sync, and avoid breaking existing mission numbers unless the team agrees.
- Consider how changes will behave on a nearly-empty battery and under repeated runs.

## Testing & Field Logging

- Treat `src/testing` scripts as the **regression suite**:
  - Capture plots / CSVs from PID runs and planners whenever you change drive constants or obstacle maps.
  - Attach these artefacts to reviews or store them under an `out/` folder.
- On-hub verification is mandatory:
  - Log mission / attachment combinations you exercised.
  - Note whether the low-voltage guard (`LOW_VOLTAGE`) triggered and at approximately what battery percentage.
- Name exported files with the mission and date (for example `out/missionC-2025-09-15.csv`) so field coaches can trace tuning history.

Try to keep at least one known-good CSV / plot per mission so new drivers have a reference.

## Commit & Pull Request Guidelines

- Follow the Conventional Commit format observed in history (`fix:`, `chore(main):`, etc.); include an optional scope describing the touched subsystem (for example `feat(pathfinding): ...`).
- Each PR should:
  - Describe the mission(s) touched and the intent of the change.
  - List reproduction steps (which commands to run, which mission slot to use, starting position, attachments).
  - Include screenshots or files from `resources/` or `out/` when visual context helps (maps, path screenshots, PID plots).
- Link issues or note field observations prompting the change, and call out any manual steps (for example recalibrating gyro or swapping attachments) that reviewers must know before testing.

Prefer smaller, focused PRs over large refactors. For any big change in `src/main.py`, start a discussion first so the team can weigh trade-offs.

