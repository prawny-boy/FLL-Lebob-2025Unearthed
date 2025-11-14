## Unearthed 2025 Robotics Codebase

This repository hosts our field-ready PrimeHub program plus a few experiment
scripts that help us iterate on strategies for the FIRST LEGO League
*SUBMERGED* / *UNEARTHEd 2025* season.

### Layout
- `src/main.py` – primary on-robot program (kept as a single file for hub compatibility)
- `src/testing/` – notebooks/scripts used for PID and pathfinding prototyping
- `resources/` – reference field maps, sketches, and iteration captures

### Usage
1. Download the repository into the SPIKE Prime app or PyBricks project folder.
2. Open `src/main.py`, adjust mission routines to match the attachments you are
   using, and deploy to the hub.
3. Optional: run the tooling inside `src/testing/` from a desktop Python
   interpreter to pre-plan new missions.

### Notes
- The main file intentionally bundles all runtime logic so PyBricks on-hub menu
  navigation keeps working.
- Be sure to charge the hub before long tuning sessions; the program will alert
  you when voltage drops below 70%.
