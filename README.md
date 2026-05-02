# FLL Unearthed 2025 - Team LEBOB

Autonomous mission program for FIRST LEGO League Unearthed 2025, built for the LEGO SPIKE Prime hub using PyBricks MicroPython.

## Project overview

This repository contains a single on-hub mission runner with:

- A mission selector that cycles through registered missions
- A custom drive base with stall-aware helpers
- Attachment motor control for left/right mechanisms
- Battery status reporting at startup

## Key features

- **Mission selector** - Choose missions on the hub display and launch with the center button
- **Custom drive helpers** - Straight/arc/turn until stalled, with controlled stopping
- **Gyro navigation** - Uses the hub IMU for heading control
- **Attachment control** - Two auxiliary motors for arms/tools

## Quick start

### In PyBricks IDE

1. Open [src/main.py](src/main.py) in PyBricks IDE.
2. Connect to the SPIKE Prime hub and click "Download and Run.

### In Terminal (`ctrl` + ` ` ` for VSC)

1. Navigate to the cloned folder
2. Run `python -m pybricksdev run ble --name "<spike-name (default: FatSean)>" src/main.py`

### Then:

3. Use left/right buttons to pick a mission number, then press center to run it.

## Missions

The missions are registered in [src/main.py](src/main.py) via the `@mission` decorator and run in order. Current set:

- Mission 1: Release boulders, Heavy lifting, and Who lived here
- Mission 2: Silo
- Mission 3: Scale pan, Market roof, Market wares
- Mission 4: Shipwreck
- Mission 5: Brushing and Topsoil
- Mission 6: Minecart and Precious Artefact
- Mission 7: Forum, statue, and flags

## Hardware configuration

- **Hub**: LEGO SPIKE Prime with PyBricks firmware
- **Drive motors**: Port D (left) and Port C (right)
- **Attachment motors**: Port F (left) and Port E (right)
- **Wheel diameter**: 62.4 mm
- **Axle track**: 130 mm

## Repository structure

```
Lebob-Unearthed/
├── src/
│   └── main.py       # Mission program (deploy to hub)
├── resources/        # Field maps and path diagrams
├── LICENSE
└── README.md
```

## Notes

- Battery percentage and raw voltage are printed on startup.
- The code stop button is set to Bluetooth to reduce accidental stops.
- Mission indexing starts at 1 on the hub display.

## License

This project is licensed under the [Apache License 2.0](LICENSE).

Copyright (c) 2025 Team LEBOB - FLL Unearthed
