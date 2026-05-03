# FLL Unearthed 2025 - Team LEBOB

Robot mission system for FIRST LEGO League *SUBMERGED / UNEARTHED 2025* season, featuring autonomous missions for LEGO SPIKE Prime with PyBricks.

## 🎯 Project Overview

This repository focuses on the **Robot Mission System** - autonomous mission programs with on-hub menu selection, PID control, smart navigation, and battery monitoring for the LEGO SPIKE Prime hub running PyBricks firmware.

## ✨ Key Features

- **On-hub mission menu** - Slot-based `@mission("slot")` decorator system for easy mission management
- **Smart PID navigation** - Closed-loop drive and turn routines for precise, repeatable movements
- **Battery safety** - Low-voltage alerts and monitoring to prevent brownouts during competition

## 🚀 Installing firware

* Open [PyBricks.com](code.pybricks.com) and click `Install Firmware` and follow the instructions.

## 📁 Repository Structure

```
FLL-Lebob-Unearthed/
├── src/
│   └── main.py            # Competition missions (deploy to hub)
├── resources/             # Field maps and path visualizations
│   ├── Map.pdf            # Competition field map
│   └── PathV*.png         # Mission path diagrams (V1-V5)
├── LICENSE                # Apache 2.0 License
└── README.md              # This file
```

## 🔧 Requirements

### Hardware

- **LEGO SPIKE Prime Hub** (Robot Inventor hub also compatible)
- **PyBricks firmware** v3.4 or later installed on hub
- **4 motors**: 2 drive motors (ports C, D) + 2 attachment motors (ports E, F)

### Software

- **PyBricks IDE** or **pybricksdev** CLI for deployment

## 💻 Deployment

### Deployment (GUI)

1. Open PyBricks IDE or SPIKE Prime app
2. Load `src/main.py`
3. Connect to hub and click "Download and Run"

### CLI Deployment (Bluetooth)

```bash
pybricksdev run ble --name "FatSean" src/main.py
```

## 🎮 Robot Mission Details

### Hardware Configuration

- **Drive base**: 2 motors (left: Port D, right: Port C), 62.4mm diameter wheels, 150mm axle track
- **Left aux motor**: Port F (attachment arm)
- **Right aux motor**: Port E (attachment arm)
- **Gyro/IMU**: Built-in hub IMU for heading control

### Mission aims

- **Mission 1**: Release ores, Heavy lifting, Who lived here
- **Mission 2**: Silo
- **Mission 3**: Scales, Roof raise, Market wares
- **Mission 4**: Shipwreck
- **Mission 5**: Brushing, Topsoil reveal
- **Mission 6**: Minecart, Precious recovery
- **Mission 7**: Forum, statue, flags

See `src/main.py` for mission code and sequencing.

### PID Control System

The robot includes a custom PID controller for:

- **Smart driving**: Gyro-corrected straight line movement
- **Smart turning**: Precise angle control with overshoot prevention
- **Battery compensation**: Adjusts for voltage drop during runs

Configure PID constants in the `Robot` class initialization.

## 🗺️ Field Resources

Competition field maps and path planning diagrams are stored in `resources/`:

- `Map.pdf` - Official FLL Unearthed field layout
- `PathV1.png` through `PathV5.png` - Documented mission paths and strategies

## 📊 Battery Management

The robot monitors battery voltage and provides visual feedback:

- **High voltage** (>8.4V): Green indicator
- **Medium voltage** (7.2V-8.4V): Orange indicator
- **Low voltage** (<7.2V): Red warning, recommend recharge

Each mission displays battery status before running. The system includes `LOW_VOLTAGE` protection to prevent brownouts during critical movements.

## Contributing

We welcome contributions from team members! Please follow these guidelines:

### Code Style

- Follow PEP 8 conventions (4-space indentation, `snake_case` functions)
- Use descriptive variable names
- Add comments for complex mission logic
- Keep `src/main.py` hub-friendly (minimal imports, single file)

### Mission Development

1. Test missions thoroughly on the competition table
2. Document attachment requirements
3. Note battery voltage during testing
4. Update path diagrams in `resources/` if routes change
5. Use the `@mission("slot")` decorator for new missions

### Pull Requests

- Use [conventional commit](https://gist.github.com/qoomon/5dfcdf8eec66a051ecd85625518cfd13 "Cheat sheet for conventional commits") format: `feat:`, `fix:`, `docs:`, etc.
- Describe mission changes and required attachments
- Include field test results
- Reference related issues

## 🫠 Troubleshooting

### Hub won't connect

- Ensure PyBricks firmware is installed (not LEGO firmware)
- Check Bluetooth is enabled
- Verify hub name matches deployment script (default: "FatSean")
- Try restarting the hub

### Mission runs incorrectly

- Check battery voltage (should be >7.5V for consistent performance)
- Verify attachments are properly installed
- Reset gyro by restarting the mission or hub
- Ensure starting position is correct

## 📄 License

This project is licensed under the [Apache License 2.0](LICENSE).

Copyright © 2026 Team LEBOB - FLL Unearthed

You are free to use, modify, and distribute this code. See the LICENSE file for full terms and conditions.

## 👥 Team

**Team LEBOB** - FIRST LEGO League Unearthed Season 2026

For questions, issues, or contributions, please open an issue on GitHub or contact the team.

---

**Good luck at competition! 🏆**
