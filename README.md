# FLL Unearthed 2025 - Team LEBOB

Complete robotics control system for FIRST LEGO League *SUBMERGED / UNEARTHED 2025* season, featuring autonomous missions, teleoperation, and deployment tooling for LEGO SPIKE Prime with PyBricks.

## 🎯 Project Overview

This repository contains two main components:

1. **Robot Mission System** - Autonomous mission programs with on-hub menu selection
2. **Teleoperation System** - Xbox controller support for manual robot control with recording/playback

Both systems are designed for the LEGO SPIKE Prime hub running PyBricks firmware, providing advanced features like PID control, smart navigation, and battery monitoring.

## ✨ Key Features

### Robot Mission System
- **On-hub mission menu** - Slot-based `@mission("slot")` decorator system for easy mission management
- **Smart PID navigation** - Closed-loop drive and turn routines for precise, repeatable movements
- **Battery safety** - Low-voltage alerts and monitoring to prevent brownouts during competition
- **Attachment control** - Manual jog mode for testing and positioning attachments
- **Multiple mission support** - 7+ pre-programmed missions ready for competition

### Teleoperation System
- **Xbox controller support** - Tank drive controls with auxiliary motor management
- **Recording & playback** - Record teleop runs and generate hub-deployable replay scripts
- **Real-time streaming** - BLE-based controller input streaming to the hub
- **Auto-generation** - Recorded runs automatically convert to standalone Python programs

## 🚀 Quick Start

### For Robot Drivers (Competition Mode)

1. **Flash the robot program** to your SPIKE Prime hub:
   - Open `src/robot/main.py` in the PyBricks IDE
   - Connect to your hub and click "Download and Run"

2. **On the hub display**:
   - Use left/right buttons to select a mission number (1-7, T, M)
   - Press center button to start the mission
   - Robot backs up slightly and resets gyro before each run

3. **Mission slots**:
   - `1-7` - Competition missions (check AGENTS.md for attachment requirements)
   - `T` - Test mission (drive square pattern)
   - `M` - Manual attachment mode (left/right buttons jog motors)

### For Teleop Drivers

1. **Install dependencies**:
   ```bash
   pip install pybricksdev evdev
   ```

2. **Connect Xbox controller** to your computer via USB or Bluetooth

3. **Deploy and run teleop**:
   ```bash
   cd src/robot
   python deploy_teleop.py --hub "YourHubName"
   ```

4. **Controls**:
   - Left stick Y-axis → Left drive motor
   - Right stick Y-axis → Right drive motor
   - Left trigger → Left aux motor (forward)
   - Right trigger → Right aux motor (forward)
   - Left bumper + trigger → Reverse left aux
   - Right bumper + trigger → Reverse right aux
   - **B button** → Start/stop recording
   - **A button** → Replay last recording

## 📁 Repository Structure

```
FLL-Lebob-Unearthed/
├── src/
│   ├── robot/              # Main robot control system
│   │   ├── main.py         # Competition missions (deploy to hub)
│   │   ├── aaron_main.py   # Alternate mission configuration
│   │   ├── teleop_hub.py   # Teleop receiver (runs on hub)
│   │   ├── deploy.py       # Simple deployment script
│   │   └── deploy_teleop.py # Xbox controller teleop system
│   └── innovations/        # Innovation project code
│       └── main.py
├── resources/              # Field maps and path visualizations
│   ├── Map.pdf            # Competition field map
│   └── PathV*.png         # Mission path diagrams (V1-V5)
├── out/                   # Generated outputs and recordings
│   ├── teleop-recording-*.csv  # Teleop recording data
│   └── teleop-recording-*.py   # Auto-generated replay scripts
├── AGENTS.md              # Contributor guidelines and coding standards
├── LICENSE                # Apache 2.0 License
└── README.md              # This file
```

## 🔧 Requirements

### Hardware
- **LEGO SPIKE Prime Hub** (Robot Inventor hub also compatible)
- **PyBricks firmware** v3.4 or later installed on hub
- **4 motors**: 2 drive motors (ports C, D) + 2 aux motors (ports E, F)
- **Xbox controller** (for teleop mode only)

### Software
- **Python 3.8+** for development scripts
- **PyBricks IDE** or **pybricksdev** CLI for deployment
- **evdev** (Linux) for Xbox controller support in teleop mode

### Python Dependencies (Teleop)
```bash
pip install pybricksdev evdev
```

## 💻 Development & Deployment

### Simple Deployment (GUI)

1. Open PyBricks IDE or SPIKE Prime app
2. Load `src/robot/main.py`
3. Connect to hub and click "Download and Run"

### CLI Deployment (Bluetooth)

Using the included deployment script:
```bash
cd src/robot
python deploy.py
```

Or manually with pybricksdev:
```bash
pybricksdev run ble --name "FatSean" src/robot/main.py
```

### Teleop Deployment

Deploy teleop and stream Xbox controller input:
```bash
cd src/robot
python deploy_teleop.py --hub "FatSean" --debug
```

Options:
- `--hub NAME` - Hub Bluetooth name (default: "FatSean")
- `--device PATH` - Specific input device (e.g., `/dev/input/event18`)
- `--deadband VALUE` - Controller deadzone (default: 0.05)
- `--debug` - Show detailed connection and input data
- `--no-record` - Disable recording features
- `--record-out PATH` - Custom recording output path

### Recording & Playback

1. **Start recording**: Press **B button** during teleop
2. **Stop recording**: Press **B button** again
3. **Replay last**: Press **A button**

Recordings are saved to `out/teleop-recording-TIMESTAMP.csv` and auto-converted to deployable Python scripts in `out/teleop-recording-TIMESTAMP.py`.

To replay a recording on the hub:
```bash
pybricksdev run ble --name "FatSean" out/teleop-recording-TIMESTAMP.py
```

## 🎮 Robot Mission Details

### Hardware Configuration

- **Drive base**: 2 motors, 62.4mm diameter wheels, 150mm axle track
- **Left aux motor**: Port F (attachment arm)
- **Right aux motor**: Port E (attachment arm)
- **Gyro/IMU**: Built-in hub IMU for heading control

### Mission Highlights

- **Mission 1**: Coral reef mission with precise positioning
- **Mission 2**: Complex curve navigation with stall detection
- **Mission 3**: Shipwreck mission with lever pulling mechanism
- **Mission 4**: Platform flip with raising bucket
- **Mission 5**: Statue lifting and transport
- **Mission 6**: High-speed attachment spinning
- **Mission 7**: Attachment test routine
- **Mission T**: Test pattern (drive square)
- **Mission M**: Manual attachment control mode

See `src/robot/main.py` for detailed mission code and sequencing.

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

Use these resources for:
- Planning new missions
- Documenting successful runs
- Team strategy discussions
- Driver training

## 📊 Battery Management

The robot monitors battery voltage and provides visual feedback:

- **High voltage** (>8.4V): Green indicator
- **Medium voltage** (7.2V-8.4V): Orange indicator  
- **Low voltage** (<7.2V): Red warning, recommend recharge

Each mission displays battery status before running. The system includes `LOW_VOLTAGE` protection to prevent brownouts during critical movements.

## 🤝 Contributing

We welcome contributions from team members! Please follow these guidelines:

### Code Style
- Follow PEP 8 conventions (4-space indentation, `snake_case` functions)
- Use descriptive variable names
- Add comments for complex mission logic
- Keep `src/robot/main.py` hub-friendly (minimal imports, single file)

### Mission Development
1. Test missions thoroughly on the competition table
2. Document attachment requirements
3. Note battery voltage during testing
4. Update path diagrams in `resources/` if routes change
5. Use the `@mission("slot")` decorator for new missions

### Pull Requests
- Use conventional commit format: `feat:`, `fix:`, `docs:`, etc.
- Describe mission changes and required attachments
- Include field test results
- Reference related issues

See `AGENTS.md` for detailed contributor guidelines.

## 🐛 Troubleshooting

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

### Teleop connection issues
- Verify Xbox controller is connected: `ls /dev/input/event*`
- Check controller batteries
- Try `--debug` flag to see connection details
- Ensure no other programs are using the controller

### Recording not working
- Check `out/` directory exists and is writable
- Ensure recording wasn't disabled with `--no-record`
- Verify there's disk space available
- Look for error messages in debug output

## 📄 License

This project is licensed under the [Apache License 2.0](LICENSE).

Copyright © 2025 Team LEBOB - FLL Unearthed

You are free to use, modify, and distribute this code. See the LICENSE file for full terms and conditions.

## 👥 Team

**Team LEBOB** - FIRST LEGO League Unearthed Season 2025

For questions, issues, or contributions, please open an issue on GitHub or contact the team.

---

**Good luck at competition! 🏆**
