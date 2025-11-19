# 35993B-2025 – VEX V5 Navigation & Autonomous Code

> Robot source code of team **35993B** for the 2024/2025 VEX V5 season.:contentReference[oaicite:1]{index=1}  

This repository contains the full navigation, localization and autonomous logic used by team 35993B.  
Core features include:

- Grid-based pathfinding (A* on a discretized field)
- High-level navigation interface (`NAVI.hpp`)
- GPS + inertial-based localization
- Autonomous routines and driver-assist helpers
- Utility modules for field parameters, buttons, and testing

---

## Features

### Navigation / Pathfinding

- A* pathfinding on a grid representation of the field (`A_Star.hpp`)
- Conversion between **millimeter coordinates** and **grid cells**
- Automatic obstacle inflation / margins to keep safe distance from objects
- Path smoothing and waypoint generation before execution (where applicable)

### Localization

- Fusion of VEX GPS and inertial sensor data (`localization.hpp`)
- Coordinate system centered at field origin (0, 0) with mm units
- Helper functions to:
  - Get current position and heading
  - Reset / re-zero position
  - Transform between field coordinates and internal grid coordinates

### Motion / Drive

- High-level `NAVI(...)` entry point for “drive to (x, y)” behavior (`NAVI.hpp`)
- Low-level motion helpers in `drive.hpp` / `driveforward.hpp`:
  - Straight driving with speed ramping
  - Turning with heading control
  - Basic tolerance handling for distance + angle

### Autonomous Routines

- Autonomous routines defined in:
  - `auton.hpp`
  - `autonomus.hpp`
- Typical structure:
  - Set initial pose
  - Call `NAVI(targetXmm, targetYmm)` multiple times
  - Run mechanism actions (intake, launcher, etc.) in between movements

### Utilities

- `fieldparameters.hpp` – field geometry and constants
- `buttons.hpp` – controller button mappings and callbacks
- `helpers.hpp` – generic helpers (logging, time, math, etc.)
- `emergencyswapminmax.hpp` – safety or emergency overrides
- `documentation.txt` / **VEX Navigation System (NAVI) - Documentation** – extra notes and experiments:contentReference[oaicite:2]{index=2}  

---
This repository contains the competition code for VEX V5 Team 35993B (Season 2024–2025).
All source files are written in VEXcode Pro V5 (C++) and organized into modular components for driving, localization, and autonomous routines.

📂 Repository Structure
main.cpp
auton.hpp
autonomus.hpp
drive.hpp
driveforward.hpp
localization.hpp
fieldparameters.hpp
buttons.hpp
helpers.hpp
emergencyswapminmax.hpp
vex.h


Below is a description of each part of the codebase.

▶️ main.cpp

Main entry point for the robot program.

Initializes all motors and sensors (vexcodeInit()).

Registers competition callbacks:

autonomous()

usercontrol()

Contains the primary runtime loop.

This file should not contain robot logic beyond setup and mode selection.

▶️ auton.hpp

Primary file for autonomous routines.

Contains one or more functions defining autonomous paths.

Implements movement using functions from drive.hpp.

May call mechanism control (intake, launcher, clamp, etc.).

The autonomous() function in main.cpp chooses which auton to run.

All auton logic should be added or modified here.

▶️ autonomus.hpp

Additional autonomous support.

Stores secondary or experimental autonomous routines.

Helps keep auton.hpp organized.

Can hold small helper sequences or specialized task routines.

Used optionally depending on team needs.

▶️ drive.hpp

Low-level motion control.

Provides the core movement functions, typically including:

driveForward(distance_mm)

turnToHeading(angle_deg)

Speed/throttle management

Movement tolerances

Basic movement control loops

All higher-level autonomous motions depend on these functions.

▶️ driveforward.hpp

Additional straight-line drive logic.

Implements refined forward driving (PID or proportional).

Used where consistent forward motion is required.

May be called internally by drive.hpp.

▶️ localization.hpp

GPS + inertial-based localization system.

Tracks:

x position (mm)

y position (mm)

heading (degrees)

Provides:

setPose(x, y, heading)

Position getters

Pose updates

Coordinate transformations (if required)

Used by auton to ensure consistent motion.

▶️ fieldparameters.hpp

Field geometry and global constants.

May include:

Tile size

Field boundary definitions

Starting coordinates

Scale factors for unit conversion

Referenced by localization and autonomous code.

▶️ buttons.hpp

Controller button logic.

Maps controller buttons to functions.

Enables auton selection (if implemented).

Handles driver-control shortcuts or toggles.

▶️ helpers.hpp

Utility functions used across the codebase, such as:

Math helpers

Clamping/threshold utilities

Logging or debug helpers (if used)

▶️ emergencyswapminmax.hpp

Safety and emergency utility logic.

Ensures values remain within safe limits.

Used for stability or emergency fallback.

▶️ vex.h

Auto-generated hardware configuration file.

Declares motors, sensors, ports, and directions.

Managed by VEXcode device configuration.

Should not be edited manually.

🔧 Building & Deploying

Open the project in VEXcode Pro V5.

Ensure the hardware configuration in vex.h matches the actual robot.

Press Build.

Download to the V5 Brain via USB or controller tether.

🧪 Autonomous Development Notes

Autonomous must be added/modified in auton.hpp.

Movement is performed using functions in drive.hpp.

Localization is optional but available via localization.hpp.

No symbolic pathfinding (NAVI) is included; all movement is hardcoded.

Example general auton structure:

void myAuton() {
    
    driveForward(600);
    turnToHeading(90);
    driveForward(300);
}

🤝 Contributing

Keep autonomous logic inside auton.hpp / autonomus.hpp.

Avoid modifying vex.h directly.

Maintain function documentation where applicable.
## Repository Structure

Key files (simplified):

```text
A_Star.hpp                       # Grid-based A* pathfinding
NAVI.hpp                         # High-level navigation interface (NAVI function) deprecated
NAVI_fixed.hpp                   # Frozen/experimental version of NAVI deprecated
auton.hpp                        # Main autonomous routines
autonomus.hpp                    # Additional autonomous helpers
buttons.hpp                      # Controller button mappings & related logic
drive.hpp                        # Low-level driving and turning functions
driveforward.hpp                 # Straight-driving helper / test
localization.hpp                 # GPS + inertial based localization functions
fieldparameters.hpp              # Field dimensions and constants
helpers.hpp                      # Misc helper functions (logging, math, etc.)
emergencyswapminmax.hpp          # Safety / parameter swap utilities
main.cpp                         # Program entry point, competition callbacks
vex.h                            # VEXcode-provided header
documentation.txt                # Developer documentation and notes
VEX Navigation System (NAVI)...  # Additional design doc (PDF or text)
📘 35993B-2025 — Robot Code Documentation

