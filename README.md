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
