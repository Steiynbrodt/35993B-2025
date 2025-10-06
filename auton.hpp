#pragma once
#include "vex.h"
using namespace vex;

// ===================== AUTON-PARAMETER (ANPASSEN) =====================
constexpr double AUTON_FWD1 = 80.0;   // erste Rückwärtsfahrt (cm)
constexpr double AUTON_FWD2 = 40.0;   // zweite Rückwärtsfahrt (cm)

constexpr double AUTON_TURN1 = 30.0;  // große Drehung links (Grad)
constexpr double AUTON_TURN2 = 15.0;  // kleine Drehung links (Grad)

constexpr int AUTON_INTAKE_IN  = 1000; // Walze rein (ms)
constexpr int AUTON_INTAKE_OUT = 800;  // Walze raus (ms)

constexpr int AUTON_DRIVE_SPEED = 60; // % Auton Fahrt
constexpr int AUTON_TURN_SPEED  = 45; // % Auton Drehung
// ======================================================================

namespace auton {
  void runAuto();
}
