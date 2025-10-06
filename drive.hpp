#pragma once
#include "vex.h"
using namespace vex;

// ============ OPTIONAL: Pneumatik aktivieren/deaktivieren ============
#ifndef HAVE_PNEUMATICS
#define HAVE_PNEUMATICS 0   // 0 = kein digital_out im Projekt, 1 = vorhanden
#endif
// =====================================================================

// ======================= Externe Geräte (aus main.cpp) ===============
extern controller  Controller1;

extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;

extern motor Motorblau;
extern motor Intakespeicher;
extern motor Intakeoben;

#if HAVE_PNEUMATICS
extern digital_out DigitalOutA;
#endif
// =====================================================================

namespace drive {

  // -------- Init --------
  inline void initSpeeds() {
    L1.setVelocity(90,  percentUnits::pct);
    L2.setVelocity(90,  percentUnits::pct);
    L3.setVelocity(90,  percentUnits::pct);
    R1.setVelocity(100, percentUnits::pct);
    R2.setVelocity(100, percentUnits::pct);
    R3.setVelocity(100, percentUnits::pct);
  }
  inline void initTorques() {
    L1.setMaxTorque(90,  percentUnits::pct);
    L2.setMaxTorque(90,  percentUnits::pct);
    L3.setMaxTorque(90,  percentUnits::pct);
    R1.setMaxTorque(100, percentUnits::pct);
    R2.setMaxTorque(100, percentUnits::pct);
    R3.setMaxTorque(100, percentUnits::pct);
  }
  inline void initIntake() {
    Motorblau.setVelocity(100,      percentUnits::pct);
    Motorblau.setMaxTorque(100,     percentUnits::pct);
    Intakespeicher.setVelocity(100, percentUnits::pct);
    Intakespeicher.setMaxTorque(100,percentUnits::pct);
    Intakeoben.setVelocity(100,     percentUnits::pct);
    Intakeoben.setMaxTorque(100,    percentUnits::pct);
  }

  // ------------------- Intake Handler -------------------
  inline void stopIntakes() {
    Motorblau.stop();
    Intakespeicher.stop();
    Intakeoben.stop();
  }

  inline void onR1Pressed() {  // Vollgas rein
    Intakespeicher.spin(directionType::fwd, 100, velocityUnits::pct);
    Motorblau.spin     (directionType::fwd, 100, velocityUnits::pct);
    Intakeoben.spin    (directionType::fwd, 100, velocityUnits::pct);
  }
  inline void onR1Released() { stopIntakes(); }

  // >>>>>>>>>>>>>>> HIER geändert: R2 <<<<<<<<<<<<<<
  inline void onR2Pressed() {
    Motorblau.spin     (directionType::fwd, 100, velocityUnits::pct);
    Intakespeicher.spin(directionType::fwd, 100, velocityUnits::pct);
    Intakeoben.spin    (directionType::fwd,  70, velocityUnits::pct);
  }
  inline void onR2Released() { stopIntakes(); }

  inline void onL1Pressed() {
    Intakespeicher.spin(directionType::rev, 100, velocityUnits::pct);
    Motorblau.spin     (directionType::fwd, 100, velocityUnits::pct);
  }
  inline void onL1Released() { stopIntakes(); }

  inline void onL2Pressed() {
    Motorblau.spin     (directionType::rev, 100, velocityUnits::pct);
    Intakeoben.spin    (directionType::fwd, 100, velocityUnits::pct);
    Intakespeicher.spin(directionType::rev, 100, velocityUnits::pct);
  }
  inline void onL2Released() { stopIntakes(); }

  inline void onYPressed() {
    Motorblau.spin     (directionType::fwd, 100, velocityUnits::pct);
    Intakespeicher.spin(directionType::rev, 100, velocityUnits::pct);
  }
  inline void onYReleased() { stopIntakes(); }

  inline void onRightPressed() {
    Motorblau.spin     (directionType::fwd, 100, velocityUnits::pct);
    Intakespeicher.spin(directionType::fwd, 100, velocityUnits::pct);
  }
  inline void onRightReleased() { stopIntakes(); }

  inline void onUpPressed() {
    Motorblau.spin  (directionType::fwd, 100, velocityUnits::pct);
    Intakeoben.spin (directionType::fwd, 100, velocityUnits::pct);
  }
  inline void onUpReleased() { stopIntakes(); }

  inline void onDownPressed() {
    Motorblau.spin  (directionType::fwd, 100, velocityUnits::pct);
    Intakeoben.spin (directionType::rev, 100, velocityUnits::pct);
  }
  inline void onDownReleased() { stopIntakes(); }

  inline void onBPressed() {
  #if HAVE_PNEUMATICS
    DigitalOutA.set(true);
  #endif
  }
  inline void onXPressed() {
  #if HAVE_PNEUMATICS
    DigitalOutA.set(false);
  #endif
  }

  inline void bindEvents() {
    Controller1.ButtonR1.pressed(onR1Pressed);
    Controller1.ButtonR1.released(onR1Released);
    Controller1.ButtonR2.pressed(onR2Pressed);
    Controller1.ButtonR2.released(onR2Released);
    Controller1.ButtonL1.pressed(onL1Pressed);
    Controller1.ButtonL1.released(onL1Released);
    Controller1.ButtonL2.pressed(onL2Pressed);
    Controller1.ButtonL2.released(onL2Released);
    Controller1.ButtonY.pressed(onYPressed);
    Controller1.ButtonY.released(onYReleased);
    Controller1.ButtonRight.pressed(onRightPressed);
    Controller1.ButtonRight.released(onRightReleased);
    Controller1.ButtonUp.pressed(onUpPressed);
    Controller1.ButtonUp.released(onUpReleased);
    Controller1.ButtonDown.pressed(onDownPressed);
    Controller1.ButtonDown.released(onDownReleased);
    Controller1.ButtonB.pressed(onBPressed);
    Controller1.ButtonX.pressed(onXPressed);
  }

  // ------------------- Arcade-Drive Loop -------------------
inline void runDriveLoop() {
    while (true) {
      // Joystick-Eingaben
      double forward = Controller1.Axis3.position(percentUnits::pct);
      double turn    = Controller1.Axis1.position(percentUnits::pct);

      // --- ALLES invertieren ---
      // vorwärts = rückwärts, rechts = links
      forward = -forward;
      turn    = -turn;

      // Arcade mischen (nach Invertierung)
      double leftMix  = forward + turn;
      double rightMix = forward - turn;

      // Auf Motoren anwenden
      L1.spin(directionType::fwd, leftMix,  velocityUnits::pct);
      L2.spin(directionType::fwd, leftMix,  velocityUnits::pct);
      L3.spin(directionType::fwd, leftMix,  velocityUnits::pct);

      R1.spin(directionType::fwd, rightMix, velocityUnits::pct);
      R2.spin(directionType::fwd, rightMix, velocityUnits::pct);
      R3.spin(directionType::fwd, rightMix, velocityUnits::pct);

      wait(15, timeUnits::msec);
    }
  }

  inline void run() {
    bindEvents();
    runDriveLoop();
  }
}
