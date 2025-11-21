#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
#include <string>          // <- fehlte
#include "test.hpp"
#include "localization.hpp"
#include "driveforward.hpp"
#include "A_Star.hpp"

// ---- Globals ----

// --- Auton data ---

static int autonMode = 0;
static const char* AUTON_NAMES[] = { "LEFT", "RIGHT", "SKILLS", "CALIB" };
static constexpr int AUTON_COUNT = sizeof(AUTON_NAMES) / sizeof(AUTON_NAMES[0]);

// --- Helper to draw the menu ---
static inline void brainDraw() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(monoM);
  Brain.Screen.setPenColor(white);

  Brain.Screen.printAt(20, 40,  false, "Select Auton (tap)");
  Brain.Screen.printAt(20, 100, false, "^  Prev");
  Brain.Screen.printAt(20, 140, false, "Mode: %s", AUTON_NAMES[autonMode]);
  Brain.Screen.printAt(20, 180, false, "v  Next");

  Brain.Screen.drawRectangle(250, 120, 110, 44, vex::color(20, 200, 20));
  Brain.Screen.printAt(275, 150, false, " OK ");
}

// --- Touch utility ---
static inline bool inRect(int x, int y, int rx, int ry, int rw, int rh) {
  return x >= rx && x <= rx + rw && y >= ry && y <= ry + rh;
}

// --- Auton selector (no time limit) ---
static inline void pre_auton() {
  brainDraw();

  while (true) {
    if (Brain.Screen.pressing()) {
      const int x = Brain.Screen.xPosition();
      const int y = Brain.Screen.yPosition();

      // Tap upper area → previous
      if (inRect(x, y, 10, 80, 220, 40)) {
        autonMode = (autonMode + AUTON_COUNT - 1) % AUTON_COUNT;
        brainDraw();
        wait(180, msec);
      }
      // Tap lower area → next
      else if (inRect(x, y, 10, 170, 220, 40)) {
        autonMode = (autonMode + 1) % AUTON_COUNT;
        brainDraw();
        wait(180, msec);
      }
      // Tap OK → finalize
      else if (inRect(x, y, 250, 120, 110, 44)) {
        break;
      }

      // Wait until finger lifted
      while (Brain.Screen.pressing()) wait(10, msec);
    }
    wait(20, msec);
  }

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(20, 40, false, "Final Auton: %d (%s)", autonMode, AUTON_NAMES[autonMode]);
}
/*static void drawRatingScreen(int rating) {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(monoXXL);
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(20, 60, false,"Rate the run");
  Brain.Screen.setFont(monoM);
  Brain.Screen.printAt(20, 110,  false,"Use Up/Down, A=Confirm, B=Cancel");
  Brain.Screen.setFont(monoXXL);
  Brain.Screen.printAt(20, 200, false, "%d", rating);
}

static void drawNotesScreen(const std::vector<std::string>& notes,
                            const std::vector<bool>& selected,
                            int cursor) {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(monoL);
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(20, 50, false, "Select notes (X=toggle, L/R=move, A=Save, B=Skip)");

  int x0 = 20, y0 = 100, step = 30;
  for (size_t i = 0; i < notes.size(); ++i) {
    int y = y0 + static_cast<int>(i) * step;
    bool isCur = (static_cast<int>(i) == cursor);
    Brain.Screen.setPenColor(isCur ? yellow : white);
    Brain.Screen.printAt(x0, y, false, "%s %s",
                         selected[i] ? "[x]" : "[ ]",
                         notes[i].c_str());
  }
}*/

// Blocks while the operator picks rating + notes.
// Returns true if confirmed, false if canceled.

// ---- nutzt die Helpers ----
void endofauton(void) {
   LeftDrivetrain.stop(coast);
  RightDrivetrain.stop(coast);

}
void intaketankfor(int number) {
  for (int count = 0; count < number; count++) {
    R1Pressed();
    wait(100, msec);
  }
  R1Released();
}

void intakeouthigh(int number) {
  for (int count = 0; count < number; count++) {
    L1Pressed();
    wait(100, msec);
  }
  L1Released();
}

void intakeoutmid(int number) {
  for (int count = 0; count < number; count++) {
    L2Pressed();
    wait(100, msec);
  }
  L2Released();
}
void intakeoutlow(int number) {
  for (int count = 0; count < number; count++) {
    R2Pressed();
    wait(100, msec);
  }
  R2Released();
}
//hard auton
void hardcodedR(void) {
  loadCalibration();

   piston1.set(true);
  piston2.set(true);
  

  driveStraightMm(400);
  turnStepDeg(90);
  driveStraightMm(600);
  turnStepDeg(84);
  piston2.set(false);
  //driveStraightMm(350);
  FullDrivetrain.spin(forward,-50,pct);
  wait(500, msec);
  FullDrivetrain.stop(coast);
  intaketankfor(10);
  
  driveStraightMm(-350);
  piston2.set(true);
  piston1.set(false);
  turnStepDeg(180);
  
  driveStraightMm(167);
  intakeouthigh(30);

}
void hardcodedL(void) {
  loadCalibration();

   piston1.set(true);
  piston2.set(true);
  

  driveStraightMm(400);
  turnStepDeg(-90);
  driveStraightMm(600);
  turnStepDeg(-84);
   piston2.set(false);
   //driveStraightMm(350);
  FullDrivetrain.spin(forward,-50,pct);
  wait(500, msec);
  FullDrivetrain.stop(coast);
  intaketankfor(10);
  
  driveStraightMm(-350);
  piston2.set(true);
  piston1.set(false);
  turnStepDeg(180);
  
  driveStraightMm(167);
  intakeouthigh(30);

}
void AIMODE() {
    loadCalibration();

    // Build field once and keep for entire run
    Field field(3600, 3600, 50.0);
    addFieldObstaclesWithSmallX(field);

    // Navigate step 1
    navigateToGoal(field, -1275.0, -1275.0);   // Example goal

    // Do whatever action on arrival
    // e.g. intake, score, wait, etc.

    // Navigate step 2
    //navigateToGoal(field, 800.0, -400.0);

    // Navigate step 3
    //navigateToGoal(field, 0.0, 1200.0);

    // Add as many goals as you want...
}


void autonomous() {
  switch (autonMode) {
    case 0: hardcodedL();  break;
    case 1: hardcodedR();  break;
    case 2: AIMODE();      break;
    case 3: calibrateDriveForward(1000.0, 4); break;
  }
}
