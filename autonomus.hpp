#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
#include <string>          // <- fehlte
#include "test.hpp"

// --- Globals (single definitions) ---
int autonMode = 0;
const char* AUTON_NAMES[] = { "LEFT", "RIGHT", "SKILLS" };
constexpr int AUTON_COUNT = sizeof(AUTON_NAMES) / sizeof(AUTON_NAMES[0]);

static void renderAuton(int mode, bool showPrompt = true) {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1); Controller1.Screen.print("Auton:");
  Controller1.Screen.setCursor(2, 1);
  // simple arrows with no printf formats
  if (mode == 0) Controller1.Screen.print("  "); else Controller1.Screen.print("^ ");
  Controller1.Screen.print(AUTON_NAMES[mode]);
  if (mode == AUTON_COUNT - 1) Controller1.Screen.print("  "); else Controller1.Screen.print(" v");
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print(showPrompt ? "Up/Down change | A confirm" : "Locked in");
}

// Lock flag to avoid accidental re-entry later
static bool autonLocked = false;

void pre_auton() {
  // --- Time-boxed selector that does not depend on disabled state ---
  // Works with old 2-position switch (always enabled) and proper field control.
  const int MENU_MS = 6000;  // adjust window (ms) as you like
  vex::timer t;

  bool prevUp=false, prevDown=false, prevA=false;
  bool confirmed=false;

  renderAuton(autonMode, true);

  while (!confirmed && t.time(vex::msec) < MENU_MS && !autonLocked) {
    bool up   = Controller1.ButtonUp.pressing();
    bool down = Controller1.ButtonDown.pressing();
    bool a    = Controller1.ButtonA.pressing();

    bool changed = false;

    if (up && !prevUp)  { autonMode = (autonMode + AUTON_COUNT - 1) % AUTON_COUNT; changed = true; Controller1.rumble("."); }
    if (down && !prevDown) { autonMode = (autonMode + 1) % AUTON_COUNT; changed = true; Controller1.rumble("."); }
    if (a && !prevA)    { confirmed = true; Controller1.rumble("-"); }

    if (changed)   renderAuton(autonMode, true);
    if (confirmed) renderAuton(autonMode, false);

    prevUp = up; prevDown = down; prevA = a;
    wait(25, vex::msec);
  }

  // Auto-lock after timeout too (keeps selection; no movement happened)
  autonLocked = true;

  // Safe defaults (no motion, only resets)
  piston1.set(false);
  piston2.set(false);
  piston3.set(false);
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
   piston1.set(true);
  piston2.set(true);
  

  driveStraightMm(400);
  turnStepDeg(90);
  driveStraightMm(600);
  turnStepDeg(92);
   piston2.set(false);
  driveStraightMm(300);
  intaketankfor(30);
  driveStraightMm(-300);
  turnStepDeg(180);
  piston2.set(true);
  piston1.set(false);
  driveStraightMm(200);
  intakeouthigh(30);

}
void hardcodedL(void) {
   piston1.set(true);
  piston2.set(true);
  

  driveStraightMm(400);
  turnStepDeg(-90);
  driveStraightMm(600);
  turnStepDeg(-92);
   piston2.set(false);
  driveStraightMm(300);
  intaketankfor(30);
  driveStraightMm(-300);
  turnStepDeg(180);
  piston2.set(true);
  piston1.set(false);
  driveStraightMm(200);
  intakeouthigh(30);

}
void AIMODE(void) {
  // --- Example usage -----------------------------------------------------------
// Field field(3600, 1800, 50.0);          // 3.6m x 1.8m, 50mm cells
// auto& grid = field.grid();
// // obstacles (in mm):
// field.addRectMm(0, 0, 400, 400);        // 40x40cm block at center
// field.addEdgeMargin(100.0);             // 10cm safety band
// field.inflateByRadius(150.0);           // robot radius 15cm
//
// // start/goal in cell coords:
// int sx = 2, sy = 2;
// int gx = grid.cols()-3, gy = grid.rows()-3;
// grid.setCell(sx, sy, Cellstate::START);
// grid.setCell(gx, gy, Cellstate::GOAL);
//
// auto path = pathfind::astar(grid, sx, sy, gx, gy);
// pathfind::paintPath(grid, path);
}
void autonomous() {
  switch (autonMode) {
    case 0: hardcodedR();  break;
    case 1: hardcodedL(); break;
    case 2: AIMODE(); break;
  }
}

