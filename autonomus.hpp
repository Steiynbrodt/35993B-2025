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

// ---- Globals ----

// --- Auton data ---

static int autonMode = 0;
static const char* AUTON_NAMES[] = { "LEFT", "RIGHT", "SKILLS" };
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
void AIMODE(void) {
    // ---------- 1) Build field (in mm) ----------
    // 3.6m x 3.6m field, 50mm grid cells  → 72 x 72 grid
    Field field(3600, 3600, 50.0);
    auto& grid = field.grid();

    // ---------- 2) Get start from GPS (on empty grid) ----------
    // Convert GPS (mm) → start cell + heading
    StartPoseCell sp = getStartFromGPS(field);
    if (!sp.valid) {
        Brain.Screen.clearScreen();
        Brain.Screen.print("GPS start invalid");
        return;
    }

    // ---------- 3) Define obstacles in mm ----------
    // Safety band 10cm around edges
   addFieldObstaclesWithSmallX(field);

    // Mark start cell (overwrites whatever inflation did at that cell)
    grid.setCell(sp.sx, sp.sy, Cellstate::START);

    // ---------- 4) Set goal in mm ----------
    // Example goal: somewhere in the south-west part of the field.
    // You can change these two numbers only, everything else updates.
    const double goalXmm = -1275.0;   // X in mm (left/right, 0 is center)
    const double goalYmm = -1275.0;   // Y in mm (forward/back, 0 is center)

    int gx = 0, gy = 0;
    if (!field.mmToCell(goalXmm, goalYmm, gx, gy)) {
        Brain.Screen.clearScreen();
        Brain.Screen.print("Goal outside field");
        return;
    }
    grid.setCell(gx, gy, Cellstate::GOAL);

    // ---------- 5) Debug info on Brain ----------
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("S:(%d,%d) G:(%d,%d)", sp.sx, sp.sy, gx, gy);

    double sXmm, sYmm, gXmm, gYmm;
    field.cellToMmCenter(sp.sx, sp.sy, sXmm, sYmm);
    field.cellToMmCenter(gx, gy, gXmm, gYmm);

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Smm:(%.0f,%.0f)", sXmm, sYmm);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Gmm:(%.0f,%.0f)", gXmm, gYmm);

    // ---------- 6) Run A* ----------
    auto pathPairs = pathfind::astar(grid, sp.sx, sp.sy, gx, gy);
    if (pathPairs.empty()) {
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("No path");
        return;
    }

    // Convert to CellXY for navigatePath
    std::vector<CellXY> path;
    path.reserve(pathPairs.size());
    for (auto& p : pathPairs)
        path.push_back({ p.first, p.second });

    // ---------- 7) Drive the path ----------
    navigatePath(
        path,
        field.cellMm(),     // cell size in mm
        sp.headingDeg,      // starting heading from GPS
        25.0,               // max heading step per segment (deg) – tune
        turnStepDeg,        // your turn primitive
        driveStraightMm     // your drive primitive
    );

    // ---------- 8) Optional: mark the path for debugging ----------
    pathfind::paintPath(grid, pathPairs);
}


void autonomous() {
  switch (autonMode) {
    case 0: hardcodedL();  break;
    case 1: hardcodedR(); break;
    case 2: AIMODE(); break;
  }
}

