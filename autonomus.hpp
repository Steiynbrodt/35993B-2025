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

    // Build your 3.6m x 3.6m field, 50mm cells, obstacles, margins, inflation, etc.
    Field field(3600, 3600, 50.0);
    auto& grid = field.grid();

    // e.g. edges + inflation:
    field.addEdgeMargin(100.0);
    field.inflateByRadius(150.0);

    // 1) Read start from GPS
    StartPoseCell sp = getStartFromGPS(field);
    if (!sp.valid) { Brain.Screen.print("GPS start invalid"); return; }
    grid.setCell(sp.sx, sp.sy, Cellstate::START);

    // 2) Choose/set a goal cell (example: near top right)
    //int gx = grid.cols() - 3, gy = grid.rows() - 3;
    int gx = 10; int gy = 10 ;
    
    grid.setCell(gx, gy, Cellstate::GOAL);
    // Debug: show grid coords
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("S:(%d,%d) G:(%d,%d)", sp.sx, sp.sy, gx, gy);

      // Convert to mm to see world positions
      double sXmm, sYmm, gXmm, gYmm;
      field.cellToMmCenter(sp.sx, sp.sy, sXmm, sYmm);
      field.cellToMmCenter(gx, gy, gXmm, gYmm);

      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("Smm:(%.0f,%.0f)", sXmm, sYmm);

      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("Gmm:(%.0f,%.0f)", gXmm, gYmm);

    // 3) A* path
    auto pathPairs = pathfind::astar(grid, sp.sx, sp.sy, gx, gy);      // :contentReference[oaicite:11]{index=11}
    if (pathPairs.empty()) { Brain.Screen.print("No path"); return; }

    // 4) Convert to CellXY for navigatePath
    std::vector<CellXY> path; path.reserve(pathPairs.size());
    for (auto& p : pathPairs) path.push_back({p.first, p.second});

    // 5) Drive it (your turn & drive primitives already exist)         // :contentReference[oaicite:12]{index=12} :contentReference[oaicite:13]{index=13}
    navigatePath(path, field.cellMm(), sp.headingDeg, 25.0, turnStepDeg, driveStraightMm);

    // (optional) paint the path for debugging
    pathfind::paintPath(grid, pathPairs);                              // :contentReference[oaicite:14]{index=14}
} 

void autonomous() {
  switch (autonMode) {
    case 0: hardcodedL();  break;
    case 1: hardcodedR(); break;
    case 2: AIMODE(); break;
  }
}

