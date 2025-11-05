#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
#include <string>          // <- fehlte
#include "test.hpp"


using namespace vex;
int autonMode = 0; 
// ---- Helpers auf Datei-Ebene (nicht in einer anderen Funktion!) ----


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
void AIMODE(void) {}

void autonomous() {
  switch (autonMode) {
    case 0: hardcodedR();  break;
    case 1: hardcodedL(); break;
    case 2: AIMODE(); break;
  }
}

