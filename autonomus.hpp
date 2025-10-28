#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
#include <string>          // <- fehlte


using namespace vex;

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
  int rating = 0;
  std::string notes;
  //(void)getOperatorFeedback(rating, notes);
}

void hardcoded(void) {}
void AIMODE(void) {}

void autonomous() {
  
  wait(5, sec);
 
 
}
