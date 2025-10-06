// Testfile: tests.cpp
#include "vex.h"
 #ifndef NAVI_TEST_HPP
#define NAVI_TEST_HPP

#include <vector>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <string>
#include <iostream>
/*
extern const int GRID_SIZE;
extern const int OFFSET;
extern bool walkable[73][73];
extern std::vector<std::pair<int, int>> pathWaypoints;
extern int goalX, goalY, startX, startY;

// Wrapper für Testausgaben
void printTestResult(const std::string& name, bool success) {
  printf("Test %-30s [%s]\n", name.c_str(), success ? "OK" : "FAILED");
}

// 1. Initialisierung des Grids
void test_gridInitialization() {
  bool allWalkable = true;
  for (int y = 0; y < GRID_SIZE; ++y) {
    for (int x = 0; x < GRID_SIZE; ++x) {
      if (!walkable[y][x]) {
        allWalkable = false;
        break;
      }
    }
  }
  printTestResult("Grid walkable init", allWalkable);
}

// 2. A*-Pfadfindung (Start/Ende gesetzt, keine Hindernisse)
void test_astarPathBasic(void (*calculatePath)()) {
  startX = -10; startY = -10;
  goalX = 10; goalY = 10;

  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      walkable[y][x] = true;

  calculatePath();

  bool valid = !pathWaypoints.empty();
  printTestResult("A* path basic", valid);
}

// 3. Pfadverlauf durch Hindernisse
void test_pathAvoidsObstacles(void (*calculatePath)(), void (*addObstacleWithMargin)(int, int)) {
  startX = -10; startY = -10;
  goalX = 10; goalY = 10;

  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      walkable[y][x] = true;

  for (int x = -10; x <= 10; x++) {
    addObstacleWithMargin(x, x); // Diagonale blockieren
  }

  calculatePath();

  bool valid = true;
  for (const auto& pt : pathWaypoints) {
    if (!walkable[pt.second + OFFSET][pt.first + OFFSET]) {
      valid = false;
      break;
    }
  }

  printTestResult("Path avoids obstacles", valid);
}

// 4. Pfad wird durch Glättung kürzer
void test_pathSmoothing(void (*calculatePath)(), void (*smoothPath)(std::vector<std::pair<int, int>>&)) {
  startX = -20; startY = -20;
  goalX = 20; goalY = 20;

  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      walkable[y][x] = true;

  calculatePath();
  size_t before = pathWaypoints.size();
  smoothPath(pathWaypoints);
  size_t after = pathWaypoints.size();

  printTestResult("Path smoothing reduces waypoints", after < before);
}

// 5. Zielerreichung prüfen
void test_goalReachedSimulation(double (*gridToMM)(int), double tolerance = 30.0) {
  if (pathWaypoints.empty()) {
    printTestResult("Goal reached (precondition)", false);
    return;
  }

  auto [gx, gy] = pathWaypoints.back();
  double robotX = gridToMM(gx);
  double robotY = gridToMM(gy + 1); // kleine Abweichung

  double dist = std::sqrt(std::pow(robotX - gridToMM(goalX), 2) +
                          std::pow(robotY - gridToMM(goalY), 2));

  bool reached = dist <= tolerance;
  printTestResult("Goal within tolerance", reached);
}

// Hauptfunktion zum Ausführen aller Tests
void runNavigationTests(
    void (*calculatePath)(),
    void (*addObstacleWithMargin)(int, int),
    void (*smoothPath)(std::vector<std::pair<int, int>>&),
    double (*gridToMM)(int)) {

  printf("\n=== NAVIGATION SYSTEM TESTS ===\n");
  test_gridInitialization();
  test_astarPathBasic(calculatePath);
  test_pathAvoidsObstacles(calculatePath, addObstacleWithMargin);
  test_pathSmoothing(calculatePath, smoothPath);
  test_goalReachedSimulation(gridToMM);
  printf("================================\n\n");
}
// 6. Stuck- & Replanning-Simulation (manuell getriggert durch unüberwindbares Hindernis)
void test_stuckAndReplanSimulation(void (*calculatePath)(), void (*addObstacleWithMargin)(int, int)) {
  startX = -5; startY = -5;
  goalX = 5; goalY = 5;

  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      walkable[y][x] = true;

  // Blockiere direkt den Pfad
  for (int i = -5; i <= 5; i++) {
    addObstacleWithMargin(i, i);
  }

  calculatePath();
  bool replanned = pathWaypoints.empty(); // A* findet keinen Pfad

  printTestResult("Stuck forces replan", replanned);
}

// 7. Heading-Korrekturtest (simuliert große Winkelabweichung)
void test_headingCorrection(double (*shortestAngleDiff)(double, double), double tolerance = 10.0) {
  double currentHeading = 10.0;
  double desiredHeading = 95.0;
  double error = shortestAngleDiff(currentHeading, desiredHeading);

  bool needsCorrection = fabs(error) > tolerance;
  printTestResult("Heading correction triggered", needsCorrection);
}

// 8. Logging-Validierung (prüft run_feedback.csv auf Format & Werte)
void test_loggingFormat(const char* logFile = "run_feedback.csv") {
  FILE* f = fopen(logFile, "r");
  if (!f) {
    printTestResult("Log file exists", false);
    return;
  }

  char line[256];
  bool valid = false;

  while (fgets(line, sizeof(line), f)) {
    long ts; int success, replans, stucks, rating;
    double time, dev;
    char notes[128];
    if (sscanf(line, "%ld,%d,%lf,%d,%d,%lf,%d,%127[^\n]",
               &ts, &success, &time, &replans, &stucks, &dev, &rating, notes) == 8) {
      valid = true;
      break;
    }
  }
  fclose(f);
  printTestResult("Log format valid", valid);
}

#endif
*/
