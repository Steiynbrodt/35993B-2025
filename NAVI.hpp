
// VEX Pfadnavigation mit A*, Stuck-Recovery, Live-Repathing, Logging, Hindernisrahmen und Rückwärtsfahrt + Liveausgabe
#include "vex.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>
#include <unordered_set>
#include <fstream>
#include <limits>
#include <utility>
#include <map>
#include <time.h>
#include <string>
using namespace vex;
bool enableLearning = true; // Learning aktivieren oder deaktivieren
double initialHeadingOffset = 0.0;
const double minTurn    = 0.2;   // Mindest-% zum Überwinden von Stiction
 const double maxTurn    = 10.0;  // Max-% für schnelles Drehen
double normalize360(double angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

void calibrateINSFromGPS() {
  double gpsHeading = GPS17.heading();  // z.B. 132°
  double insHeading = INS.heading(degrees); // z.B. -48°
  if (gpsHeading < 0) gpsHeading += 360;
  if (insHeading < 0) insHeading += 360;
  initialHeadingOffset = gpsHeading - insHeading;
}

double getFusedHeading360() {
  double h = INS.heading(degrees) + initialHeadingOffset;
  return normalize360(h);
}


// --- Supervised Feedback Logging ---
// Logs the outcome and feedback of each autonomous run to run_feedback.csv for offline analysis and supervised learning.
// Parameters:
//   success         - whether the run reached the goal (1) or not (0)
//   timeTaken       - total time taken for the run (seconds)
//   replans         - number of times the bot replanned its path
//   stucks          - number of times the bot detected it was stuck
//   finalDeviation  - distance from goal at end of run (mm)
//   operatorRating  - user/driver rating (1-5) for the run
//   notes           - any additional notes or comments
void logRunFeedback(bool success, double timeTaken, int replans, int stucks, double finalDeviation, int operatorRating, const std::string& notes) {
    FILE* logFile = fopen("run_feedback.csv", "a");
    if (logFile) {
        long now = (long)::time(0); // Unix timestamp
        fprintf(logFile, "%ld,%d,%.2f,%d,%d,%.2f,%d,%s\n",
            now, success, timeTaken, replans, stucks, finalDeviation, operatorRating, notes.c_str());
        fclose(logFile);
    }
}

// Gibt eine gedämpfte Drehstärke zurück basierend auf Winkelabweichung
double taperOutput(double angleError, double maxOutput, double minOutput, double taperRange = 90.0) {
  double absError = fabs(angleError);
  if (absError >= taperRange) return maxOutput * (angleError < 0 ? -1 : 1);
  if (absError <= 5.0) return 0.0; // DEADZONE
  double scaled = (absError / taperRange) * (maxOutput - minOutput) + minOutput;
  return scaled * (angleError < 0 ? -1 : 1);
}
// Liefert den kürzesten Drehrichtungs-Fehler zwischen zwei Winkeln (immer -180° bis +180°)
double shortestAngleDiff(double from, double to) {
  double diff = to - from;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}
void rotateToHeading(double targetHeadingDeg, double toleranceDeg = 5.0) {
  const int maxAttempts = 40;
  int attempts = 0;

  // Fixiere Drehrichtung beim Start
  double currentHeading = getFusedHeading360();
  double initialError = shortestAngleDiff(currentHeading, targetHeadingDeg);
  int turnDir = (initialError >= 0) ? 1 : -1;

  while (++attempts <= maxAttempts) {
    currentHeading = getFusedHeading360();
    double error = shortestAngleDiff(currentHeading, targetHeadingDeg);

    if (fabs(error) <= toleranceDeg) break;

    // Stärke nach Distanz tappen, keine Richtungswechsel mehr
    double strength = taperOutput(fabs(error), maxTurn, minTurn) * turnDir;

    LeftDrivetrain.spin(reverse, strength, percent);
    RightDrivetrain.spin(forward, strength, percent);

    printf("  ⟳ Rotating: curr=%.1f°, target=%.1f°, error=%.1f°, turn=%.1f%%\n",
           currentHeading, targetHeadingDeg, error, strength);

    wait(100, msec);
  }

  FullDrivetrain.stop();
}

const double FIELD_SIZE_MM = 3600.0;
const int GRID_SIZE = 73;
const int OFFSET = GRID_SIZE / 2;
const double CELL_SIZE = FIELD_SIZE_MM / GRID_SIZE;
double waypointTolerance = 30.0;
double deviationThreshold = 50.0; // mm, adaptive below
double headingTolerance = 10;   // degrees
bool pathSmoothingEnabled = true;
int startX, startY;
int goalX, goalY;
bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = {{false}};
std::vector<std::pair<int, int>> pathWaypoints;
std::vector<std::pair<int, int>> customObstacles;

// Logs the current GPS position and heading to the SD card (gps_log.txt) if available.
void logGPSData() {
  if (!Brain.SDcard.isInserted()) {
    Brain.Screen.print("No SD card inserted!");
    return;
  }

  FILE* logFile = fopen("gps_log.txt", "a");
  if (!logFile) return;

  for (size_t j = 0; j < 99999999; j++){

   
    double x = GPS17.xPosition(mm);
    double y = GPS17.yPosition(mm);
    double heading = getFusedHeading360();
    fprintf(logFile, "%.2f,%.2f,%.2f,%.2f\n", Brain.timer(sec), x, y, heading);

                // force flush buffer to SD (optional but safe)
    task::sleep(100);            // log every 100ms = 10 Hz
  }

  fclose(logFile);               // 🔁 Note: unreachable here, but fine if refactored
}


// Clamps a value between minVal and maxVal.
double clamp(double value, double minVal, double maxVal) {
  return std::max(minVal, std::min(maxVal, value));
}

// Normalizes an angle to the range [0, 360).

// Marks a grid cell and its 8 neighbors as non-walkable (obstacle with margin).
void addObstacleWithMargin(int x, int y) {
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      int nx = x + dx;
      int ny = y + dy;
      if (nx >= -OFFSET && nx <= OFFSET && ny >= -OFFSET && ny <= OFFSET)
        walkable[ny + OFFSET][nx + OFFSET] = false;
    }
  }
}

// Converts a grid coordinate to millimeters.
double gridToMM(int gridVal) {
  return gridVal * CELL_SIZE;
}

// Converts a millimeter position to a grid coordinate, clamped to grid bounds.
int toGridCoord(double mm) {
  return clamp(static_cast<int>(round(mm / CELL_SIZE)), -OFFSET, OFFSET);
}

// Updates the start grid coordinates from the current GPS position.
void updateStartPositionFromGPS() {
  double gpsX = GPS17.xPosition(mm);
  double gpsY = GPS17.yPosition(mm);
  if (!std::isnan(gpsX) && !std::isnan(gpsY)) {
    startX = toGridCoord(gpsX);
    startY = toGridCoord(gpsY);
    task::sleep(100);
  }
}

struct Node {
  int x, y;
  int gCost, hCost;
  Node* parent;
  int fCost() const { return gCost + hCost; }
};

Node* nodes[GRID_SIZE][GRID_SIZE];

int heuristic(int x1, int y1, int x2, int y2) {
  return 10 * (abs(x1 - x2) + abs(y1 - y2));
}

// Simplifies a path by removing intermediate waypoints that are collinear (including diagonals).
void simplifyPath(std::vector<std::pair<int, int>>& waypoints) {
  if (waypoints.size() < 3) return;
  std::vector<std::pair<int, int>> simplified;
  simplified.push_back(waypoints[0]);
  int dxPrev = waypoints[1].first - waypoints[0].first;
  int dyPrev = waypoints[1].second - waypoints[0].second;
  int lenPrev = std::max(abs(dxPrev), abs(dyPrev));
  if (lenPrev != 0) { dxPrev /= lenPrev; dyPrev /= lenPrev; }
  for (size_t i = 1; i < waypoints.size() - 1; ++i) {
    int dx = waypoints[i+1].first - waypoints[i].first;
    int dy = waypoints[i+1].second - waypoints[i].second;
    int len = std::max(abs(dx), abs(dy));
    if (len != 0) { dx /= len; dy /= len; }
    if (dx != dxPrev || dy != dyPrev) {
      simplified.push_back(waypoints[i]);
      dxPrev = dx;
      dyPrev = dy;
    }
  }
  simplified.push_back(waypoints.back());
  waypoints = simplified;
}

// Runs A* to find a path from start to goal, fills pathWaypoints, and simplifies the path.
void calculatePath() {
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      nodes[y][x] = new Node{x - OFFSET, y - OFFSET, 9999, 9999, nullptr};
      isPath[y][x] = false;
    }
  }

  pathWaypoints.clear();
  Node* start = nodes[startY + OFFSET][startX + OFFSET];
  Node* goal = nodes[goalY + OFFSET][goalX + OFFSET];
  start->gCost = 0;
  start->hCost = heuristic(startX, startY, goalX, goalY);

  std::vector<Node*> openSet = { start };
  bool closedSet[GRID_SIZE][GRID_SIZE] = { false };
  int dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
  int dy[8] = { 0, -1, -1, -1, 0, 1, 1, 1};

  while (!openSet.empty()) {
    Node* current = *std::min_element(openSet.begin(), openSet.end(), [](Node* a, Node* b) {
      return a->fCost() < b->fCost();
    });
    openSet.erase(std::remove(openSet.begin(), openSet.end(), current), openSet.end());
    closedSet[current->y + OFFSET][current->x + OFFSET] = true;

    if (current == goal) {
      Node* p = goal;
      while (p != start && p != nullptr) {
        isPath[p->y + OFFSET][p->x + OFFSET] = true;
        pathWaypoints.push_back({p->x, p->y});
        p = p->parent;
      }
      std::reverse(pathWaypoints.begin(), pathWaypoints.end());
      simplifyPath(pathWaypoints);
      break;
    }

    for (int d = 0; d < 8; d++) {
      int nx = current->x + dx[d];
      int ny = current->y + dy[d];
      if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;
      if (!walkable[ny + OFFSET][nx + OFFSET] || closedSet[ny + OFFSET][nx + OFFSET]) continue;

      int moveCost = (dx[d] == 0 || dy[d] == 0) ? 10 : 14;
      int tentativeG = current->gCost + moveCost;
      Node* neighbor = nodes[ny + OFFSET][nx + OFFSET];

      if (tentativeG < neighbor->gCost) {
        neighbor->gCost = tentativeG;
        neighbor->hCost = heuristic(nx, ny, goalX, goalY);
        neighbor->parent = current;
        openSet.push_back(neighbor);
      }
    }
  }

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      delete nodes[y][x];
      nodes[y][x] = nullptr;
    }
  }
}

// Drives to a target position (mm) with stuck detection and recovery. Returns true if target reached, false if stuck.
bool driveToWithRecovery(double targetXmm, double targetYmm) {
  const double tolerance = 30.0;
  const double maxSpeed = 10.0;

  double lastDist = 9999.0;
  int stuckCounter = 0;
  
  while (true) {
    double currentX = GPS17.xPosition(mm);
    double currentY = GPS17.yPosition(mm);
    double dx = targetXmm - currentX;
    double dy = targetYmm - currentY;
    double dist = sqrt(dx * dx + dy * dy);

    logGPSData();

    if (dist <= tolerance) {
      FullDrivetrain.stop();
      printf("[Drive] Arrived at target\n");
      return true;
    }

    double heading = getFusedHeading360();
    double angleToTarget = atan2(dy, dx) * 180.0 / M_PI;
    if (angleToTarget < 0) angleToTarget += 360.0;

    double angleError = shortestAngleDiff(heading, angleToTarget);
    bool reverse = fabs(angleError) > 90.0;

    
    double fixedHeading = angleToTarget;
    if (reverse)
    fixedHeading = normalize360(fixedHeading + 180.0);

    rotateToHeading(fixedHeading);
    double driveSpeed = reverse ? -maxSpeed : maxSpeed;
    if (dist < 100.0) driveSpeed *= 0.5;
    if (dist < 50.0)  driveSpeed *= 0.3;

    LeftDrivetrain.spin(forward, driveSpeed, percent);
    RightDrivetrain.spin(forward, driveSpeed, percent);

    printf("[Drive] → Target: %.0f/%.0f mm | Pos: %.0f/%.0f mm | Δ%.0f mm | Heading %.1f°\n",
           targetXmm, targetYmm, currentX, currentY, dist, heading);

    if (fabs(dist - lastDist) < 1.0)
      stuckCounter++;
    else
      stuckCounter = 0;

    if (stuckCounter > 15) {
      int cx = toGridCoord(currentX);
      int cy = toGridCoord(currentY);
      addObstacleWithMargin(cx, cy);
      printf("[WARN] Movement stalled. Replanning...\n");
      FullDrivetrain.stop();
      return false;
    }

    lastDist = dist;
    wait(50, msec);
  }
}
// Configurable parameters
// 1. Adaptive Deviation Threshold, 2. Waypoint Lookahead, 3. Path Smoothing
// 7. Heading Correction at Waypoints, 8. Goal Overshoot Handling, 9. Parameter Tuning, 10. (Unit tests: see test.hpp)


// Path smoothing using simple line-of-sight (skips waypoints if direct path is clear)
// Checks if a straight line between two grid points is clear of obstacles (Bresenham's algorithm).
bool isLineClear(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  while (x0 != x1 || y0 != y1) {
    if (!walkable[y0 + OFFSET][x0 + OFFSET]) return false;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx)  { err += dx; y0 += sy; }
  }
  return true;
}

// Further smooths a path by skipping waypoints if a direct line is clear.
void smoothPath(std::vector<std::pair<int, int>>& waypoints) {
  if (waypoints.size() < 3) return;
  std::vector<std::pair<int, int>> smoothed;
  size_t i = 0;
  while (i < waypoints.size()) {
    smoothed.push_back(waypoints[i]);
    size_t j = waypoints.size() - 1;
    for (; j > i + 1; --j) {
      if (isLineClear(waypoints[i].first, waypoints[i].second, waypoints[j].first, waypoints[j].second)) {
        break;
      }
    }
    i = j;
  }
  if (smoothed.back() != waypoints.back()) smoothed.push_back(waypoints.back());
  waypoints = smoothed;
}

// Follows the current path, stopping at each direction-changing waypoint to check/correct position and heading.
// Handles deviation, heading correction, and goal overshoot.
void followPath() {
  

  double integral = 0.0;
  double lastError = 0.0;
  double currentHeading;
  double headingError;
  double integralLimit = 100.0;

  int initialStartX = startX;
  int initialStartY = startY;
  bool reached = false;
  int waypointIdx = 0;
  

  while (!reached && waypointIdx < pathWaypoints.size()) {
    auto& wp = pathWaypoints[waypointIdx];
    double targetX = gridToMM(wp.first);
    double targetY = gridToMM(wp.second);
    // 1. Adaptive deviation threshold: smaller near goal
    double adaptiveDeviation = deviationThreshold;
    if (waypointIdx == pathWaypoints.size() - 1) adaptiveDeviation = deviationThreshold * 0.5;
    // 2. Waypoint lookahead: skip to next if close and heading is good
    double currentX = GPS17.xPosition(mm);
    double currentY = GPS17.yPosition(mm);
    double currentHeading = getFusedHeading360();
    double deviation = sqrt((currentX - targetX) * (currentX - targetX) + (currentY - targetY) * (currentY - targetY));
    double desiredHeading = 0.0;
    if (waypointIdx + 1 < pathWaypoints.size()) {
      double nextX = gridToMM(pathWaypoints[waypointIdx + 1].first);
      double nextY = gridToMM(pathWaypoints[waypointIdx + 1].second);
      desiredHeading = normalize360(atan2(nextY - currentY, nextX - currentX) * 180.0 / M_PI);
    } else if (waypointIdx > 0) {
      double prevX = gridToMM(pathWaypoints[waypointIdx - 1].first);
      double prevY = gridToMM(pathWaypoints[waypointIdx - 1].second);
      desiredHeading = normalize360(atan2(targetY - prevY, targetX - prevX) * 180.0 / M_PI);
    } else {
      desiredHeading = currentHeading;
    }
    double headingError = normalize360(desiredHeading - currentHeading);
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    printf("[Missile] At waypoint %d: Target(%.1f, %.1f) Actual(%.1f, %.1f) Deviation: %.1f mm Heading: %.1f° (desired %.1f° error %.1f°)\n", waypointIdx, targetX, targetY, currentX, currentY, deviation, currentHeading, desiredHeading, headingError);
        // 7. Heading correction at waypoints
    {
      // Berechne aktuellen und gewünschten Heading in [0,360)
      double currentHeading =getFusedHeading360();
      if (currentHeading < 0) currentHeading += 360.0;
      double desiredHeadingDeg = desiredHeading;  // aus deinem obigen Code

      // Initialen Fehler im Bereich -180…+180 berechnen
      

     
  
  if (fabs(headingError) > headingTolerance) {
  rotateToHeading(desiredHeading);
  printf("[Missile] Heading error too large (%.1f°), correcting heading...\n", headingError);
  }
       

// 3. Path smoothing is handled in calculatePath
    // If deviation is too large, correct by replanning from here
    if (deviation > adaptiveDeviation) {
      printf("[Missile] Deviation too large, replanning from current position.\n");
      updateStartPositionFromGPS();
      calculatePath();
      waypointIdx = 0;
      continue;
    }
    // Otherwise, proceed to next waypoint
    waypointIdx++;
    // 8. Goal overshoot handling
    if (waypointIdx == pathWaypoints.size()) {
      double dx = targetX - currentX;
      double dy = targetY - currentY;
      double dist = sqrt(dx * dx + dy * dy);
      if (dist <= waypointTolerance) {
        reached = true;
      } else if (dist > waypointTolerance * 2) {
        printf("[Missile] Overshot goal, correcting...\n");
        updateStartPositionFromGPS();
        calculatePath();
        waypointIdx = 0;
      }
    }
  }
  while (true) {
    printf("[Path] Last target: X=%.2f mm, Y=%.2f mm\n", gridToMM(goalX), gridToMM(goalY));
    printf("[GPS ] Current:     X=%.2f mm, Y=%.2f mm, Heading=%.1f°\n",GPS17.xPosition(mm),getFusedHeading360());
    task::sleep(500);
  }
}
}
// 9. Parameter tuning: expose parameters above for easy adjustment
// 10. Unit tests: see test.hpp for path logic tests
void applyLearnedTuning() {
  if (!enableLearning) return;

  FILE* f = fopen("run_feedback.csv", "r");
  if (!f) return;

  char line[256];
  double totalDev = 0, totalReplans = 0, totalStucks = 0;
  int count = 0;

  while (fgets(line, sizeof(line), f)) {
    long ts; int success, replans, stucks, rating;
    double time, dev;
    char notes[128];
    if (sscanf(line, "%ld,%d,%lf,%d,%d,%lf,%d,%127[^\n]",
               &ts, &success, &time, &replans, &stucks, &dev, &rating, notes) == 8) {
      totalDev += dev;
      totalReplans += replans;
      totalStucks += stucks;
      count++;
    }
  }
  fclose(f);

  if (count == 0) return;

  double avgDev = totalDev / count;
  double avgReplans = totalReplans / count;
  double avgStucks = totalStucks / count;

  // Parameter anpassen
  if (avgDev > 60) deviationThreshold *= 1.2;
  else if (avgDev < 30) deviationThreshold *= 0.8;

  if (avgReplans > 3) headingTolerance += 2;
  else if (avgReplans < 1) headingTolerance = std::max(5.0, headingTolerance - 2.0);

  if (avgStucks > 4) headingTolerance += 1;
}

// Main entry point: plans and follows a path to the target (mm), logs run feedback for supervised learning.
void NAVI(double targetXmm, double targetYmm) {
  

 INS.calibrate(0);
task::sleep(2000);  // wichtig!
calibrateINSFromGPS();

 
  goalX = clamp(toGridCoord(targetXmm), -OFFSET, OFFSET);
  goalY = clamp(toGridCoord(targetYmm), -OFFSET, OFFSET);

  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      walkable[y][x] = true;

  for (int i = -36; i <= 36; i++) {
    addObstacleWithMargin(i, -36);
    addObstacleWithMargin(i, 36);
    addObstacleWithMargin(-36, i);
    addObstacleWithMargin(36, i);
  }

  // Only get the position at the start
  updateStartPositionFromGPS();
  calculatePath();
  followPath();

  // --- Feedback logging (replace with real values as needed) ---
  bool success = true; // Set based on whether goal was reached
  double timeTaken = 0; // Measure time for the run
  int replans = 0;      // Count replans during followPath
  int stucks = 0;       // Count stuck events
  double finalDeviation = 0; // Compute at end
  int operatorRating = 5;    // Prompt operator for rating (1-5)
  std::string notes = "Auto log"; // Optionally prompt for notes

  logRunFeedback(success, timeTaken, replans, stucks, finalDeviation, operatorRating, notes);
}
