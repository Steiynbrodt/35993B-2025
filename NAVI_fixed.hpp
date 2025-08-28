


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
double timeTaken; 
 int replans;
  int stucks;
  bool enableLearning = true; // Learning Toggle
static double initialHeadingOffset;
const double minTurn    = 3;  
 const double maxTurn    = 6; 
using namespace vex;
 

double normalize360(double angle) {
    angle = std::fmod(angle, 360.0);  // now in –360…+360
    if (angle < 0) angle += 360.0;     // shift negatives into 0…360
    return angle;                     // final in [0,360)
}

double shortestAngleDiff(double from, double to) {
  double diff = to - from;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}


/**
 * Call once at startup to align INS to GPS.
 * Reads both sensors, computes the minimal signed difference (–180…+180)
 * and stores it in initialHeadingOffset.
 */
void calibrateINSFromGPS() {
  // normalize everything to [0,360)
  double gpsH = normalize360(GPS17.heading(degrees));
  double insH = normalize360(INS.heading(degrees));

  // offset = (gps - ins) wrapped to [-180,180]
  initialHeadingOffset = shortestAngleDiff(insH, gpsH);

  double fusedH = normalize360(insH + initialHeadingOffset);
  printf("[Calib] GPS=%.1f°, INS=%.1f°, Offset=%.1f° → Fused=%.1f°\n",
         gpsH, insH, initialHeadingOffset, fusedH);
}

// Always return fused heading in [0,360)
double getFusedHeading360() {
  return normalize360(INS.heading(degrees) + initialHeadingOffset);
}/*
 * Returns the “fused” heading in [0,360):
 *   INS.heading + the offset you computed at startup.
 */

double yawOffset = 180;

double INSS;

double getYaw()
{
    INSS = INS.heading();
    double yaw = INSS - 180 - yawOffset;
    if (yaw < -180)
    {
        yaw += 360;
    }
    else if (yaw > 180)
    {
        yaw -= 360;
    }
    
    return yaw;
}
void turnToYaw(double targetYaw) {
  // normalize desired yaw to [0,360)
  targetYaw = normalize360(targetYaw);

  const double tolDeg = 1.5;
  const int dt_ms = 15;
  const int timeout_ms = 2500;
  int elapsed = 0;

  while (true) {
    // use fused heading so the offset applies everywhere
    double current = getFusedHeading360();
    double err = shortestAngleDiff(current, targetYaw);

    if (fabs(err) <= tolDeg) break;

    // smooth, bounded turn power
    double speed = std::max(6.0, 20.0 * (fabs(err) / 90.0));
    if (fabs(err) < 10.0) speed = std::max(2.0, speed * 0.1);

    int dir = (err > 0) ? 1 : -1;
    LeftDrivetrain.spin(forward,  speed * dir, percent);
    RightDrivetrain.spin(forward, -speed * dir, percent);

    wait(dt_ms, msec);
    elapsed += dt_ms;
    if (elapsed >= timeout_ms) break;  // fail-safe: don't block forever
  }

  LeftDrivetrain.stop();
  RightDrivetrain.stop();
}



/*static bool rotateToHeading(double targetDeg, double toleranceDeg = 5.0,int    maxLoops     = 80){
    

    printf("[Rotate/GPS] Turning to %.1f° ±%.1f°", targetDeg, toleranceDeg);
    for (int i = 1; i <= maxLoops; ++i) {
        double curr = getFusedHeading360();
        double error = curr - targetDeg;
        if (error > 180.0)  error -= 360.0;
        if (error < -180.0) error += 360.0;

        printf("[Rotate/GPS] #%2d: curr=%.1f°, err=%+.1f°", i, curr, error);
        if (fabs(error) <= toleranceDeg) {
            FullDrivetrain.stop();
            printf("[Rotate/GPS] Arrived at %.1f°", curr);
            return true;
        }

        double power = taperOutput(error, maxTurn, minTurn);
        if (power > 0) {
            LeftDrivetrain.spin (forward,  power, percent);
            RightDrivetrain.spin(reverse,  power, percent);
        } else {
            LeftDrivetrain.spin (reverse, -power, percent);
            RightDrivetrain.spin (forward, -power, percent);
        }
        task::sleep(50);
    }

    FullDrivetrain.stop();
    printf("[Rotate/GPS] TIMEOUT at target=%.1f°", targetDeg);
    return false;
}*/
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
static const double kP         = 0.5;   // proportional gain (adjust as needed)
static const double maxSpeed   = 50.0;  // maximum motor speed (%)
static const double tolerance  = 1.0;   // how close is “close enough” (°)
// Gibt eine gedämpfte Drehstärke zurück basierend auf Winkelabweichung
double taperOutput(double angleError, double maxOutput, double minOutput, double taperRange = 90.0) {
  double absError = fabs(angleError);
  if (absError >= taperRange) return maxOutput * (angleError < 0 ? -1 : 1);
  if (absError <= 5.0) return 0.0; // DEADZONE
  double scaled = (absError / taperRange) * (maxOutput - minOutput) + minOutput;
  return scaled * (angleError < 0 ? -1 : 1);
}
// Liefert den kürzesten Drehrichtungs-Fehler zwischen zwei Winkeln (immer -180° bis +180°)
/*void rotateToHeading(double target) {
  // Make sure target is 0…359.999
  target = normalize360(target);

  // Loop until within tolerance
  while (true) {
    double current = normalize360( INS.heading() );  
    // INS.heading() returns 0–360° in degrees by default :contentReference[oaicite:0]{index=0} 
    double error   = shortestAngleDiff(current, target);
    
    // If we’re close enough, stop
    if (fabs(error) <= tolerance) break;

    // Proportional control: command speed = kP * error, clamped to maxSpeed
    double speed = kP * error;
    speed = (speed > 0)
              ? std::min(speed,  maxSpeed)
              : std::max(speed, -maxSpeed);

    // Spin motors: left ← fwd when error>0, right → rev (and vice versa)
    LeftDrivetrain.spin(  directionType::fwd,  speed, velocityUnits::pct );
    RightDrivetrain.spin( directionType::rev,  speed, velocityUnits::pct );

    task::sleep(10);  // small delay to avoid hogging CPU
  }

  // Brake to hold heading
  LeftDrivetrain.stop(  brakeType::brake );
  RightDrivetrain.stop( brakeType::brake );
}*/
double distancesqrt(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}


double targetX;
double targetY;
double currentY;
double currentX;


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
  if (!Brain.SDcard.isInserted()) return;
  FILE* logFile = fopen("gps_log.txt", "a");
  if (!logFile) return;
  double x = GPS17.xPosition(mm);
  double y = GPS17.yPosition(mm);
  double heading = INS.heading(deg);
  fprintf(logFile, "%.2f,%.2f,%.2f,%.2f\n", Brain.timer(sec), x, y, heading);
  fclose(logFile);
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

  if (std::isnan(gpsX) || std::isnan(gpsY)) {
    printf("[ERROR] GPS returned NaN: x=%.2f, y=%.2f\n", gpsX, gpsY);
    return;
  }

  startX = toGridCoord(gpsX);
  startY = toGridCoord(gpsY);
  if (startX < -OFFSET || startX > OFFSET || startY < -OFFSET || startY > OFFSET) {
    printf("[ERROR] Computed start grid (%d,%d) is out of bounds!\n", startX, startY);
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
  // --- FIX: Ensure walkable grid is initialized ---
  for (int y = 0; y < GRID_SIZE; ++y)
    for (int x = 0; x < GRID_SIZE; ++x)
      walkable[y][x] = true;

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
  if (goal->parent == nullptr) {
    printf("[A*] No path found from (%d,%d) to (%d,%d)\n", startX, startY, goalX, goalY);
    pathWaypoints.clear();
    return;
  }
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
    if (reverse) fixedHeading = normalize360(fixedHeading + 180.0);
    turnToYaw(fixedHeading);

    turnToYaw(fixedHeading);
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
      stuckCounter++;
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

thread gpsLoggerThread([](){
  while(true) {
    logGPSData();
    task::sleep(100);

    printf("[Path] Last target: X=%.2f mm, Y=%.2f mm\n",
           gridToMM(goalX), gridToMM(goalY));

    printf("[GPS ] Current: X=%.2f mm, Y=%.2f mm, Heading=%.1f°\n",
           GPS17.xPosition(mm), GPS17.yPosition(mm), getFusedHeading360());

    printf("[Fusion] GPS %.1f°, INS %.1f°, Offset %.1f°, Result %.1f°\n",
           normalize360(GPS17.heading(degrees)),
           normalize360(INS.heading(degrees)),
           initialHeadingOffset,
           getFusedHeading360());

    task::sleep(500);
  }
});

// Follows the current path, stopping at each direction-changing waypoint to check/correct position and heading.
// Handles deviation, heading correction, and goal overshoot.
void followPath() {
  // --- persistent helpers for rate limiting & progress ---
  static double lastReplanTime = 0.0;
  static double lastX = -9999.0, lastY = -9999.0;
  static int noProgressCount = 0;

  bool reached = false;
  size_t waypointIdx = 0;

  if (pathWaypoints.empty()) {
    printf("[Follow] No path to follow.\n");
    return;
  }

  printf("[Follow] Starting; waypoints=%zu\n", pathWaypoints.size());

  while (!reached && waypointIdx < pathWaypoints.size()) {
    // --- 0) Targets & pose ---
    const double targetX = gridToMM(pathWaypoints[waypointIdx].first);
    const double targetY = gridToMM(pathWaypoints[waypointIdx].second);
    const double currentX = GPS17.xPosition(mm);
    const double currentY = GPS17.yPosition(mm);

    // --- 1) Progress check (stall detection) ---
    if (lastX > -9000.0 && lastY > -9000.0) {
      const double dxProg = currentX - lastX;
      const double dyProg = currentY - lastY;
      if (hypot(dxProg, dyProg) < 20.0) {
        noProgressCount++;
      } else {
        noProgressCount = 0;
      }
    }
    lastX = currentX;
    lastY = currentY;

    if (noProgressCount >= 3) {
      printf("[Follow] No progress (%d) → driveToWithRecovery()\n", noProgressCount);
      LeftDrivetrain.stop();
      RightDrivetrain.stop();

      const bool recovered = driveToWithRecovery(targetX, targetY);
      if (!recovered) {
        stucks++;
        // Mark obstacle (assumed inside recovery), re-seed from GPS, and REPLAN
        updateStartPositionFromGPS();
        calculatePath();
        replans++;
        waypointIdx = 0;
        noProgressCount = 0;
        printf("[Follow] Recovery failed → replanned path with %zu waypoints\n", pathWaypoints.size());
        continue;  // follow the NEW path
      } else {
        noProgressCount = 0;
      }
    }

    // --- 2) Distance to current waypoint ---
    const double dx = targetX - currentX;
    const double dy = targetY - currentY;
    const double deviation = hypot(dx, dy);

    // --- 3) Adaptive deviation threshold (tighter at final WP) ---
    double adaptiveDeviation = deviationThreshold;
    if (waypointIdx == pathWaypoints.size() - 1) {
      adaptiveDeviation *= 0.5;
    }

    // --- 4) Desired heading to this waypoint ---
    double desiredHeading = atan2(dy, dx) * 180.0 / M_PI;
    if (desiredHeading < 0) desiredHeading += 360.0;

    // Use fused heading consistently
    double currentHeading = getFusedHeading360();
    double headingError = shortestAngleDiff(currentHeading, desiredHeading);

    printf("[Follow] WP %zu  Target(%.1f,%.1f)  Pos(%.1f,%.1f)  Dev=%.1fmm  Head=%.1f°→%.1f°  Err=%.1f°\n",
           waypointIdx, targetX, targetY, currentX, currentY, deviation,
           currentHeading, desiredHeading, headingError);

    // --- 5) Heading correction if too large (cooperative turn) ---
    if (fabs(headingError) > headingTolerance) {
      turnToYaw(desiredHeading);               // must be the cooperative version
      currentHeading = getFusedHeading360();   // refresh after turn
      headingError = shortestAngleDiff(currentHeading, desiredHeading);
      printf("[Follow] Corrected heading → now err=%.1f°\n", headingError);
    }

    // --- 6) Replan if far off-path (rate-limited) ---
    const double now = Brain.timer(sec);
    if (deviation > adaptiveDeviation && fabs(headingError) < 15.0) {
      if (now - lastReplanTime < 2.0) {
        printf("[Follow] Replan wanted but rate-limited (%.2fs)\n", now - lastReplanTime);
        // IMPORTANT: do NOT advance waypoint here; just wait a cycle
      } else {
        lastReplanTime = now;
        LeftDrivetrain.stop();
        RightDrivetrain.stop();
        updateStartPositionFromGPS();
        calculatePath();
        replans++;
        waypointIdx = 0;
        printf("[Follow] Deviation too large → replanned path with %zu waypoints\n", pathWaypoints.size());
        continue;
      }
    }

    // --- 7) Waypoint advancement (only if close enough) ---
    if (deviation <= waypointTolerance) {
      printf("[Follow] Reached WP %zu (%.1f, %.1f)  dist=%.1fmm\n",
             waypointIdx, targetX, targetY, deviation);
      waypointIdx++;
      continue;
    }

    // --- 8) End-state handling when last WP is "near but not quite" ---
    if (waypointIdx == pathWaypoints.size() - 1) {
      if (deviation <= waypointTolerance) {
        reached = true;
        break;
      } else if (deviation > waypointTolerance * 2.0) {
        printf("[Follow] Overshot goal, correcting...\n");
        updateStartPositionFromGPS();
        calculatePath();
        replans++;
        waypointIdx = 0;
        continue;
      }
    }

    // --- 9) Yield so other logic can run ---
    wait(20, msec);
  }

  LeftDrivetrain.stop();
  RightDrivetrain.stop();
  printf("[Follow] Completed. Waypoints traversed=%zu\n", pathWaypoints.size());
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
  double startTime = Brain.timer(sec);

  
 // wichtig!
  
 
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
  double timeTaken = Brain.timer(sec) - startTime;

  // --- Feedback logging (replace with real values as needed) ---
  

  double finalX = GPS17.xPosition(mm);
  double finalY = GPS17.yPosition(mm);
  double finalDeviation = sqrt(pow(finalX - targetXmm, 2) + pow(finalY - targetYmm, 2));
  bool success = finalDeviation <= waypointTolerance;

  int operatorRating = 5;
  std::string notes = "Auto log";

logRunFeedback(success, timeTaken, replans, stucks, finalDeviation, operatorRating, notes);

}
