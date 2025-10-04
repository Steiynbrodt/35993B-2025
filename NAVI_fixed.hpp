#include "vex.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <utility>
#include <vector>
#include <string>

using namespace vex;

// =====================
// Globals / Parameters
// =====================
double timeTaken = 0.0;
int replans = 0;
int stucks = 0;
bool enableLearning = true;

static double initialHeadingOffset = 0.0;

const double FIELD_SIZE_MM = 3600.0;
const int    GRID_SIZE     = 73;
const int    OFFSET        = GRID_SIZE / 2;        // grid index shift
const double CELL_SIZE     = FIELD_SIZE_MM / GRID_SIZE;

double waypointTolerance   = 30.0;   // mm at final point
double deviationThreshold  = 50.0;   // mm (adaptive near goal)
double headingToleranceDeg = 10.0;   // deg allowable at waypoints
bool   pathSmoothingEnabled = true;

// Start/goal in grid coords (centered at 0,0)
int startX = 0, startY = 0;
int goalX  = 0, goalY  = 0;

// Walkable grid and “isPath” display buffer
bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath [GRID_SIZE][GRID_SIZE] = {{false}};

// Current planned path (grid coords)
std::vector<std::pair<int,int>> pathWaypoints;

// =====================
// Math / Angle helpers
// =====================
static inline double norm360(double a) {
  a = std::fmod(a, 360.0);
  if (a < 0) a += 360.0;
  return a;
}
// returns shortest signed difference (to - from) in (-180, 180]
static inline double angDiff(double from, double to) {
  double d = to - from;
  while (d > 180.0)  d -= 360.0;
  while (d <= -180.0) d += 360.0;
  return d;
}
static inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
static inline int clampi(int v, int lo, int hi) {
  return std::max(lo, std::min(hi, v));
}

// =====================
// GPS/IMU fusion
// =====================
void calibrateINSFromGPS() {
  double gpsH = norm360(GPS17.heading(vex::degrees));
  double insH = norm360(INS.heading (vex::degrees));

  // Add this offset to INS to match GPS heading
  initialHeadingOffset = angDiff(insH, gpsH);

  double fused = norm360(insH + initialHeadingOffset);
  printf("[Calib] GPS=%.1f INS=%.1f Offset=%.1f → Fused=%.1f\n",
         gpsH, insH, initialHeadingOffset, fused);
}

double getFusedHeading360() {
  return norm360(INS.heading(vex::degrees) + initialHeadingOffset);
}

// =====================
// Turn helper (absolute target heading in 0–360)
// =====================
bool turnToHeadingAbs(double targetDeg, double stopTolDeg = 1.5, int maxMs = 1500) {
  targetDeg = norm360(targetDeg);
  const double minPct = 6.0, maxPct = 40.0;

  int elapsed = 0;
  while (elapsed < maxMs) {
    double curr = getFusedHeading360();
    double err  = angDiff(curr, targetDeg); // (-180,180]
    if (std::fabs(err) <= stopTolDeg) break;

    double pct = std::max(minPct, maxPct * (std::fabs(err)/90.0));
    if (std::fabs(err) < 10.0) pct = std::max(2.0, pct * 0.1);
    double sgn = (err > 0) ? +1.0 : -1.0;

    LeftDrivetrain.spin (fwd,  pct*sgn, percent);
    RightDrivetrain.spin(fwd, -pct*sgn, percent);

    vex::this_thread::sleep_for(10);
    elapsed += 10;
  }
  LeftDrivetrain.stop();
  RightDrivetrain.stop();
  return true;
}

// =====================
// Logging (SD card)
// =====================
void logGPSData() {
  if (!Brain.SDcard.isInserted()) return;
  FILE* f = fopen("gps_log.txt", "a");
  if (!f) return;
  double x = GPS17.xPosition(mm);
  double y = GPS17.yPosition(mm);
  double h = getFusedHeading360();
  fprintf(f, "%.2f,%.2f,%.2f,%.2f\n", Brain.timer(sec), x, y, h);
  fclose(f);
}

void logRunFeedback(bool success, double timeTaken, int replans, int stucks,
                    double finalDeviation, int operatorRating, const std::string& notes) {
  FILE* f = fopen("run_feedback.csv", "a");
  if (!f) return;
  long now = (long)::time(0);
  fprintf(f, "%ld,%d,%.2f,%d,%d,%.2f,%d,%s\n",
          now, success ? 1 : 0, timeTaken, replans, stucks, finalDeviation, operatorRating, notes.c_str());
  fclose(f);
}

// =====================
// Grid helpers
// =====================
double gridToMM(int g) { return g * CELL_SIZE; }

int toGridCoord(double mm) {
  return clampi((int)std::round(mm / CELL_SIZE), -OFFSET, OFFSET);
}

void addObstacleWithMargin(int gx, int gy) {
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      int nx = gx + dx;
      int ny = gy + dy;
      if (nx >= -OFFSET && nx <= OFFSET && ny >= -OFFSET && ny <= OFFSET) {
        walkable[ny + OFFSET][nx + OFFSET] = false;
      }
    }
  }
}

void updateStartPositionFromGPS() {
  double gx = GPS17.xPosition(mm);
  double gy = GPS17.yPosition(mm);
  if (std::isnan(gx) || std::isnan(gy)) {
    printf("[ERROR] GPS NaN: x=%.2f y=%.2f\n", gx, gy);
    return;
  }
  startX = toGridCoord(gx);
  startY = toGridCoord(gy);
  if (startX < -OFFSET || startX > OFFSET || startY < -OFFSET || startY > OFFSET) {
    printf("[ERROR] Start grid OOB: (%d,%d)\n", startX, startY);
  }
}

// =====================
// A* data structures
// =====================
struct Node {
  int x, y;
  int gCost, hCost;
  Node* parent;
  int fCost() const { return gCost + hCost; }
};
Node* nodes[GRID_SIZE][GRID_SIZE];

// Octile heuristic (consistent w/ diagonal move cost 14)
int heuristic(int x1,int y1,int x2,int y2){
  int dx = std::abs(x1-x2), dy = std::abs(y1-y2);
  return 10*(dx+dy) + (14-20)*std::min(dx,dy);
}

// Bresenham line-of-sight on the grid
bool isLineClear(int x0, int y0, int x1, int y1) {
  int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  while (true) {
    if (!walkable[y0 + OFFSET][x0 + OFFSET]) return false;
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 <  dx) { err += dx; y0 += sy; }
  }
  return true;
}

// Keep only direction changes
void simplifyPath(std::vector<std::pair<int,int>>& wps) {
  if (wps.size() < 3) return;
  std::vector<std::pair<int,int>> out;
  out.push_back(wps[0]);

  int dxPrev = wps[1].first - wps[0].first;
  int dyPrev = wps[1].second - wps[0].second;
  int len    = std::max(std::abs(dxPrev), std::abs(dyPrev));
  if (len != 0) { dxPrev /= len; dyPrev /= len; }

  for (size_t i = 1; i+1 < wps.size(); ++i) {
    int dx = wps[i+1].first  - wps[i].first;
    int dy = wps[i+1].second - wps[i].second;
    int l  = std::max(std::abs(dx), std::abs(dy));
    if (l != 0) { dx /= l; dy /= l; }
    if (dx != dxPrev || dy != dyPrev) {
      out.push_back(wps[i]);
      dxPrev = dx; dyPrev = dy;
    }
  }
  out.push_back(wps.back());
  wps.swap(out);
}

// Long-segment skipping using LoS
void smoothPath(std::vector<std::pair<int,int>>& wps) {
  if (wps.size() < 3) return;
  std::vector<std::pair<int,int>> out;
  size_t i = 0;
  while (i < wps.size()) {
    out.push_back(wps[i]);
    size_t j = wps.size() - 1;
    for (; j > i + 1; --j) {
      if (isLineClear(wps[i].first, wps[i].second, wps[j].first, wps[j].second)) break;
    }
    i = j;
  }
  if (out.back() != wps.back()) out.push_back(wps.back());
  wps.swap(out);
}

// Run A* and fill pathWaypoints
void calculatePath() {
  // Reset grid to walkable
  for (int y = 0; y < GRID_SIZE; ++y)
    for (int x = 0; x < GRID_SIZE; ++x)
      walkable[y][x] = (walkable[y][x]); // leave any obstacles you set

  // Fresh node pool
  for (int y = 0; y < GRID_SIZE; ++y)
    for (int x = 0; x < GRID_SIZE; ++x) {
      nodes[y][x] = new Node{ x - OFFSET, y - OFFSET, 999999, 999999, nullptr };
      isPath[y][x] = false;
    }

  pathWaypoints.clear();

  Node* start = nodes[startY + OFFSET][startX + OFFSET];
  Node* goal  = nodes[goalY  + OFFSET][goalX  + OFFSET];

  start->gCost = 0;
  start->hCost = heuristic(startX, startY, goalX, goalY);

  std::vector<Node*> openSet = { start };
  static bool closed[GRID_SIZE][GRID_SIZE];
  for (int y = 0; y < GRID_SIZE; ++y)
    for (int x = 0; x < GRID_SIZE; ++x)
      closed[y][x] = false;

  int DX[8] = {-1,-1,0,1,1,1,0,-1};
  int DY[8] = { 0,-1,-1,-1,0,1,1, 1};

  bool found = false;
  while (!openSet.empty()) {
    Node* cur = *std::min_element(openSet.begin(), openSet.end(),
                   [](Node* a, Node* b){ return a->fCost() < b->fCost(); });
    openSet.erase(std::remove(openSet.begin(), openSet.end(), cur), openSet.end());
    closed[cur->y + OFFSET][cur->x + OFFSET] = true;

    if (cur == goal) {
      found = true;
      Node* p = goal;
      while (p && p != start) {
        isPath[p->y + OFFSET][p->x + OFFSET] = true;
        pathWaypoints.emplace_back(p->x, p->y);
        p = p->parent;
      }
      std::reverse(pathWaypoints.begin(), pathWaypoints.end());
      simplifyPath(pathWaypoints);
      if (pathSmoothingEnabled) smoothPath(pathWaypoints);
      break;
    }

    for (int d = 0; d < 8; ++d) {
      int nx = cur->x + DX[d];
      int ny = cur->y + DY[d];
      if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;

      if (!walkable[ny + OFFSET][nx + OFFSET]) continue;
      if (closed   [ny + OFFSET][nx + OFFSET]) continue;

      int stepCost  = (DX[d] == 0 || DY[d] == 0) ? 10 : 14; // straight vs diagonal
      int tentative = cur->gCost + stepCost;
      Node* n       = nodes[ny + OFFSET][nx + OFFSET];

      if (tentative < n->gCost) {
        n->gCost = tentative;
        n->hCost = heuristic(nx, ny, goalX, goalY);
        n->parent = cur;
        openSet.push_back(n);
      }
    }
  }

  if (!found) {
    printf("[A*] No path found from (%d,%d) to (%d,%d)\n", startX, startY, goalX, goalY);
    pathWaypoints.clear();
  }

  for (int y = 0; y < GRID_SIZE; ++y)
    for (int x = 0; x < GRID_SIZE; ++x) {
      delete nodes[y][x];
      nodes[y][x] = nullptr;
    }
}

// =====================
// Drive helper w/ recovery
// =====================
bool driveToWithRecovery(double targetXmm, double targetYmm) {
  const double tolMM   = 30.0;
  const double basePct = 10.0;

  double lastDist = 1e9;
  double lastProgress = 1e9;
  int stagnantMs = 0;

  while (true) {
    double cx = GPS17.xPosition(mm);
    double cy = GPS17.yPosition(mm);

    double dx = targetXmm - cx;
    double dy = targetYmm - cy;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist <= tolMM) {
      FullDrivetrain.stop();
      printf("[Drive] Arrived\n");
      return true;
    }

    double angleToTarget = std::atan2(dy, dx) * 180.0 / M_PI;
    if (angleToTarget < 0) angleToTarget += 360.0;

    double currH   = getFusedHeading360();
    double err     = angDiff(currH, angleToTarget);
    bool   reverse = std::fabs(err) > 90.0;

    double fixedHeading = angleToTarget;
    if (reverse) fixedHeading = norm360(fixedHeading + 180.0);

    turnToHeadingAbs(fixedHeading, 1.5, 1000);

    double pct = reverse ? -basePct : basePct;
    if (dist < 100.0) pct *= 0.5;
    if (dist <  50.0) pct *= 0.3;

    LeftDrivetrain.spin (fwd, pct, percent);
    RightDrivetrain.spin(fwd, pct, percent);

    printf("[Drive] Target %.0f/%.0f | Pos %.0f/%.0f | Δ%.0f | H=%.1f\n",
           targetXmm, targetYmm, cx, cy, dist, currH);

    // progress monitor (3 mm improvement threshold)
    if (dist < lastProgress - 3.0) {
      lastProgress = dist;
      stagnantMs = 0;
    } else {
      stagnantMs += 50;
    }

    if (stagnantMs > 1000) {
      // Mark an obstacle ~150mm ahead of current bearing
      double aheadX = cx + 150.0 * std::cos(angleToTarget * M_PI/180.0);
      double aheadY = cy + 150.0 * std::sin(angleToTarget * M_PI/180.0);
      addObstacleWithMargin(toGridCoord(aheadX), toGridCoord(aheadY));
      printf("[WARN] Stalled. Mark obstacle & replan.\n");
      FullDrivetrain.stop();
      return false;
    }

    lastDist = dist;
    wait(50, msec);
  }
}

// =====================
// Background logger thread
// =====================
thread gpsLoggerThread([](){
  while (true) {
    logGPSData();

    printf("[Path] Last target: X=%.2f mm, Y=%.2f mm\n",
           gridToMM(goalX), gridToMM(goalY));

    printf("[GPS ] Current: X=%.2f mm, Y=%.2f mm, Heading=%.1f°\n",
           GPS17.xPosition(mm), GPS17.yPosition(mm), getFusedHeading360());

    printf("[Fusion] GPSH=%.1f°, INSH=%.1f°, Offset=%.1f°, Fused=%.1f°\n",
           norm360(GPS17.heading(vex::degrees)),
           norm360(INS.heading (vex::degrees)),
           initialHeadingOffset,
           getFusedHeading360());

    vex::this_thread::sleep_for(500);
  }
});

// =====================
// Follow current path
// =====================
void followPath() {
  if (pathWaypoints.empty()) {
    printf("[Follow] No path to follow\n");
    return;
  }

  // progress tracking
  static double lastX = -1e9, lastY = -1e9;
  static int    noProgressCount = 0;

  // replan rate limit
  static double lastReplanTime = -1e9;

  bool reached = false;
  size_t i = 0;

  while (!reached && i < pathWaypoints.size()) {
    // optional line-of-sight lookahead skip (cheap smoothing of zig-zags)
    while (i + 1 < pathWaypoints.size()) {
      auto [ax, ay] = pathWaypoints[i];
      auto [bx, by] = pathWaypoints[i+1];
      if (isLineClear(ax, ay, bx, by)) i++;
      else break;
    }

    auto [gx, gy] = pathWaypoints[i];
    double targetXmm = gridToMM(gx);
    double targetYmm = gridToMM(gy);

    // adaptive deviation: tighter near goal
    double adaptiveDev = deviationThreshold;
    if (i == pathWaypoints.size()-1) adaptiveDev *= 0.5;

    // current state
    double cx = GPS17.xPosition(mm);
    double cy = GPS17.yPosition(mm);
    double currH = getFusedHeading360();

    // deviation from waypoint
    double dx = targetXmm - cx;
    double dy = targetYmm - cy;
    double deviation = std::sqrt(dx*dx + dy*dy);

    // desired heading towards next waypoint (or along segment)
    double desiredH = (i + 1 < pathWaypoints.size())
      ? norm360(std::atan2(gridToMM(pathWaypoints[i+1].second) - cy,
                           gridToMM(pathWaypoints[i+1].first ) - cx) * 180.0 / M_PI)
      : norm360(std::atan2(dy, dx) * 180.0 / M_PI);

    double hErr = angDiff(currH, desiredH);

    printf("[Follow] wp %zu/%zu → (%.1f,%.1f) pos(%.1f,%.1f) dev=%.1f H=%.1f (des=%.1f err=%.1f)\n",
           i, pathWaypoints.size()-1, targetXmm, targetYmm, cx, cy, deviation, currH, desiredH, hErr);

    // If badly off-heading, correct now (quick snap turn)
    if (std::fabs(hErr) > headingToleranceDeg) {
      turnToHeadingAbs(desiredH, 1.5, 1200);
    }

    // progress bookkeeping
    double dxp = cx - lastX;
    double dyp = cy - lastY;
    if (std::sqrt(dxp*dxp + dyp*dyp) < 20.0) noProgressCount++;
    else                                     noProgressCount = 0;
    lastX = cx; lastY = cy;

    if (noProgressCount >= 3) {
      printf("[Follow] No progress → driveToWithRecovery()\n");
      bool recovered = driveToWithRecovery(targetXmm, targetYmm);
      if (!recovered) stucks++;
      return; // stop following old path — NAVI will replan/continue
    }

    // too far off → replan (rate limited)
    double now = Brain.timer(sec);
    if (deviation > adaptiveDev && std::fabs(hErr) < 15.0) {
      if (now - lastReplanTime >= 2.0) {
        printf("[Follow] Deviation too large → Replan\n");
        updateStartPositionFromGPS();
        calculatePath();
        replans++;
        i = 0;
        lastReplanTime = now;
        continue;
      } else {
        printf("[Follow] Skipping replan (rate limited)\n");
      }
    }

    // If close enough to waypoint → advance
    if (deviation < waypointTolerance) {
      ++i;
      if (i >= pathWaypoints.size()) {
        // final goal check
        double dist = std::sqrt((targetXmm - cx)*(targetXmm - cx) +
                                (targetYmm - cy)*(targetYmm - cy));
        if (dist <= waypointTolerance) reached = true;
        else {
          printf("[Follow] Overshot goal → Replan\n");
          updateStartPositionFromGPS();
          calculatePath();
          replans++;
          i = 0;
        }
        continue;
      }
    } else {
      // advance towards waypoint (short burst)
      bool at = driveToWithRecovery(targetXmm, targetYmm);
      if (!at) return; // recovery requested replan already
      ++i;
      if (i >= pathWaypoints.size()) reached = true;
    }
  }
}

// =====================
// Simple learning tweak
// =====================
void applyLearnedTuning() {
  if (!enableLearning) return;
  FILE* f = fopen("run_feedback.csv", "r"); if (!f) return;

  char line[256];
  double totalDev=0, totalRe=0, totalSt=0; int count=0;

  while (fgets(line, sizeof(line), f)) {
    long ts; int success, re, st, rating; double t, dev; char notes[128];
    if (sscanf(line, "%ld,%d,%lf,%d,%d,%lf,%d,%127[^\n]",
               &ts, &success, &t, &re, &st, &dev, &rating, notes) == 8) {
      totalDev += dev; totalRe += re; totalSt += st; count++;
    }
  }
  fclose(f);
  if (count == 0) return;

  double avgDev = totalDev / count;
  double avgRe  = totalRe  / count;
  double avgSt  = totalSt  / count;

  if (avgDev > 60) deviationThreshold *= 1.2;
  else if (avgDev < 30) deviationThreshold *= 0.8;

  if (avgRe > 3) headingToleranceDeg += 2;
  else if (avgRe < 1) headingToleranceDeg = std::max(5.0, headingToleranceDeg - 2.0);

  if (avgSt > 4) headingToleranceDeg += 1;
}

// =====================
// Main NAVI entry
// =====================
void NAVI(double targetXmm, double targetYmm) {
  double t0 = Brain.timer(sec);

  // Optional: apply learnt parameter nudges
  applyLearnedTuning();

  // Goal clamp & grid init
  goalX = clampi(toGridCoord(targetXmm), -OFFSET, OFFSET);
  goalY = clampi(toGridCoord(targetYmm), -OFFSET, OFFSET);

  for (int y = 0; y < GRID_SIZE; ++y)
    for (int x = 0; x < GRID_SIZE; ++x)
      walkable[y][x] = true;

  // Perimeter walls (+1 margin)
  for (int i = -36; i <= 36; ++i) {
    addObstacleWithMargin(i,  -36);
    addObstacleWithMargin(i,   36);
    addObstacleWithMargin(-36, i);
    addObstacleWithMargin( 36, i);
  }

  // Start from current GPS position
  updateStartPositionFromGPS();

  // Plan & go
  calculatePath();
  followPath();

  timeTaken = Brain.timer(sec) - t0;

  // Evaluate result
  double fx = GPS17.xPosition(mm);
  double fy = GPS17.yPosition(mm);
  double fdev = std::sqrt((fx - targetXmm)*(fx - targetXmm) + (fy - targetYmm)*(fy - targetYmm));
  bool success = fdev <= waypointTolerance;

  // Log supervised feedback
  int operatorRating = 5;
  std::string notes  = "Auto log";
  logRunFeedback(success, timeTaken, replans, stucks, fdev, operatorRating, notes);

  printf("[NAVI] Done. success=%d time=%.2fs replans=%d stucks=%d finalDev=%.1fmm\n",
         success ? 1 : 0, timeTaken, replans, stucks, fdev);
}
