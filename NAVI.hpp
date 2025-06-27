// VEX Pfadnavigation mit A*, Stuck-Recovery, Live-Repathing, Logging, Hindernisrahmen und Rückwärtsfahrt + Liveausgabe
#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <unordered_set>
#include <fstream>
#include <limits>

using namespace vex;

const double FIELD_SIZE_MM = 3600.0;
const int GRID_SIZE = 73;
const int OFFSET = GRID_SIZE / 2;
const double CELL_SIZE = FIELD_SIZE_MM / GRID_SIZE;

int startX, startY;
int goalX, goalY;
bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = {{false}};
std::vector<std::pair<int, int>> pathWaypoints;
std::vector<std::pair<int, int>> customObstacles;

void logGPSData() {
  if (!Brain.SDcard.isInserted()) {
    Brain.Screen.print("No SD card inserted!");
    return;
  }
  FILE* logFile = fopen("gps_log.txt", "a");
  if (logFile) {
    double x = GPS17.xPosition(mm);
    double y = GPS17.yPosition(mm);
    double heading = GPS17.heading();
    fprintf(logFile, "X: %.2f mm, Y: %.2f mm, Heading: %.2f deg\n", x, y, heading);
    fclose(logFile);
  }
}

double clamp(double value, double minVal, double maxVal) {
  return std::max(minVal, std::min(maxVal, value));
}

double normalize360(double angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

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

double gridToMM(int gridVal) {
  return gridVal * CELL_SIZE;
}

int toGridCoord(double mm) {
  return clamp(static_cast<int>(round(mm / CELL_SIZE)), -OFFSET, OFFSET);
}

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

bool driveToWithRecovery(double targetXmm, double targetYmm) {
  const double tolerance = 30;
  double maxSpeed = 30.0;
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

    double heading = normalize360(GPS17.heading());
    double angleToTarget = normalize360(atan2(dy, dx) * 180.0 / M_PI);
    double angleError = angleToTarget - heading;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    bool reverse = fabs(angleError) > 90.0;
    double speed = reverse ? -maxSpeed : maxSpeed;
    if (dist < 100.0) speed *= 0.5;
    if (dist < 50.0)  speed *= 0.3;

    if (reverse) {
      angleToTarget = normalize360(angleToTarget + 180);
      angleError = angleToTarget - heading;
      if (angleError > 180) angleError -= 360;
      if (angleError < -180) angleError += 360;
    }

    double turnStrength = clamp(angleError * 0.04, -20.0, 20.0);
    double leftSpeed = speed - turnStrength;
    double rightSpeed = speed + turnStrength;

    LeftDrivetrain.spin(forward, leftSpeed, percent);
    RightDrivetrain.spin(forward, rightSpeed, percent);

    printf("[Drive] → Target: %.0f/%.0f mm | Pos: %.0f/%.0f mm | Δ%.0f mm | Heading %.1f° | Mode: %s\n",
           targetXmm, targetYmm, currentX, currentY, dist, heading, reverse ? "REV" : "FWD");

    if (fabs(dist - lastDist) < 1.0) stuckCounter++;
    else stuckCounter = 0;

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

void followPath() {
  while (true) {
    for (auto& wp : pathWaypoints) {
      double x = gridToMM(wp.first);
      double y = gridToMM(wp.second);
      if (!driveToWithRecovery(x, y)) {
        updateStartPositionFromGPS();
        calculatePath();
        break;
      }
      updateStartPositionFromGPS();
    }
    break;
  }
  while (true) {
    printf("[Path] Last target: X=%.2f mm, Y=%.2f mm\n", gridToMM(goalX), gridToMM(goalY));
    printf("[GPS ] Current:     X=%.2f mm, Y=%.2f mm, Heading=%.1f°\n",
           GPS17.xPosition(mm),
           GPS17.yPosition(mm),
           GPS17.heading());
    task::sleep(500);
  }
}

void NAVI(double targetXmm, double targetYmm) {
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

  updateStartPositionFromGPS();
  calculatePath();
  followPath();
}
