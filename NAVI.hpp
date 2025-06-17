#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <unordered_set>
#include <fstream>
#include <limits>

using namespace vex;

// === Feld- und Rasterkonfiguration ===
const double FIELD_SIZE_MM = 3600.0;
const int GRID_SIZE = 37;
const int OFFSET = 18;
const double CELL_SIZE = FIELD_SIZE_MM / GRID_SIZE;

int startX, startY;
int goalX, goalY;
bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = { false };

std::vector<std::pair<int, int>> pathWaypoints;
std::vector<std::pair<int, int>> customObstacles;
// === Hilfsfunktion zur Normalisierung von Sensorwerten (-180° bis 180° → 0° bis 360°) ===
double normalize360(double angle) {
  if (angle < 0) angle += 360;
  return angle;
}
void addObstacle(int x, int y) {
  if (x >= -OFFSET && x <= OFFSET && y >= -OFFSET && y <= OFFSET) {
    walkable[y + OFFSET][x + OFFSET] = false;
    customObstacles.push_back({x, y});
  }
}

void clearObstacles() {
  for (auto& obs : customObstacles) {
    int x = obs.first;
    int y = obs.second;
    if (x >= -OFFSET && x <= OFFSET && y >= -OFFSET && y <= OFFSET) {
      walkable[y + OFFSET][x + OFFSET] = true;
    }
  }
  customObstacles.clear();
}

double gridToMM(int gridVal) {
  return gridVal * CELL_SIZE;
}

int toGridCoord(double mm) {
  return static_cast<int>(round(mm / CELL_SIZE));
}

void updateStartPositionFromGPS() {
  startX = toGridCoord(GPS17.xPosition(mm));
  startY = toGridCoord(GPS17.yPosition(mm));
}

void printGrid() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  const int step = 3;
  for (int y = -OFFSET; y <= OFFSET; y += step) {
    for (int x = -OFFSET; x <= OFFSET; x += step) {
      int rx = x + OFFSET;
      int ry = y + OFFSET;
      if (x == 0 && y == 0)
        Brain.Screen.print("X");
      else if (x == startX && y == startY)
        Brain.Screen.print("R");
      else if (x == goalX && y == goalY)
        Brain.Screen.print("Z");
      else if (!walkable[ry][rx])
        Brain.Screen.print("#");
      else if (isPath[ry][rx])
        Brain.Screen.print("*");
      else
        Brain.Screen.print(".");
    }
    Brain.Screen.newLine();
  }
}

double shortestAngleDiff(double target, double current) {
  double diff = target - current;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  return diff;
}

double clamp(double value, double minVal, double maxVal) {
  return std::max(minVal, std::min(maxVal, value));
}

void turnTo(int targetAngleDeg) {
  targetAngleDeg = normalize360(targetAngleDeg);
  double currentAngle = normalize360(GPS17.heading());
  double error = shortestAngleDiff(targetAngleDeg, currentAngle);
  double direction = (error > 0) ? 1 : -1;

  while (fabs(error) > 10) {
    driveMotorLeftOne.spin(forward, -5 * direction, percent);
    driveMotorLeftTwo.spin(forward, -5 * direction, percent);
    driveMotorLeftThree.spin(forward, -5 * direction, percent);
    driveMotorRightOne.spin(forward, 5 * direction, percent);
    driveMotorRightTwo.spin(forward, 5 * direction, percent);
    driveMotorRightThree.spin(forward, 5 * direction, percent);

    wait(50, msec);

    currentAngle = normalize360(GPS17.heading());
    error = shortestAngleDiff(targetAngleDeg, currentAngle);
    direction = (error > 0) ? 1 : -1;
  }

  driveMotorLeftOne.stop(brake);
  driveMotorLeftTwo.stop(brake);
  driveMotorLeftThree.stop(brake);
  driveMotorRightOne.stop(brake);
  driveMotorRightTwo.stop(brake);
  driveMotorRightThree.stop(brake);
}

void driveTo(double targetXmm, double targetYmm) {
  const double tolerance = 20.0;
  const double speed = 30.0;

  while (true) {
    double currentX = GPS17.xPosition(mm);
    double currentY = GPS17.yPosition(mm);
    double dx = targetXmm - currentX;
    double dy = targetYmm - currentY;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist <= tolerance) break;

    double angle = atan2(dy, dx) * 180.0 / M_PI;
    angle = normalize360(angle);
    double currentHeading = normalize360(GPS17.heading());
    double angleError = shortestAngleDiff(angle, currentHeading);

    double turnStrength = clamp(angleError * 0.5, -20.0, 20.0);
    double leftSpeed = speed - turnStrength;
    double rightSpeed = speed + turnStrength;

    driveMotorLeftOne.spin(forward, leftSpeed, percent);
    driveMotorLeftTwo.spin(forward, leftSpeed, percent);
    driveMotorLeftThree.spin(forward, leftSpeed, percent);
    driveMotorRightOne.spin(forward, rightSpeed, percent);
    driveMotorRightTwo.spin(forward, rightSpeed, percent);
    driveMotorRightThree.spin(forward, rightSpeed, percent);

    wait(50, msec);
  }

  driveMotorLeftOne.stop(brake);
  driveMotorLeftTwo.stop(brake);
  driveMotorLeftThree.stop(brake);
  driveMotorRightOne.stop(brake);
  driveMotorRightTwo.stop(brake);
  driveMotorRightThree.stop(brake);
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
  bool openSetMap[GRID_SIZE][GRID_SIZE] = { false };
  openSetMap[startY + OFFSET][startX + OFFSET] = true;

  int dx[8] = { -1, -1, 0, 1, 1, 1, 0, -1 };
  int dy[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };

  while (!openSet.empty()) {
    Node* currentnode = *std::min_element(openSet.begin(), openSet.end(), [](Node* a, Node* b) {
      return a->fCost() < b->fCost();
    });
    openSet.erase(std::remove(openSet.begin(), openSet.end(), currentnode), openSet.end());
    openSetMap[currentnode->y + OFFSET][currentnode->x + OFFSET] = false;

    int cx = currentnode->x + OFFSET;
    int cy = currentnode->y + OFFSET;
    closedSet[cy][cx] = true;

    if (currentnode == goal) {
      Node* p = goal;
      while (p != start && p != nullptr) {
        int gx = p->x + OFFSET;
        int gy = p->y + OFFSET;
        isPath[gy][gx] = true;
        pathWaypoints.push_back({p->x, p->y});
        p = p->parent;
      }
      std::reverse(pathWaypoints.begin(), pathWaypoints.end());
      break;
    }

    for (int d = 0; d < 8; d++) {
      int nx = currentnode->x + dx[d];
      int ny = currentnode->y + dy[d];
      if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;
      int rx = nx + OFFSET;
      int ry = ny + OFFSET;
      if (!walkable[ry][rx] || closedSet[ry][rx]) continue;

      if (dx[d] != 0 && dy[d] != 0) {
        int adj1x = currentnode->x + dx[d];
        int adj1y = currentnode->y;
        int adj2x = currentnode->x;
        int adj2y = currentnode->y + dy[d];
        if (!walkable[adj1y + OFFSET][adj1x + OFFSET] ||
            !walkable[adj2y + OFFSET][adj2x + OFFSET]) continue;
      }

      Node* neighbor = nodes[ry][rx];
      int moveCost = (dx[d] == 0 || dy[d] == 0) ? 10 : 14;
      int tentativeG = currentnode->gCost + moveCost;

      if (tentativeG < neighbor->gCost) {
        neighbor->parent = currentnode;
        neighbor->gCost = tentativeG;
        neighbor->hCost = heuristic(nx, ny, goalX, goalY);
        if (!openSetMap[ry][rx]) {
          openSet.push_back(neighbor);
          openSetMap[ry][rx] = true;
        }
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

// === Folgen des Pfads mit Umrechnung der Koordinaten ===
void followPath() {
  for (auto& point : pathWaypoints) {
    double absXmm = gridToMM(point.first);
    double absYmm = gridToMM(point.second);
    driveTo(absXmm, absYmm);
    wait(300, msec);
    updateStartPositionFromGPS();
  }
}



// === Hauptfunktion zur Navigation und Hindernisinitialisierung ===
int NAVI(int targetX, int targetY) {
  // Zielkoordinaten begrenzen
  targetX = clamp(targetX, -OFFSET, OFFSET);
  targetY = clamp(targetY, -OFFSET, OFFSET);
  goalX = targetX;
  goalY = targetY;

  // Raster zurücksetzen
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      walkable[y][x] = true;
      isPath[y][x] = false;
    }
  }

  // Alte Hindernisse entfernen
  

  // Neue Hindernisse einfügen (Beispiel)
  int obstacles[][2] = {
    {-6, 0}, {6, 0}, {0, 6}, {0, -6},
    {-7, 0}, {7, 0}, {0, -7}, {0, 7},
    {-6, -1}, {6, 1}, {1, 6}, {1, -6},
    {-5, 0}, {5, 0}, {0, 5}, {0, -5}
  };
  int obstacleCount = sizeof(obstacles) / sizeof(obstacles[0]);
  for (int i = 0; i < obstacleCount; i++) {
    addObstacle(obstacles[i][0], obstacles[i][1]);
  }

  updateStartPositionFromGPS();
  calculatePath();
  printGrid();
  followPath();

  while (true) {
    wait(500, msec);
  }
}
