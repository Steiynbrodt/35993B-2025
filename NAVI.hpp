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
const int GRID_SIZE = 72;
const int OFFSET = GRID_SIZE / 2; // bleibt kompatibel
const double CELL_SIZE = FIELD_SIZE_MM / GRID_SIZE;


int startX, startY;
int goalX, goalY;
bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = {{false}};
double clamp(double value, double minVal, double maxVal) {
  return std::max(minVal, std::min(maxVal, value));
}
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
  int coord = static_cast<int>(round(mm / CELL_SIZE));
  return clamp(coord, -OFFSET, OFFSET);
}
void addObstacleWithMargin(int x, int y) {
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      int nx = x + dx;
      int ny = y + dy;
      if (nx >= -OFFSET && nx <= OFFSET && ny >= -OFFSET && ny <= OFFSET) {
        walkable[ny + OFFSET][nx + OFFSET] = false;
      }
    }
  }
}

void updateStartPositionFromGPS() {
  double gpsX = GPS17.xPosition(mm);
  double gpsY = GPS17.yPosition(mm);
  if (std::isnan(gpsX) || std::isnan(gpsY)) {
    printf("[ERROR] Invalid GPS data");
    return;
  }
  startX = clamp(toGridCoord(gpsX), -OFFSET, OFFSET);
  startY = clamp(toGridCoord(gpsY), -OFFSET, OFFSET);
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





void driveTo(double targetXmm, double targetYmm) {
  const double tolerance = 60.0;         // Wann gilt Ziel als erreicht
  double maxSpeed = 30.0;                // Startgeschwindigkeit
  double speed = maxSpeed;
  double lastDist = 9999.0;
  int stuckCounter = 0;

  while (true) {
    double currentX = GPS17.xPosition(mm);
    double currentY = GPS17.yPosition(mm);
    double dx = targetXmm - currentX;
    double dy = targetYmm - currentY;
    double dist = sqrt(dx * dx + dy * dy);

    // === Ziel erreicht? ===
    if (dist <= tolerance) {
      Brain.Screen.print("arrived");
      FullDrivetrain.stop();
      break;
    }

    // === Rückwärtsentscheidung ===
    double angleToTarget = atan2(dy, dx) * 180.0 / M_PI;
    angleToTarget = normalize360(angleToTarget);
    double currentHeading = normalize360(GPS17.heading());
    double angleError = shortestAngleDiff(angleToTarget, currentHeading);

    bool reverse = fabs(angleError) > 90.0;  // rückwärts fahren ab 90°

    if (reverse) {
      speed = -maxSpeed;  // Rückwärtsgeschwindigkeit
      angleToTarget = normalize360(angleToTarget + 180);  // Neue Zielausrichtung
      angleError = shortestAngleDiff(angleToTarget, currentHeading);
    } else {
      speed = maxSpeed; // Vorwärts
    }

    // === Geschwindigkeit reduzieren beim Nähern ===
    if (dist < 100.0) speed *= 0.5;
    if (dist < 50.0)  speed *= 0.3;

    // === Drehen und Korrektur ===
    double turnStrength = clamp(angleError * 0.004, -20.0, 20.0);
    double leftSpeed = speed - turnStrength;
    double rightSpeed = speed + turnStrength;

    LeftDrivetrain.spin(forward, leftSpeed, percent);
    RightDrivetrain.spin(forward, rightSpeed, percent);

    // === Ausgabe & Stuck-Detection ===
    printf("[Drive] → Target: %.0f/%.0f mm | Pos: %.0f/%.0f mm | Δ%.0f mm | Heading %.1f° | Mode: %s\n",
           targetXmm, targetYmm,
           currentX, currentY,
           dist,
           currentHeading,
           reverse ? "REV" : "FWD");

    if (fabs(dist - lastDist) < 1.5) stuckCounter++;
    else stuckCounter = 0;
    lastDist = dist;

    if (stuckCounter > 15) {
      printf("[WARN] Movement stalled. Aborting.\n");
      break;  // Roboter steckt fest
    }

    wait(50, msec);

    
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
  double lastTargetX = 0;
  double lastTargetY = 0;

  for (auto& point : pathWaypoints) {
    double absXmm = gridToMM(point.first);
    double absYmm = gridToMM(point.second);

    lastTargetX = absXmm;
    lastTargetY = absYmm;

    driveTo(absXmm, absYmm);
    wait(300, msec);
    updateStartPositionFromGPS();
  }

  //  Dauerausgabe nach dem Pfad
  while (true) {
    printf("[Path] Last target: X=%.2f mm, Y=%.2f mm\n", lastTargetX, lastTargetY);
    printf("[GPS ] Current:     X=%.2f mm, Y=%.2f mm, Heading=%.1f°\n",
           GPS17.xPosition(mm),
           GPS17.yPosition(mm),
           GPS17.heading());
    task::sleep(500);  // 0.5 Sekunden warten
  }
}


void logGPSData() {
  if (!Brain.SDcard.isInserted()) {
    Brain.Screen.print("No SD card inserted!");
    return;
  }

  // Open file for appending
  FILE* logFile = fopen("gps_log.txt", "a");

  if (logFile) {
    for (int i = 0; i < 100; i++) {  // Log 100 entries, then stop
      double x = GPS17.xPosition(mm);
      double y = GPS17.yPosition(mm);
      double heading = GPS17.heading();

      fprintf(logFile, "X: %.2f mm, Y: %.2f mm, Heading: %.2f deg\n", x, y, heading);
      
      wait(500, msec);  // Wait 0.5 seconds
    }

    fclose(logFile);  // Important!
    Brain.Screen.print("GPS data logged.");
  } else {
    Brain.Screen.print("Failed to open log file.");
  }
}

// === Hauptfunktion zur Navigation und Hindernisinitialisierung ===
 void NAVI(double targetXmm, double targetYmm) {
  // Zielkoordinaten begrenzen
  int targetX = toGridCoord(targetXmm);
  int targetY = toGridCoord(targetYmm);
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
  // Alte Hindernisse skaliert + Margin angewendet
int obstacles[][2] = {
  // Top edge
  {-36, 36}, {-34, 36}, {-32, 36}, {-30, 36}, {-28, 36}, {-26, 36}, {-24, 36}, {-22, 36},
  {-20, 36}, {-18, 36}, {-16, 36}, {-14, 36}, {-12, 36}, {-10, 36}, {-8, 36}, {-6, 36},
  {-4, 36}, {-2, 36}, {0, 36}, {2, 36}, {4, 36}, {6, 36}, {8, 36}, {10, 36}, {12, 36},
  {14, 36}, {16, 36}, {18, 36}, {20, 36}, {22, 36}, {24, 36}, {26, 36}, {28, 36},
  {30, 36}, {32, 36}, {34, 36}, {36, 36},

  // Bottom edge
  {-36, -36}, {-34, -36}, {-32, -36}, {-30, -36}, {-28, -36}, {-26, -36}, {-24, -36}, {-22, -36},
  {-20, -36}, {-18, -36}, {-16, -36}, {-14, -36}, {-12, -36}, {-10, -36}, {-8, -36}, {-6, -36},
  {-4, -36}, {-2, -36}, {0, -36}, {2, -36}, {4, -36}, {6, -36}, {8, -36}, {10, -36}, {12, -36},
  {14, -36}, {16, -36}, {18, -36}, {20, -36}, {22, -36}, {24, -36}, {26, -36}, {28, -36},
  {30, -36}, {32, -36}, {34, -36}, {36, -36},

  // Left edge
  {-36, -34}, {-36, -32}, {-36, -30}, {-36, -28}, {-36, -26}, {-36, -24}, {-36, -22},
  {-36, -20}, {-36, -18}, {-36, -16}, {-36, -14}, {-36, -12}, {-36, -10}, {-36, -8},
  {-36, -6}, {-36, -4}, {-36, -2}, {-36, 0}, {-36, 2}, {-36, 4}, {-36, 6}, {-36, 8},
  {-36, 10}, {-36, 12}, {-36, 14}, {-36, 16}, {-36, 18}, {-36, 20}, {-36, 22}, {-36, 24},
  {-36, 26}, {-36, 28}, {-36, 30}, {-36, 32}, {-36, 34},

  // Right edge
  {36, -34}, {36, -32}, {36, -30}, {36, -28}, {36, -26}, {36, -24}, {36, -22},
  {36, -20}, {36, -18}, {36, -16}, {36, -14}, {36, -12}, {36, -10}, {36, -8},
  {36, -6}, {36, -4}, {36, -2}, {36, 0}, {36, 2}, {36, 4}, {36, 6}, {36, 8},
  {36, 10}, {36, 12}, {36, 14}, {36, 16}, {36, 18}, {36, 20}, {36, 22}, {36, 24},
  {36, 26}, {36, 28}, {36, 30}, {36, 32}, {36, 34}
};

// Einfügen ins Grid mit Margin
for (int i = 0; i < sizeof(obstacles)/sizeof(obstacles[0]); ++i) {
  int x = obstacles[i][0];
  int y = obstacles[i][1];
  addObstacleWithMargin(x, y);
};
   
  updateStartPositionFromGPS();
  calculatePath();
  //printGrid();
  
  followPath();

  while (true) {
    wait(1000, msec);
  }
  

}
