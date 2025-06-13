#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
using namespace vex;


// Field dimensions
const double FIELD_SIZE_MM = 3600.0;  // Field is 3600mm x 3600mm

// Use a 37x37 grid to have a unique center cell (0,0)
const int GRID_SIZE = 37;
const int OFFSET = 18;  // Grid coordinate = index - OFFSET, so valid range: -18 to +18

// Calculate cell size so that the grid exactly covers the field.
const double CELL_SIZE = FIELD_SIZE_MM / GRID_SIZE;  // ≈ 97.30 mm per cell

int startX, startY;
int goalX, goalY;

bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = { false };

/*void Debuging(){
  while(true){
  Brain.Screen.print(INS.heading());
  Brain.Screen.newLine();
  }
  
}*/
// Converts a grid coordinate to millimeters.
double gridToMM(int gridVal) {
  return gridVal * CELL_SIZE;
}

// Converts a millimeter measurement to a grid coordinate.
int toGridCoord(double mm) {
  return static_cast<int>(round(mm / CELL_SIZE));
}

void updateStartPositionFromGPS() {
  startX = toGridCoord(GPS17.xPosition(mm));
  startY = toGridCoord(GPS17.yPosition(mm));
}

std::vector<std::pair<int, int>> pathWaypoints;

void printGrid() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  const int step = 3;
  // Loop over grid coordinates from -18 to +18
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

double normalize360(double angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

double shortestAngleDiff(double target, double current) {
  double diff = target - current;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  return diff;
}

void turnTo(int targetAngleDeg) {
  targetAngleDeg = normalize360(targetAngleDeg);

  double currentAngle = normalize360(GPS17.heading() + 180.0); // Shift to 0–360°
  double error = shortestAngleDiff(targetAngleDeg, currentAngle);
  double direction = (error > 0) ? 1 : -1;

  while (fabs(error) > 10) {
    // Apply turning in place
    LeftDrivetrain.spin(forward, -5 * direction, percent);
    RightDrivetrain.spin(reverse, -5 * direction, percent);

    wait(50, msec);

    currentAngle = normalize360(GPS17.heading() + 180.0);
    error = shortestAngleDiff(targetAngleDeg, currentAngle);
    direction = (error > 0) ? 1 : -1;
  }

  LeftDrivetrain.stop(brake);
  RightDrivetrain.stop(brake);
}
double clamp(double value, double minVal, double maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}
void driveTo(double targetXmm, double targetYmm) {
  const double tolerance = 20.0; // Acceptable range in mm
  const double speed = 30.0;     // Drive speed in percent

  while (true) {
    double currentX = GPS17.xPosition(mm);
    double currentY = GPS17.yPosition(mm);

    double dx = targetXmm - currentX;
    double dy = targetYmm - currentY;
    double dist = sqrt(dx * dx + dy * dy);

    // Exit condition
    if (dist <= tolerance)
      break;

    // Angle to target
    double angle = atan2(dy, dx) * 180.0 / M_PI;
    angle = fmod(angle + 360.0, 360.0);

    // Current heading
    double currentHeading = normalize360(GPS17.heading() + 180.0);
    double angleError = shortestAngleDiff(angle, currentHeading);

    // Simple turning correction (proportional)
    double turnStrength = angleError * 0.5; // Tune as needed
    turnStrength = clamp(turnStrength, -20.0, 20.0);

    // Drive with correction
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

  // Stop all motors
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

// Manhattan distance heuristic
int heuristic(int x1, int y1, int x2, int y2) {
  return 10 * (abs(x1 - x2) + abs(y1 - y2));
}

void calculatePath() {
  // Allocate and initialize nodes for each grid cell.
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
  
  int dx[8] = { -1, -1, 0, 1, 1, 1, 0, -1 };
  int dy[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
  
  while (!openSet.empty()) {
    Node* currentnode = openSet[0];
    int currentIndex = 0;
    for (int i = 1; i < openSet.size(); i++) {
      if (openSet[i]->fCost() < currentnode->fCost()) {
        currentnode = openSet[i];
        currentIndex = i;
      }
    }
    openSet.erase(openSet.begin() + currentIndex);
    int cx = currentnode->x + OFFSET;
    int cy = currentnode->y + OFFSET;
    closedSet[cy][cx] = true;
    
    // If the goal is reached, backtrack to record the path.
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
    
    // Process neighbors.
    for (int d = 0; d < 8; d++) {
      int nx = currentnode->x + dx[d];
      int ny = currentnode->y + dy[d];
      if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;
      int rx = nx + OFFSET;
      int ry = ny + OFFSET;
      if (!walkable[ry][rx] || closedSet[ry][rx]) continue;
      
      // For diagonal movement, ensure adjacent cells are walkable.
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
        openSet.push_back(neighbor);
      }
    }
  }
  
  // Free the dynamically allocated nodes to avoid memory leaks.
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      delete nodes[y][x];
      nodes[y][x] = nullptr;
    }
  }
}

void followPath() {
  for (auto& point : pathWaypoints) {
    int targetGridX = point.first;
    int targetGridY = point.second;

    // Convert grid coordinates directly to absolute mm positions
    double absXmm = gridToMM(targetGridX);
    double absYmm = gridToMM(targetGridY);

    driveTo(absXmm, absYmm);

    wait(300, msec);

    updateStartPositionFromGPS(); // Optional: if you need up-to-date GPS for next move
  }
}

int NAVI(int targetX, int targetY) {
  // Clamp target coordinates to valid grid coordinates.
  if (targetX < -OFFSET) targetX = -OFFSET;
  if (targetX > OFFSET)  targetX = OFFSET;
  if (targetY < -OFFSET) targetY = -OFFSET;
  if (targetY > OFFSET)  targetY = OFFSET;
  goalX = targetX;
  goalY = targetY;
  
  // Initialize grid: mark all cells as walkable and clear any previous path markings.
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      walkable[y][x] = true;
      isPath[y][x] = false;
    }
  }
  
  // Example obstacles (grid coordinates).
  int obstacles[][2] = {
    {-6, 0}, {6, 0}, {0, 6}, {0, -6},{-7, 0}, {7, 0}, {0, -7}, {7, 0},{-6, -1}, {6, 1}, {1, 6}, {1, -6},{-5, 0}, {5, 0}, {0, 5}, {0, -5}
    
  };
  int obstacleCount = sizeof(obstacles) / sizeof(obstacles[0]);
  for (int i = 0; i < obstacleCount; i++) {
    int ox = obstacles[i][0];
    int oy = obstacles[i][1];
    if (ox >= -OFFSET && ox <= OFFSET && oy >= -OFFSET && oy <= OFFSET) {
      walkable[oy + OFFSET][ox + OFFSET] = false;
    }
  }
  
  updateStartPositionFromGPS();
  calculatePath();
  printGrid();
  followPath();
  

  while (true) {
    wait(500, msec);
  }
}


