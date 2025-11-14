#include <queue>
#include <vector>
#include <cmath>
#include "vex.h"
#include <limits>
#include <utility>
#include "driveforward.hpp"
#include "fieldparameters.hpp"


// --- A* (Octile) on your Grid -----------------------------------------------

namespace pathfind {

struct Node {
    int x, y;
    double f;
    // optional tie-breaker: prefer nodes closer to goal along straight line
    double h;
};

struct NodeCmp {
    bool operator()(const Node& a, const Node& b) const {
        if (a.f == b.f) return a.h > b.h; // smaller h first on equal f
        return a.f > b.f;                 // min-heap
    }
};

inline bool traversable(const Cell& c) {
    // Walkable: FREE, START, GOAL (do NOT step on INFLATED or OBSTACLE)
    switch (c.state) {
        case Cellstate::FREE:
        case Cellstate::START:
        case Cellstate::GOAL:
            return true;
        default:
            return false;
    }
}

// Disallow "corner cutting": for diagonal moves, both orthogonal neighbors must be free.
inline bool canStep(const Grid& g, int x, int y, int nx, int ny) {
    if (!g.inBounds(nx, ny)) return false;
    if (!traversable(g.at(nx, ny))) return false;

    const int dx = nx - x;
    const int dy = ny - y;
    if (dx != 0 && dy != 0) {
        // diagonal: require both side cells to be free
        if (!traversable(g.at(x, ny))) return false;
        if (!traversable(g.at(nx, y))) return false;
    }
    return true;
}

// Octile heuristic (8-connected)
inline double octile(int x, int y, int gx, int gy) {
    int dx = std::abs(gx - x);
    int dy = std::abs(gy - y);
    static const double SQRT2 = std::sqrt(2.0);
    int m = std::min(dx, dy);
    int M = std::max(dx, dy);
    // = (M - m)*1 + m*sqrt(2)
    return (M - m) + m * SQRT2;
}

// Reconstruct path from parents (indices)
inline std::vector<std::pair<int,int>> reconstructPath(
    int goalIdx, const std::vector<int>& parent, int cols)
{
    std::vector<std::pair<int,int>> path;
    for (int cur = goalIdx; cur != -1; cur = parent[cur]) {
        int y = cur / cols;
        int x = cur % cols;
        path.emplace_back(x, y);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Main A* function
inline std::vector<std::pair<int,int>> astar(
    const Grid& g, int sx, int sy, int gx, int gy)
{
    if (!g.inBounds(sx, sy) || !g.inBounds(gx, gy)) return {};
    if (!traversable(g.at(sx, sy)) || !traversable(g.at(gx, gy))) return {};
    const int C = g.cols();
    const int R = g.rows();
    const size_t N = static_cast<size_t>(C) * R;

    static const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> gScore(N, INF);
    std::vector<double> fScore(N, INF);
    std::vector<int>    parent(N, -1);
    std::vector<char>   inOpen(N, 0); // small optimization

    auto idx = [C](int x, int y){ return y * C + x; };

    auto h0 = octile(sx, sy, gx, gy);
    gScore[idx(sx, sy)] = 0.0;
    fScore[idx(sx, sy)] = h0;

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;
    open.push(Node{sx, sy, h0, h0});
    inOpen[idx(sx, sy)] = 1;

    // 8 moves with their costs
    static const int DX[8] = {+1,-1, 0, 0, +1,+1,-1,-1};
    static const int DY[8] = { 0, 0,+1,-1, +1,-1,+1,-1};
    static const double COST[8] = {
        1, 1, 1, 1, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)
    };

    while (!open.empty()) {
        Node cur = open.top(); open.pop();
        int cx = cur.x, cy = cur.y;
        int cidx = idx(cx, cy);

        if (cx == gx && cy == gy) {
            return reconstructPath(cidx, parent, C);
        }

        // If this node is stale (its f doesn't match current best), skip
        if (cur.f != fScore[cidx]) continue;

        for (int k = 0; k < 8; ++k) {
            int nx = cx + DX[k];
            int ny = cy + DY[k];
            if (!canStep(g, cx, cy, nx, ny)) continue;

            int nidx = idx(nx, ny);
            double tentative = gScore[cidx] + COST[k];
            if (tentative < gScore[nidx]) {
                parent[nidx]  = cidx;
                gScore[nidx]  = tentative;
                double h      = octile(nx, ny, gx, gy);
                fScore[nidx]  = tentative + h;
                open.push(Node{nx, ny, fScore[nidx], h});
                inOpen[nidx] = 1;
            }
        }
    }
    // No path
    return {};
}

// Optional: paint the path onto a Grid
inline void paintPath(Grid& g, const std::vector<std::pair<int,int>>& path) {
    for (size_t i = 0; i < path.size(); ++i) {
        auto [x,y] = path[i];
        // Keep START/GOAL if they are already marked
        if (g.at(x,y).state == Cellstate::START || g.at(x,y).state == Cellstate::GOAL) continue;
        g.at(x,y).setPath();
    }
}

} // namespace pathfind


struct CellXY { int x, y; };

// -------- helpers --------


// Map 8-neighbor (dx,dy) to a compass angle in degrees.
// We use the standard math convention: +x = 0°, +y = +90°.
inline bool directionAngleDeg(int dx, int dy, double& outDeg){
    // normalize to -1,0,1
    dx = (dx > 0) - (dx < 0);
    dy = (dy > 0) - (dy < 0);
    if (dx == 0 && dy == 0) return false;

    if      (dx ==  1 && dy ==  0) outDeg =   0.0;   // E
    else if (dx ==  1 && dy ==  1) outDeg =  45.0;   // NE
    else if (dx ==  0 && dy ==  1) outDeg =  90.0;   // N
    else if (dx == -1 && dy ==  1) outDeg = 135.0;   // NW
    else if (dx == -1 && dy ==  0) outDeg = 180.0;   // W (same as -180)
    else if (dx == -1 && dy == -1) outDeg = -135.0;  // SW
    else if (dx ==  0 && dy == -1) outDeg =  -90.0;  // S
    else if (dx ==  1 && dy == -1) outDeg =  -45.0;  // SE
    else return false;

    return true;
}

// Turn by multiple ±stepDeg increments to reach target
inline void turnBySteps(double& currentHeadingDeg,
                        double targetHeadingDeg,
                        double stepDeg,
                        const std::function<void(double)>& turnStepDeg)
{
    double err = wrap180(targetHeadingDeg - currentHeadingDeg);
    while (std::fabs(err) > stepDeg / 2.0) {
        double step = std::min(std::fabs(err), stepDeg);
        double signedStep = (err >= 0 ? +step : -step);
        turnStepDeg(signedStep);                 // ← your motor turn here
        currentHeadingDeg = wrap180(currentHeadingDeg + signedStep);
        err = wrap180(targetHeadingDeg - currentHeadingDeg);
    }
}
void turnStepDeg(double angle) {
  const double Kp      = 0.35;  // tune
  const double tolDeg  = 2.0;   // stop within ±2°
  const double minPct  = 8;     // overcome stiction
  const double maxPct  = 50;    // cap speed
  const int    timeout = 2500;  // ms safety

  INS.setRotation(0, degrees);
  wait(50, msec);

  int t = 0;
  while (t < timeout) {
    double err = angle - INS.rotation(degrees); // signed error
    if (fabs(err) <= tolDeg) break;

    double cmd = Kp * err;                      // signed turn command
    // clamp & ensure minimum magnitude
    if (cmd >  maxPct) cmd =  maxPct;
    if (cmd < -maxPct) cmd = -maxPct;
    if (fabs(cmd) < minPct) cmd = (cmd >= 0 ? minPct : -minPct);

    // Mix: positive cmd = turn left (CCW); negative cmd = turn right (CW)
    LeftDrivetrain.spin(fwd,  -cmd, pct);
    RightDrivetrain.spin(fwd,  cmd, pct);

    wait(10, msec);
    t += 10;
  }

  LeftDrivetrain.stop(brake);
  RightDrivetrain.stop(brake);
}

void ReCalibrateGyro()
{
  INS.calibrate();
  while( INS.isCalibrating() )
  { wait(10,msec); }
}




// -------- main routine --------
// cellMm: size of a grid cell in millimeters
// stepDeg: size of your discrete turn steps (e.g., 25°)
inline void navigatePath(const std::vector<CellXY>& path,
                         double cellMm,
                         double initialHeadingDeg,
                         double stepDeg,
                         const std::function<void(double)>& turnStepDeg,
                         const std::function<void(double)>& driveStraightMm)
{
    if (path.size() < 2) return;

    double heading = initialHeadingDeg;

    // compress runs with same (dx,dy)
    size_t i = 0;
    while (i + 1 < path.size()) {
        int dx = path[i+1].x - path[i].x;
        int dy = path[i+1].y - path[i].y;
        // normalize to unit step
        dx = (dx > 0) - (dx < 0);
        dy = (dy > 0) - (dy < 0);
        if (dx == 0 && dy == 0) { ++i; continue; }

        // grow the run while direction stays the same
        size_t j = i + 1;
        while (j + 1 < path.size()) {
            int ndx = (path[j+1].x - path[j].x);
            int ndy = (path[j+1].y - path[j].y);
            ndx = (ndx > 0) - (ndx < 0);
            ndy = (ndy > 0) - (ndy < 0);
            if (ndx != dx || ndy != dy) break;
            ++j;
        }

        // direction → target heading
        double targetDeg = 0.0;
        if (!directionAngleDeg(dx, dy, targetDeg)) return;

        // turn to that heading in ±stepDeg chunks
        turnBySteps(heading, targetDeg, stepDeg, turnStepDeg);

        // drive the run length in mm
        size_t runLenCells = (j - i); // number of steps
        const bool diagonal = (dx != 0 && dy != 0);
        const double perStep = diagonal ? (cellMm * std::sqrt(2.0)) : cellMm;
        driveStraightMm(runLenCells * perStep);//still need to write this one 

        // now we’re aligned to targetDeg
        heading = targetDeg;
        i = j;
    }
}

// ---------------- example wiring (replace with your robot API) ---------------
/*
int main(){
    // Example path:
    std::vector<CellXY> path = {{2,2},{3,3},{4,4},{5,5},{6,5},{7,5},{8,6}};

    double cellMm = 50.0;
    double initialHeadingDeg = 0.0;
    double stepDeg = 25.0;

    auto turnStepDeg = [](double d){
        std::cout << "Turn " << d << " deg\n";
        // robot.turnDegrees(d);
    };
    auto driveStraightMm = [](double mm){
        std::cout << "Drive " << mm << " mm\n";
        // robot.driveForwardMm(mm);
    };

    navigatePath(path, cellMm, initialHeadingDeg, stepDeg, turnStepDeg, driveStraightMm);
}
*/

// --- Example usage -----------------------------------------------------------
// Field field(3600, 1800, 50.0);          // 3.6m x 1.8m, 50mm cells
// auto& grid = field.grid();
// // obstacles (in mm):
// field.addRectMm(0, 0, 400, 400);        // 40x40cm block at center
// field.addEdgeMargin(100.0);             // 10cm safety band
// field.inflateByRadius(150.0);           // robot radius 15cm
//
// // start/goal in cell coords:
// int sx = 2, sy = 2;
// int gx = grid.cols()-3, gy = grid.rows()-3;
// grid.setCell(sx, sy, Cellstate::START);
// grid.setCell(gx, gy, Cellstate::GOAL);
//
// auto path = pathfind::astar(grid, sx, sy, gx, gy);
// pathfind::paintPath(grid, path);
//we can now creat two grids one for the skills and one for 15 sek autonomous mode
/*so to navigate the bot i can just iterate through this array and check what direction it have to go
 turn the bot into the right direction drive the distance of the size of the cells and that i am supposed to
  go into said direction  out of the eight possible directions and checking if the direction of the path changes when it does
 stop and repeat . 
while checking in short time frames check my gps coordinates  and convert them to grid coordinates . */
// 3568 * 3568 feld mm 
//You just need to wire two callbacks:

//turnStepDeg(deltaDeg) → turn the robot by deltaDeg (positive = CCW)

//driveStraightMm(distanceMm) → drive forward that distance