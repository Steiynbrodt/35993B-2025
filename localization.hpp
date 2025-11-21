#pragma once
#include "vex.h"
//#include "drive.hpp"            // your GPS device (e.g., GPS17)
//#include "fieldparameters.hpp"  // Cell/Cellstate, Grid, Field
//#include "helpers.hpp"          // (optional) angle helpers
#include <queue>
#include <vector>
#include <utility>
#include "fieldparameters.hpp" 
// ---- Data returned for starting pose in grid space ----
struct StartPoseCell {
    int sx{-1}, sy{-1};
    double headingDeg{0.0};   // field-relative heading in degrees (−180..+180]
    bool valid{false};
};

// Traversable = FREE/START/GOAL (matches your A* logic)
inline bool isTraversableCell(const Cell& c){
    return c.state == Cellstate::FREE ||
           c.state == Cellstate::START ||
           c.state == Cellstate::GOAL;
}

// BFS from (cx,cy) to nearest traversable cell (handles starting in inflated/blocked cells)
inline bool nearestTraversable(const Grid& g, int cx, int cy, int& outX, int& outY){
    if (!g.inBounds(cx, cy)) return false;
    const int C = g.cols(), R = g.rows();

    std::vector<char> seen(static_cast<size_t>(C) * R, 0);
    auto idx = [C](int x, int y){ return y * C + x; };

    std::queue<std::pair<int,int>> q;
    q.push({cx,cy});
    seen[idx(cx,cy)] = 1;

    static const int DX[8] = {+1,-1, 0, 0, +1,+1,-1,-1};
    static const int DY[8] = { 0, 0,+1,-1, +1,-1,+1,-1};

    while(!q.empty()){
        auto [x,y] = q.front(); q.pop();
        if (isTraversableCell(g.at(x,y))) {
            outX = x; outY = y;
            return true;
        }
        for (int k=0;k<8;++k){
            int nx = x + DX[k], ny = y + DY[k];
            if (!g.inBounds(nx,ny)) continue;
            int nidx = idx(nx,ny);
            if (seen[nidx]) continue;
            seen[nidx] = 1;
            q.push({nx,ny});
        }
    }
    return false;
}

// Read VEX GPS → convert to grid cell → snap to nearest traversable cell; also return heading.
inline StartPoseCell getStartFromGPS(const Field& field){
    StartPoseCell out;
    double xMm = GPS17.xPosition(vex::distanceUnits::mm);
    double yMm = GPS17.yPosition(vex::distanceUnits::mm);

    int cx, cy;
    if (!field.mmToCell(xMm, yMm, cx, cy)) return out; // invalid

    out.sx = cx;
    out.sy = cy;
    double h = GPS17.heading(vex::rotationUnits::deg);
    while (h > 180.0) h -= 360.0;
    while (h <= -180.0) h += 360.0;
    out.headingDeg = h;
    out.valid = true;
    return out;
}

