#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <utility>
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