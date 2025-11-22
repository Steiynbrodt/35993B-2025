#pragma once
#include <array>
#include "vex.h"
#include <cstdint>
#include <vector>
#include <queue>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <limits>   
#include <cassert>


enum class Cellstate : unsigned char {
    FREE = 0,
    OBSTACLE = 1,
    START = 2,
    GOAL = 3,
    PATH = 4,
    INFLATED = 5,
};

struct Cell
{
    Cellstate state = Cellstate::FREE;

    bool isFree() const {
        return state == Cellstate::FREE;
    }
    void setObstacle() {
        state = Cellstate::OBSTACLE;
    }
    void setStart() {
        state = Cellstate::START;
    }
    void setGoal() {
        state = Cellstate::GOAL;
    }
    void setPath() {
        state = Cellstate::PATH;
    }
    void reset() {
        state = Cellstate::FREE;
    }
    /* data */
};

class Grid {
public:
    Grid(int cols, int rows)
        : cols_(cols), rows_(rows), data_(static_cast<size_t>(cols) * rows) {
       // if (cols_ <= 0 || rows_ <= 0) throw std::invalid_argument("Grid size must be > 0");
    }

    int cols() const { return cols_; }
    int rows() const { return rows_; }

    bool inBounds(int x, int y) const {
        return (x >= 0 && x < cols_ && y >= 0 && y < rows_);
    }

    Cell& at(int x, int y) {
        return data_[index(x, y)];
    }
    const Cell& at(int x, int y) const {
        return data_[index(x, y)];
    }

    void fill(Cellstate s) {
        for (auto& c : data_) c.state = s;
    }

    // ---- Simple drawing helpers (cell units) ----
    void setCell(int x, int y, Cellstate s) {
        if (!inBounds(x, y)) return;
        at(x, y).state = s;
    }

    // Inclusive rectangle [x0..x1], [y0..y1]
    void fillRect(int x0, int y0, int x1, int y1, Cellstate s) {
        if (x0 > x1)  std::swap(x0, x1);
        if (y0 > y1) std::swap(y0, y1);
        x0 = std::max(0, x0); y0 = std::max(0, y0);
        x1 = std::min(cols_ - 1, x1); y1 = std::min(rows_ - 1, y1);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                at(x, y).state = s;
    }

    // Disk centered at (cx,cy) with radius in cells
    void fillDisk(int cx, int cy, double radiusCells, Cellstate s) {
        if (radiusCells < 0) return;
        int r = static_cast<int>(std::ceil(radiusCells));
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                int x = cx + dx, y = cy + dy;
                if (!inBounds(x, y)) continue;
                double d = std::sqrt(static_cast<double>(dx*dx + dy*dy));
                if (d <= radiusCells + 1e-9) at(x, y).state = s;
            }
        }
    }

    // Inflate all non-free cells by a radius (cells) into state Inflated
    void inflate(double radiusCells) {
        if (radiusCells <= 0) return;
        std::vector<Cell> out = data_; // start with a copy
        int r = static_cast<int>(std::ceil(radiusCells));
        for (int y = 0; y < rows_; ++y) {
            for (int x = 0; x < cols_; ++x) {
                if (at(x, y).isFree()) continue;
                // stamp a disk of Inflated around obstacles
                for (int dy = -r; dy <= r; ++dy) {
                    for (int dx = -r; dx <= r; ++dx) {
                        int nx = x + dx, ny = y + dy;
                        if (!inBounds(nx, ny)) continue;
                        double d = std::sqrt(static_cast<double>(dx*dx + dy*dy));
                        if (d <= radiusCells + 1e-9)
                            out[static_cast<size_t>(ny) * cols_ + nx].state = Cellstate::INFLATED;
                    }
                }
            }
        }
        // Any original obstacles stay obstacles; inflated areas become Inflated if they were Free.
        for (size_t i = 0; i < data_.size(); ++i) {
            if (data_[i].state == Cellstate::OBSTACLE) out[i].state = Cellstate::OBSTACLE;
        }
        data_.swap(out);
    }

private:
    int cols_;
    int rows_;
    std::vector<Cell> data_;

    size_t index(int x, int y) const {
        // Single-row-major index
        return static_cast<size_t>(y) * cols_ + x;
    }
};

class Field {
public:
    // Field is length_mm (X) by width_mm (Y); cellSizeMm controls grid resolution.
    Field(int length_mm, int width_mm, double cellSizeMm)
        : lengthMm_(length_mm), widthMm_(width_mm), cellMm_(cellSizeMm),
          cols_(static_cast<int>(std::ceil(length_mm / cellSizeMm))),
          rows_(static_cast<int>(std::ceil(width_mm  / cellSizeMm))),
          grid_(cols_, rows_) {
       // if (length_mm <= 0 || width_mm <= 0 || cellSizeMm <= 0.0)
           // throw std::invalid_argument("Invalid field or cell size.");
    }

    // Dimensions
    int lengthMm() const { return lengthMm_; }
    int widthMm()  const { return widthMm_;  }
    double cellMm() const { return cellMm_;  }
    int cols() const { return cols_; }
    int rows() const { return rows_; }

    // Access to the grid
    Grid& grid() { return grid_; }
    const Grid& grid() const { return grid_; }

      bool mmToCell(double xMm, double yMm, int& outX, int& outY) const {
        // shift to [0..length), [0..width)
        double sx = xMm + 0.5 * lengthMm_;
        double sy = yMm + 0.5 * widthMm_;
        if (sx < 0.0 || sy < 0.0 || sx >= lengthMm_ || sy >= widthMm_) return false;
        outX = static_cast<int>(std::floor(sx / cellMm_));
        outY = static_cast<int>(std::floor(sy / cellMm_));
        return grid_.inBounds(outX, outY);
    }

    // Cell center (mm) from cell indices
    void cellToMmCenter(int cx, int cy, double& outXmm, double& outYmm) const {
       // if (!grid_.inBounds(cx, cy)) throw std::out_of_range("cellToMmCenter: out of bounds");
        double sx = (cx + 0.5) * cellMm_;
        double sy = (cy + 0.5) * cellMm_;
        outXmm = sx - 0.5 * lengthMm_;
        outYmm = sy - 0.5 * widthMm_;
    }

    // ---- Obstacle helpers in MM ----
    void addRectMm(double centerXmm, double centerYmm, double sizeXmm, double sizeYmm, Cellstate s = Cellstate::OBSTACLE) {
        // Convert mm box to cell box and fill
        double xMin = centerXmm - 0.5 * sizeXmm;
        double xMax = centerXmm + 0.5 * sizeXmm;
        double yMin = centerYmm - 0.5 * sizeYmm;
        double yMax = centerYmm + 0.5 * sizeYmm;

        int cx0, cy0, cx1, cy1;
        clampMmToCell(xMin, yMin, cx0, cy0);
        clampMmToCell(xMax, yMax, cx1, cy1);
        if (cx0 > cx1) std::swap(cx0, cx1);
        if (cy0 > cy1) std::swap(cy0, cy1);
        grid_.fillRect(cx0, cy0, cx1, cy1, s);
    }

    void addDiskMm(double centerXmm, double centerYmm, double radiusMm, Cellstate s = Cellstate::OBSTACLE) {
        int ccx, ccy;
        if (!clampMmToCell(centerXmm, centerYmm, ccx, ccy)) return;
        double rCells = radiusMm / cellMm_;
        grid_.fillDisk(ccx, ccy, rCells, s);
    }

    // Reserve a safety band around the edge (in mm)
    void addEdgeMargin(double marginMm, Cellstate s = Cellstate::OBSTACLE) {
        if (marginMm <= 0) return;
        int m = static_cast<int>(std::ceil(marginMm / cellMm_));
        grid_.fillRect(0, 0, cols_ - 1, std::min(rows_ - 1, m - 1), s);                   // bottom
        grid_.fillRect(0, std::max(0, rows_ - m), cols_ - 1, rows_ - 1, s);               // top
        grid_.fillRect(0, 0, std::min(cols_ - 1, m - 1), rows_ - 1, s);                   // left
        grid_.fillRect(std::max(0, cols_ - m), 0, cols_ - 1, rows_ - 1, s);               // right
    }

    // Inflate by robot radius (mm) → marks cells as Inflated (obstacles remain Obstacle)
    void inflateByRadius(double robotRadiusMm) {
        grid_.inflate(robotRadiusMm / cellMm_);
    }

private:
    int lengthMm_;
    int widthMm_;
    double cellMm_; 
    int cols_;
    int rows_;
    Grid grid_;

    // Clamp an mm point to the field bounds and convert to cell indices
    bool clampMmToCell(double xMm, double yMm, int& outX, int& outY) const {
        double cx = std::min(std::max(xMm, -0.5 * static_cast<double>(lengthMm_)),  0.5 * static_cast<double>(lengthMm_) - 1e-9);
        double cy = std::min(std::max(yMm, -0.5 * static_cast<double>(widthMm_)),   0.5 * static_cast<double>(widthMm_)  - 1e-9);
        return mmToCell(cx, cy, outX, outY);
    }
};
constexpr double BLOCK_RADIUS_MM = 100.0;


inline void clearDynamicGridStates(Field& field) {
  Grid& g = field.grid();
  for (int y = 0; y < g.rows(); ++y) {
    for (int x = 0; x < g.cols(); ++x) {
      Cell& c = g.at(x, y);
      if (c.state == Cellstate::START ||
          c.state == Cellstate::GOAL  ||
          c.state == Cellstate::PATH) {
        c.state = Cellstate::FREE;
      }
    }
  }
}

inline void addFieldObstaclesWithSmallX(Field& field) {
    // ---- Walls ----
   

    // ---- Long-goal bars ----
    field.addRectMm(0.0,  1200.0,1270.0, 250.0);   // top bar
    field.addRectMm(0.0, -1200.0, 1270.0, 250.0);   // bottom bar
    field.addRectMm(-1500, 0.0, 300.0,300.0);
    field.addRectMm(1500, 0.0, 300.0,300.0);

    // ---- Corner bays ----
    //field.addRectMm(-1700.0, 0.0, 300.0, 800.0);   // left bay
   // field.addRectMm( 1700.0, 0.0, 300.0, 800.0);   // right bay

    // ---- Small diagonal X (±300mm) ----
    constexpr double R = 200.0;   // disk radius in mm

    const double centers[][2] = {
        //{-300.0, -300.0},
       // {-300.0,  300.0},
        {-150.0, -150.0},
        {-150.0,  150.0},
       // {-200.0, -200.0},
       // {-200.0,  200.0},
        {   0.0,    0.0},
        { 150.0,  150.0},
        { 150.0, -150.0},
        // { 200.0,  200.0},
        //{ 200.0, -200.0},
        //{ 300.0,  300.0},
        //{ 300.0, -300.0}
    };

    for (auto &c : centers)
        field.addDiskMm(c[0], c[1], R);


        field.inflateByRadius(150);
    // Important:
    // DO NOT call field.inflateByRadius() when using these precise shapes!
    
}
