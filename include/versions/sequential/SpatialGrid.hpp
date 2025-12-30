#pragma once

#include <vector>
#include <unordered_set>

#include "versions/sequential/SequentialParameters.hpp"


struct SpatialGrid {
    float cellSize;
    int cellsX;
    int cellsY;
    int cellsZ;

    bool is2D;
    bool bounce;
    bool hasZeroCells;
    bool isBuilt;

    // Cell structure
    struct Cell {
        std::vector<int> basic;
        std::vector<int> predator;
        std::vector<int> obstacle;
    };

    // 3D grid of cells
    std::vector<Cell> cells;


    explicit SpatialGrid(const SequentialParameters& params);

    // Build + populate grid for current frame
    void build(const SequentialParameters& params);

    // Get neighbor boid indices for given boid
    std::unordered_set<int> getNeighborIndices(const SequentialParameters& params, int boidIndex, BoidType neighborType) const;

private:
    // Flatten 3D cell index to 1D 
    int flattenIndex(int cx, int cy, int cz) const;

    // Insert object into appropriate cell based on its type
    void insertObjectToCell(BoidType type, int boidIndex, Cell& cell);

    // Insert point objects
    void insertPointObject(const SequentialParameters& params, int boidIndex);

    // Insert radial objects
    void insertRadialObject(const SequentialParameters& params, int boidIndex, float radius);

    // Clamp or wrap coordinate based on simulation mode
    int worldToCellIndex(float p, float worldSize, int cellCount) const;
};
