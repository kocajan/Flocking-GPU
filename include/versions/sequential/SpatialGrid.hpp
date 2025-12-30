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

    // All boids live in one grid structure
    std::vector<std::vector<int>> cells;

    explicit SpatialGrid(const SequentialParameters& params);

    // Build + populate grid for current frame
    void build(const SequentialParameters& params);

    // Get neighbor boid indices for given boid
    std::unordered_set<int> getNeighborIndices(const SequentialParameters& params, int boidIndex) const;

private:
    // Flatten 3D cell index to 1D 
    int flattenIndex(int cx, int cy, int cz) const;

    // Insert point objects
    void insertPointObject(const SequentialParameters& params, int boidIndex);

    // Insert radial objects
    void insertRadialObject(const SequentialParameters& params, int boidIndex, float radius);

    // Clamp or wrap coordinate based on simulation mode
    int worldToCellIndex(float p, float worldSize, int cellCount) const;
};
