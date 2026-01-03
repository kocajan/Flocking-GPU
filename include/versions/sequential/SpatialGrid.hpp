#pragma once

#include <vector>

#include "versions/sequential/SequentialParameters.hpp"


struct SpatialGrid {
    float cellSize;
    int numCellsX;
    int numCellsY;
    int numCellsZ;

    bool is2D;
    bool bounce;
    bool hasZeroCells;
    bool isBuilt;

    struct Cell {
        std::vector<int> items;
    };

    // Grids for each boid type
    std::vector<Cell> basicCells;
    std::vector<Cell> predatorCells;
    std::vector<Cell> obstacleCells;

    // Scratch buffer for neighbor queries
    mutable std::vector<int> scratch;

    explicit SpatialGrid(const SequentialParameters& params);

    void build(const SequentialParameters& params);

    // Get neighbor boid indices of specified type for given boid
    const std::vector<int>& getNeighborIndices(const SequentialParameters& params, 
                                               int boidIndex, BoidType boidType,
                                               BoidType neighborType) const;

private:
    int flattenIndex(int cx, int cy, int cz) const;
    int worldToCellIndex(float p, float worldSize, int cellCount) const;

    void insertBasicBoids(const SequentialParameters& params);
    void insertPredatorBoids(const SequentialParameters& params);
    void insertObstacleBoids(const SequentialParameters& params);

    void insertPointObject(std::vector<Cell>& grid, const SequentialParameters& params, int boidIndex, Vec3& pos);
    void insertRadialObject(std::vector<Cell>& grid, const SequentialParameters& params, int boidIndex, Vec3& pos, float radius);
};
