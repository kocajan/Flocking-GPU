#pragma once

#include <vector>

#include "versions/sequential/SequentialParameters.hpp"

struct SpatialGrid
{
    float cellSize;
    int cellsX;
    int cellsY;
    int cellsZ;

    bool is2D;
    bool bounce;
    bool hasZeroCells;
    bool isBuilt;

    struct Cell {
        std::vector<int> items;
    };

    // Three independent grids
    std::vector<Cell> basicCells;
    std::vector<Cell> predatorCells;
    std::vector<Cell> obstacleCells;

    // Scratch buffer returned by getNeighborIndices()
    mutable std::vector<int> scratch;

    explicit SpatialGrid(const SequentialParameters& params);

    void build(const SequentialParameters& params);

    // Query neighbors of a given type (same external API)
    const std::vector<int>& getNeighborIndices(
        const SequentialParameters& params,
        int boidIndex,
        BoidType neighborType
    ) const;

private:

    int flattenIndex(int cx, int cy, int cz) const;

    int worldToCellIndex(float p, float worldSize, int cellCount) const;

    void insertPointObject(
        std::vector<Cell>& grid,
        const SequentialParameters& params,
        int boidIndex
    );

    void insertRadialObject(
        std::vector<Cell>& grid,
        const SequentialParameters& params,
        int boidIndex,
        float radius
    );
};
