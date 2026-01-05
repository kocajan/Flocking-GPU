/**
 * \file SpatialGrid.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Spatial hashing grid used for neighborhood queries in the sequential backend.
 */

#pragma once

#include <vector>

#include "versions/sequential/SequentialParameters.hpp"

/**
 * \brief Uniform spatial grid structure for accelerating neighbor lookup.
 *
 * The grid partitions the world into cells and stores indices of boids
 * belonging to each cell. It is rebuilt every simulation step.
 */
struct SpatialGrid {

    // Grid configuration
    float cellSize;
    int numCellsX;
    int numCellsY;
    int numCellsZ;

    // Mode flags and state
    bool is2D;
    bool bounce;
    bool hasZeroCells;
    bool isBuilt;

    /**
     * \brief Cell storing indices of objects contained in the cell.
     */
    struct Cell {
        std::vector<int> items;
    };

    // Separate grids for each boid type
    std::vector<Cell> basicCells;
    std::vector<Cell> predatorCells;
    std::vector<Cell> obstacleCells;

    // Temporary buffer used during neighbor queries
    mutable std::vector<int> scratch;

    /**
     * \brief Construct a spatial grid using parameters from the simulation.
     *
     * \param[in] params Precomputed simulation parameters.
     */
    explicit SpatialGrid(const SequentialParameters& params);

    /**
     * \brief Rebuild the grid and insert all boids into cells.
     *
     * \param[in] params Precomputed simulation parameters.
     */
    void build(const SequentialParameters& params);

    /**
     * \brief Retrieve indices of neighboring boids of a given type.
     *
     * Returns indices stored in the scratch buffer.
     *
     * \param[in] params Simulation parameters.
     * \param[in] boidIndex Index of the boid whose neighbors are queried.
     * \param[in] boidType Type of the boid performing the query.
     * \param[in] neighborType Type of neighbors to collect.
     * \return Reference to a vector of neighbor indices.
     */
    const std::vector<int>& getNeighborIndices(const SequentialParameters& params,
                                               int boidIndex,
                                               BoidType boidType,
                                               BoidType neighborType) const;

private:
    // Coordinate mapping helpers
    int flattenIndex(int cx, int cy, int cz) const;
    int worldToCellIndex(float p, float worldSize, int cellCount) const;

    // Boid insertion helpers
    void insertBasicBoids(const SequentialParameters& params);
    void insertPredatorBoids(const SequentialParameters& params);
    void insertObstacleBoids(const SequentialParameters& params);

    void insertPointObject(std::vector<Cell>& grid,
                           const SequentialParameters& params,
                           int boidIndex,
                           Vec3& pos);

    void insertRadialObject(std::vector<Cell>& grid,
                            const SequentialParameters& params,
                            int boidIndex,
                            Vec3& pos,
                            float radius);
};
