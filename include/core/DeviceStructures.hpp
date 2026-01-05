/**
 * \file DeviceStructures.hpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief GPU-side mirror structures for simulation data stored in device memory.
 *
 * These structures correspond to CPU SoA containers but store raw pointers
 * to GPU buffers allocated and managed by compute backends.
 */

#pragma once

#include "core/Types.hpp"

/**
 * \struct DeviceBoids
 * \brief GPU-side mirror of boid population storage (SoA layout).
 *
 * Contains raw device pointers for each attribute array and
 * per-type boid population counts.
 */
struct DeviceBoids {
    // Core SoA fields
    // - Basic boids
    Vec3* posBasic = nullptr;          ///< Positions of basic boids.
    Vec3* velBasic = nullptr;          ///< Velocities of basic boids.
    Vec3* accBasic = nullptr;          ///< Accelerations of basic boids.
    Vec3* targetPointBasic = nullptr;  ///< Target points of basic boids.

    // - Predator boids
    Vec3* posPredator = nullptr;            ///< Positions of predator boids.
    Vec3* velPredator = nullptr;            ///< Velocities of predator boids.
    Vec3* accPredator = nullptr;            ///< Accelerations of predator boids.
    Vec3* targetPointPredator = nullptr;    ///< Target points of predator boids.

    float* staminaPredator = nullptr;       ///< Predator stamina values.
    uint8_t* restingPredator = nullptr;     ///< Predator resting flags.

    int*   targetBoidIdxPredator = nullptr;      ///< Predator target index.
    float* targetBoidDistancePredator = nullptr; ///< Distance to predator target.
    uint8_t* targetBoidTypePredator = nullptr;   ///< Target boid type id.

    // - Obstacle boids
    Vec3* posObstacle = nullptr; ///< Positions of obstacle boids.

    // Bookkeeping by boid type
    int basicBoidCount = 0;    ///< Number of basic boids.
    int predatorBoidCount = 0; ///< Number of predator boids.
    int obstacleBoidCount = 0; ///< Number of obstacle boids.
};

/**
 * \struct DeviceGrid
 * \brief GPU-side grid and spatial hashing buffers for neighbor search.
 */
struct DeviceGrid {
    // Basic boid grid
    int* hashBasic;
    int* indexBasic;

    int* cellStartBasic;
    int* cellEndBasic;

    // Predator boid grid
    int* hashPredator;
    int* indexPredator;

    int* cellStartPredator;
    int* cellEndPredator;

    // Obstacle boid grid
    int* hashObstacle;
    int* indexObstacle;

    int* cellStartObstacle;
    int* cellEndObstacle;

    // Spatial partitioning
    // - Cell grid coordinates
    int* cellX;
    int* cellY;
    int* cellZ;

    float cellSize;

    int numCellsX;
    int numCellsY;
    int numCellsZ;

    int totalCells;
};
