/**
 * \file Boids.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief CPU-side SoA container storing boid population data and lifecycle helpers.
 */

#pragma once

#include <vector>
#include <cstdint>

#include "core/Types.hpp"


/**
 * \struct Boids
 * \brief CPU-side structure-of-arrays container for all boid populations.
 *
 * Contains separate SoA buffers for:
 * - basic boids
 * - predator boids
 * - obstacle boids
 *
 * Provides resize and clear helpers for lifecycle management.
 */
struct Boids {
    // ---------------------------------------------------------------------
    // Core SoA fields — Basic boids
    // ---------------------------------------------------------------------
    std::vector<Vec3> posBasic;
    std::vector<Vec3> velBasic;
    std::vector<Vec3> accBasic;
    std::vector<Vec3> targetPointBasic;

    // ---------------------------------------------------------------------
    // Core SoA fields — Predator boids
    // ---------------------------------------------------------------------
    std::vector<Vec3> posPredator;
    std::vector<Vec3> velPredator;
    std::vector<Vec3> accPredator;
    std::vector<Vec3> targetPointPredator;

    std::vector<float>   staminaPredator;
    std::vector<uint8_t> restingPredator;

    std::vector<int>     targetBoidIdxPredator;
    std::vector<float>   targetBoidDistancePredator;
    std::vector<BoidType> targetBoidTypePredator;

    // ---------------------------------------------------------------------
    // Core SoA fields — Obstacle boids
    // ---------------------------------------------------------------------
    std::vector<Vec3> posObstacle;

    // ---------------------------------------------------------------------
    // Bookkeeping by boid type
    // ---------------------------------------------------------------------
    int basicBoidCount = 0;
    int predatorBoidCount = 0;
    int obstacleBoidCount = 0;

    // ---------------------------------------------------------------------
    // Lifecycle helpers
    // ---------------------------------------------------------------------

    /**
     * \brief Resize all basic-boid attribute arrays.
     *
     * \param[in] n New number of basic boids.
     */
    void resizeBasic(int n) {
        basicBoidCount = n;

        posBasic.resize(n);
        velBasic.resize(n);
        accBasic.resize(n);
        targetPointBasic.resize(n);
    }

    /**
     * \brief Resize all predator-boid attribute arrays.
     *
     * \param[in] n New number of predator boids.
     */
    void resizePredator(int n) {
        predatorBoidCount = n;

        posPredator.resize(n);
        velPredator.resize(n);
        accPredator.resize(n);
        targetPointPredator.resize(n);

        staminaPredator.resize(n);
        restingPredator.resize(n);

        targetBoidIdxPredator.resize(n);
        targetBoidDistancePredator.resize(n);
        targetBoidTypePredator.resize(n);
    }

    /**
     * \brief Resize all obstacle-boid attribute arrays.
     *
     * \param[in] n New number of obstacle boids.
     */
    void resizeObstacle(int n) {
        obstacleBoidCount = n;

        posObstacle.resize(n);
    }

    /**
     * \brief Clear all boid storage and reset counts to zero.
     */
    void clear() {
        posBasic.clear();
        velBasic.clear();
        accBasic.clear();
        targetPointBasic.clear();

        posPredator.clear();
        velPredator.clear();
        accPredator.clear();
        targetPointPredator.clear();
        staminaPredator.clear();
        restingPredator.clear();
        targetBoidIdxPredator.clear();
        targetBoidDistancePredator.clear();
        targetBoidTypePredator.clear();

        posObstacle.clear();

        basicBoidCount = 0;
        predatorBoidCount = 0;
        obstacleBoidCount = 0;
    }
};
