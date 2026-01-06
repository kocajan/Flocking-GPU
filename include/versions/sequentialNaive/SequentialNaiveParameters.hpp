/**
 * \file SequentialNaiveParameters.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Parameter container for the sequential naive simulation backend.
 */

#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

#include "core/Types.hpp"
#include "core/Boids.hpp"
#include "config/Config.hpp"
#include "core/SimState.hpp"

/**
 * \brief Aggregates simulation constants and derived runtime parameters.
 *
 * Parameters are computed from the configuration and simulation state and
 * reused throughout the sequential naive simulation implementation.
 */
struct SequentialNaiveParameters {

    // Simulation data references and mode flags
    Boids& boids;
    Interaction interaction;
    bool is2D;
    bool bounce;

    // Boid radii
    float basicBoidRadius;
    float predatorBoidRadius;
    float obstacleBoidRadius;

    // World dimensions
    float worldX;
    float worldY;
    float worldZ;

    // Core numeric constants
    float eps;
    float dt;

    // Vision ranges
    float visionRangeBasic;
    float visionRangeBasic2;
    float visionRangePredator;

    // Force and interaction multipliers
    float baseForce;
    float obstacleAvoidanceMultiplier;
    float mouseInteractionMultiplier;

    // Flocking behavior weights
    float cohesionWeightBasic;
    float alignmentWeightBasic;
    float separationWeightBasic;
    float targetAttractionWeightBasic;

    // Basic boid speed limits
    float cruisingSpeedBasic;
    float maxSpeedBasic;
    float minSpeedBasic;

    // Predator speed limits
    float cruisingSpeedPredator;
    float maxSpeedPredator;
    float minSpeedPredator;

    // Predator stamina parameters
    float maxStaminaPredator;
    float staminaRecoveryRatePredator;
    float staminaDrainRatePredator;

    // Motion dynamics
    float drag;
    float noise;
    float numStepsToStopDueToMaxDrag;

    // Boundary interaction parameters
    float bounceFactor;

    // Neighbor evaluation limit
    float maxNeighborsBasic;

    // Cached geometric distance bounds
    float maxDistanceBetweenPoints;
    float maxDistanceBetweenPoints2;

    /**
     * \brief Construct parameter set from simulation state and configuration.
     *
     * \param[in] s Simulation state object.
     * \param[in] c Configuration source.
     */
    SequentialNaiveParameters(SimState& s, const Config& c);

    /**
     * \brief Destructor.
     */
    ~SequentialNaiveParameters();
};
