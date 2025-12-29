#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

#include "core/Types.hpp"
#include "config/Config.hpp"
#include "core/SimState.hpp"

// Container for cached frame parameters used by SequentialNaive simulation.
struct SequentialNaiveParameters
{
    // Reference to boid array
    std::vector<Boid>& boids;

    // References to boid index arrays
    std::vector<size_t>& obstacleBoidIndices;
    std::vector<size_t>& basicBoidIndices;
    std::vector<size_t>& predatorBoidIndices;

    // Interaction
    Interaction interaction;

    // Boid count
    int boidCount;

    // 
    bool is2D;
    bool bounce;

    // --- radii ---
    float basicBoidRadius;
    float predatorRadius;
    float obstacleRadius;

    // --- world bounds ---
    float worldX;
    float worldY;
    float worldZ;

    // --- constants ---
    float eps;
    float dt;

    // --- vision ---
    float visualRangeBasic;
    float visualRangeBasic2;
    float visualRangePredator;

    // --- forces / multipliers ---
    float maxForce;
    float obstacleAvoidanceMultiplier;
    float mouseInteractionMultiplier;

    // --- flocking weights ---
    float cohesionWeightBasic;
    float alignmentWeightBasic;
    float separationWeightBasic;
    float targetAttractionWeightBasic;

    // --- speeds (basic) ---
    float cruisingSpeedBasic;
    float maxSpeedBasic;
    float minSpeedBasic;

    // --- speeds (predator) ---
    float cruisingSpeedPredator;
    float maxSpeedPredator;
    float minSpeedPredator;

    // --- predator stamina ---
    float maxStaminaPredator;
    float staminaRecoveryRatePredator;
    float staminaDrainRatePredator;

    // --- dynamics ---
    float drag;
    float noise;
    float numStepsToStopDueToMaxDrag;

    // --- collision / bounce ---
    float bounceFactor;

    // --- neighbor limit ---
    float maxNeighborsBasic;

    // --- cached distances ---
    float maxDistanceBetweenPoints;
    float maxDistanceBetweenPoints2;

    // --- ctor computes all cached values ---
    SequentialNaiveParameters(SimState& s, const Config& c);
};
