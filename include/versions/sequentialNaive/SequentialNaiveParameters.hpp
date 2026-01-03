#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

#include "core/Types.hpp"
#include "config/Config.hpp"
#include "core/SimState.hpp"


struct SequentialNaiveParameters {
    // Reference to boid array
    Boids& boids;

    // Interaction
    Interaction interaction;

    // Simulation modes
    bool is2D;
    bool bounce;

    // Boid radii
    float basicBoidRadius;
    float predatorBoidRadius;
    float obstacleBoidRadius;

    // World bounds
    float worldX;
    float worldY;
    float worldZ;

    // Constants
    float eps;
    float dt;

    // Vision
    float visionRangeBasic;
    float visionRangeBasic2;
    float visionRangePredator;

    // Forces / multipliers
    float maxForce;
    float obstacleAvoidanceMultiplier;
    float mouseInteractionMultiplier;

    // Flocking weights
    float cohesionWeightBasic;
    float alignmentWeightBasic;
    float separationWeightBasic;
    float targetAttractionWeightBasic;

    // Speeds (basic)
    float cruisingSpeedBasic;
    float maxSpeedBasic;
    float minSpeedBasic;

    // Speeds (predator)
    float cruisingSpeedPredator;
    float maxSpeedPredator;
    float minSpeedPredator;

    // Predator stamina
    float maxStaminaPredator;
    float staminaRecoveryRatePredator;
    float staminaDrainRatePredator;

    // Dynamics
    float drag;
    float noise;
    float numStepsToStopDueToMaxDrag;

    // Collision / bounce
    float bounceFactor;

    // Neighbor limit
    float maxNeighborsBasic;

    // Cached distances
    float maxDistanceBetweenPoints;
    float maxDistanceBetweenPoints2;

    // Constructor / destructor
    SequentialNaiveParameters(SimState& s, const Config& c);
    ~SequentialNaiveParameters(); 
};
