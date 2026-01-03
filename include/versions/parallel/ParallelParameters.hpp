#pragma once

#include <cstdint>
#include <vector>

#include "core/Types.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


struct ParallelParameters {
    struct GPUParams {
        // Boid data — device pointers
        DeviceBoids dBoids;

        // Grid - device pointers
        DeviceGrid dGrid;

        // Interaction
        DeviceInteraction dInteraction;
        
        // Simulation flags
        bool is2D;
        bool bounce;

        // Radii
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

        // Neighbor selection
        int maxNeighborsBasic;

        // Cached distances
        float maxDistanceBetweenPoints;
        float maxDistanceBetweenPoints2;
    };

    struct CPUParams {
        // Simulation configuration
        int blockSize;

        // Boid data — device pointers
        Boids &hBoids;
    };

    // CPU scheduling config
    CPUParams cpu;

    // GPU parameter block
    GPUParams gpu;

    // Constructor / destructor
    ParallelParameters(SimState& s, const Config& c);
    ~ParallelParameters();
};
