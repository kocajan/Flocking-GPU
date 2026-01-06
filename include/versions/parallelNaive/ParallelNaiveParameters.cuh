/**
 * \file ParallelNaiveParameters.cuh
 * \author Jan Koča
 * \date 05-01-2026
 * \brief CPU–GPU parameter container for the parallel-naive simulation backend.
 *
 * This structure:
 *  - stores host-side scheduling parameters
 *  - stores GPU-side simulation constants and buffers
 *  - prepares data for CUDA kernels
 */

#pragma once

#include <cstdint>
#include <vector>

#include "core/Types.hpp"
#include "core/Boids.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"
#include "core/DeviceStructures.hpp"


struct ParallelNaiveParameters {
    struct GPUParams {
        // Boid data - device pointers
        DeviceBoids dBoids;

        // Interaction
        Interaction interaction;

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
        float baseForce;
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

        // Boid data - host pointers
        Boids &hBoids;
    };

    // CPU scheduling config
    CPUParams cpu;

    // GPU parameter block
    GPUParams gpu;

    /**
     * \brief Construct CPU–GPU parameter state from simulation state and configuration.
     *
     * Initializes derived constants, prepares scheduling parameters,
     * allocates device memory, and uploads boid data to the GPU.
     *
     * \param[in,out] s Simulation state providing source buffers.
     * \param[in] c Configuration object used to derive runtime parameters.
     */
    ParallelNaiveParameters(SimState& s, const Config& c);

    /**
     * \brief Destroy parameter state and synchronize device buffers back to CPU.
     *
     * Copies simulation results from GPU to host memory and frees
     * all allocated device resources.
     */
    ~ParallelNaiveParameters();
};
