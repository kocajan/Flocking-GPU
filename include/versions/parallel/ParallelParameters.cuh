/**
 * \file ParallelParameters.cuh
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Runtime parameter container for the optimized parallel GPU simulation.
 *
 * This structure groups:
 *  - CPU scheduling configuration
 *  - GPU runtime parameters
 *  - device boid buffers
 *  - spatial grid data
 *
 * Values are initialized from SimState + Config and transferred to GPU memory.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "core/Types.hpp"
#include "core/Boids.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"
#include "core/DeviceStructures.hpp"


/**
 * \brief Combined CPU+GPU parameter bundle for the optimized parallel simulation.
 *
 * CPUParams:
 *  - scheduling configuration
 *  - reference to host boid buffers
 *
 * GPUParams:
 *  - device boid arrays
 *  - spatial grid buffers
 *  - simulation constants and flags
 *  - vision, dynamics, and interaction parameters
 */
struct ParallelParameters {
    struct GPUParams {
        // Boid data — device pointers
        DeviceBoids dBoids;

        // Grid - device pointers
        DeviceGrid dGrid;

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

        // Boid data — host reference
        Boids &hBoids;
    };

    // CPU scheduling config
    CPUParams cpu;

    // GPU parameter block
    GPUParams gpu;

    /**
     * \brief Construct parallel simulation parameters and allocate GPU resources.
     *
     * Responsibilities:
     *  - read simulation constants and flags from SimState + Config
     *  - compute normalized weights and derived world metrics
     *  - allocate device memory for boid arrays
     *  - allocate spatial grid buffers
     *  - copy initial boid data from host to device
     *
     * \param[in,out] s Simulation state containing host boid buffers.
     * \param[in] c Configuration source (numeric + binary parameters).
     */
    ParallelParameters(SimState& s, const Config& c);

    /**
     * \brief Destroy parameter object and release GPU resources.
     *
     * Responsibilities:
     *  - copy boid data back from device to host
     *  - free all device boid buffers
     *  - free spatial grid buffers
     */
    ~ParallelParameters();
};
