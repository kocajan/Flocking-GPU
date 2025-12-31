#pragma once

#include <cstdint>
#include <vector>

#include "core/Types.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


struct ParallelNaiveParameters {

    //
    // =====================================================================
    //  GPU parameter block (POD — copied to device memory)
    // =====================================================================
    //
    //  Only contains:
    //   - primitive values
    //   - raw pointers into SoA arrays
    //
    struct GPUParams {
        // Boid data — device pointers
        DeviceBoids dBoids;

        // Boid counts
        int dBoidCount;

        // Simulation flags
        bool dIs2D;
        bool dBounce;

        // Radii
        float dBasicBoidRadius;
        float dPredatorRadius;
        float dObstacleRadius;

        // World bounds
        float dWorldX;
        float dWorldY;
        float dWorldZ;

        // Constants
        float dEps;
        float dDt;

        // Vision
        float dVisionRangeBasic;
        float dVisionRangeBasic2;
        float dVisionRangePredator;

        // Forces / multipliers
        float dMaxForce;
        float dObstacleAvoidanceMultiplier;
        float dMouseInteractionMultiplier;

        // Flocking weights
        float dCohesionWeightBasic;
        float dAlignmentWeightBasic;
        float dSeparationWeightBasic;
        float dTargetAttractionWeightBasic;

        // Speeds (basic)
        float dCruisingSpeedBasic;
        float dMaxSpeedBasic;
        float dMinSpeedBasic;

        // Speeds (predator)
        float dCruisingSpeedPredator;
        float dMaxSpeedPredator;
        float dMinSpeedPredator;

        // Predator stamina
        float dMaxStaminaPredator;
        float dStaminaRecoveryRatePredator;
        float dStaminaDrainRatePredator;

        // Dynamics
        float dDrag;
        float dNoise;
        float dNumStepsToStopDueToMaxDrag;

        // Collision / bounce
        float dBounceFactor;

        // Neighbor selection
        float dMaxNeighborsBasic;

        // Cached distances
        float dMaxDistanceBetweenPoints;
        float dMaxDistanceBetweenPoints2;

        // Interaction
        DeviceInteraction dInteraction;
    };


    //
    // =====================================================================
    //  CPU Parameters
    // =====================================================================
    //
    struct CPUParams {
        // Simulation configuration
        int blockSize = 256;
        int gridSize  = 1;

        // Boid data — device pointers
        Boids &hBoids;

        // Boid counts
        int hBoidCount;

        // Simulation flags
        bool hIs2D;
        bool hBounce;

        // Radii
        float hBasicBoidRadius;
        float hPredatorRadius;
        float hObstacleRadius;

        // World bounds
        float hWorldX;
        float hWorldY;
        float hWorldZ;

        // Constants
        float hEps;
        float hDt;

        // Vision
        float hVisionRangeBasic;
        float hVisionRangeBasic2;
        float hVisionRangePredator;

        // Forces / multipliers
        float hMaxForce;
        float hObstacleAvoidanceMultiplier;
        float hMouseInteractionMultiplier;

        // Flocking weights
        float hCohesionWeightBasic;
        float hAlignmentWeightBasic;
        float hSeparationWeightBasic;
        float hTargetAttractionWeightBasic;

        // Speeds (basic)
        float hCruisingSpeedBasic;
        float hMaxSpeedBasic;
        float hMinSpeedBasic;

        // Speeds (predator)
        float hCruisingSpeedPredator;
        float hMaxSpeedPredator;
        float hMinSpeedPredator;

        // Predator stamina
        float hMaxStaminaPredator;
        float hStaminaRecoveryRatePredator;
        float hStaminaDrainRatePredator;

        // Dynamics
        float hDrag;
        float hNoise;
        float hNumStepsToStopDueToMaxDrag;

        // Collision / bounce
        float hBounceFactor;

        // Neighbor selection
        float hMaxNeighborsBasic;

        // Cached distances
        float hMaxDistanceBetweenPoints;
        float hMaxDistanceBetweenPoints2;

        // Interaction
        Interaction hInteraction;
    };

    // CPU scheduling config
    CPUParams cpu;

    // GPU parameter block (to be uploaded)
    GPUParams gpu;


    //
    // Constructor — builds cpu + gpu views from SimState + Config
    //
    ParallelNaiveParameters(SimState& s, const Config& c);
};
