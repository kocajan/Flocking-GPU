#include <algorithm>
#include <cmath>

#include "versions/parallelNaive/ParallelNaiveParameters.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


ParallelNaiveParameters::ParallelNaiveParameters(SimState& s, const Config& c)
    : cpu{.hBoids = s.boids} {
        
    Boids& boids = s.boids;

    //
    // ================================================================
    // CPU-side launch defaults
    // ================================================================
    //
    cpu.blockSize = 256;
    cpu.hBoidCount = static_cast<int>(boids.count);

    cpu.gridSize =
        (cpu.hBoidCount + cpu.blockSize - 1) / cpu.blockSize;


    //
    // ================================================================
    // Store raw HOST pointers in CPU params
    // ================================================================
    //
    cpu.hBoids = boids;


    //
    // ================================================================
    // Flags
    // ================================================================
    //
    cpu.hIs2D   = (s.dimensions.string() == "2D");
    cpu.hBounce = c.binary("bounce");
    cpu.hBounceFactor = c.number("bounceFactor") / 100.0f;


    //
    // Radii
    //
    cpu.hBasicBoidRadius = s.basicBoidRadius.number();
    cpu.hPredatorRadius  = s.predatorRadius.number();
    cpu.hObstacleRadius  = s.obstacleRadius.number();


    //
    // World bounds
    //
    cpu.hWorldX = s.worldX.number();
    cpu.hWorldY = s.worldY.number();
    cpu.hWorldZ = s.worldZ.number();

    float longestWorldEdge =
        std::max({ cpu.hWorldX,
                   cpu.hWorldY,
                   cpu.hIs2D ? 0.0f : cpu.hWorldZ });

    float worldDiag = cpu.hIs2D
        ? std::sqrt(cpu.hWorldX*cpu.hWorldX + cpu.hWorldY*cpu.hWorldY)
        : std::sqrt(cpu.hWorldX*cpu.hWorldX +
                    cpu.hWorldY*cpu.hWorldY +
                    cpu.hWorldZ*cpu.hWorldZ);

    cpu.hMaxDistanceBetweenPoints =
        cpu.hBounce ? worldDiag * 0.5f : worldDiag;

    cpu.hMaxDistanceBetweenPoints2 =
        cpu.hMaxDistanceBetweenPoints *
        cpu.hMaxDistanceBetweenPoints;


    //
    // Core constants
    //
    cpu.hEps = s.eps.number();
    cpu.hDt  = s.dt.number();


    //
    // Vision
    //
    cpu.hVisionRangeBasic =
        worldDiag * (c.number("visionBasic") / 100.0f);

    cpu.hVisionRangePredator =
        worldDiag * (c.number("visionPredator") / 100.0f);

    cpu.hVisionRangeBasic2 =
        cpu.hVisionRangeBasic * cpu.hVisionRangeBasic;


    //
    // Multipliers
    //
    cpu.hObstacleAvoidanceMultiplier = c.number("obstacleAvoidanceMultiplier");
    cpu.hMouseInteractionMultiplier  = c.number("mouseInteractionMultiplier");


    //
    // Flocking weights
    //
    cpu.hCohesionWeightBasic =
        std::clamp(c.number("cohesionBasic") / 100.0f, 0.0f, 1.0f);

    cpu.hAlignmentWeightBasic =
        std::clamp(c.number("alignmentBasic") / 100.0f, 0.0f, 1.0f);

    cpu.hSeparationWeightBasic =
        std::clamp(c.number("separationBasic") / 100.0f, 0.0f, 1.0f);

    cpu.hTargetAttractionWeightBasic =
        std::clamp(c.number("targetAttractionBasic") / 100.0f, 0.0f, 1.0f);


    //
    // Speed scaling relative to world size
    //
    float maxSpeedWorld = longestWorldEdge / 10.0f;

    // Basic
    cpu.hCruisingSpeedBasic =
        maxSpeedWorld * std::clamp(c.number("cruisingSpeedBasic") / 100.0f, 0.0f, 1.0f);

    cpu.hMaxSpeedBasic =
        maxSpeedWorld * std::clamp(c.number("maxSpeedBasic") / 100.0f, 0.0f, 1.0f);

    cpu.hMinSpeedBasic =
        maxSpeedWorld * std::clamp(c.number("minSpeedBasic") / 100.0f, 0.0f, 1.0f);

    // Predator
    cpu.hCruisingSpeedPredator =
        maxSpeedWorld * std::clamp(c.number("cruisingSpeedPredator") / 100.0f, 0.0f, 1.0f);

    cpu.hMaxSpeedPredator =
        maxSpeedWorld * std::clamp(c.number("maxSpeedPredator") / 100.0f, 0.0f, 1.0f);

    cpu.hMinSpeedPredator =
        maxSpeedWorld * std::clamp(c.number("minSpeedPredator") / 100.0f, 0.0f, 1.0f);


    //
    // Predator stamina
    //
    cpu.hMaxStaminaPredator        = c.number("predatorMaxStamina");
    cpu.hStaminaRecoveryRatePredator = c.number("predatorStaminaRecoveryRate");
    cpu.hStaminaDrainRatePredator    = c.number("predatorStaminaDrainRate");


    //
    // Dynamics
    //
    cpu.hDrag  = std::clamp(c.number("drag")  / 100.0f, 0.0f, 1.0f);
    cpu.hNoise = std::clamp(c.number("noise") / 100.0f, 0.0f, 1.0f);

    cpu.hNumStepsToStopDueToMaxDrag = 100.0f;


    //
    // Forces
    //
    cpu.hMaxForce =
        maxSpeedWorld * std::clamp(c.number("maxForce") / 100.0f, 0.0f, 1.0f);


    //
    // Neighbor selection
    //
    cpu.hMaxNeighborsBasic =
        static_cast<float>(boids.basicBoidCount) *
        (c.number("neighbourAccuracy") / 100.0f);


    //
    // Interaction
    //
    cpu.hInteraction.type =
        s.leftMouseEffect.string() == "attract" ? InteractionType::Attract :
        s.leftMouseEffect.string() == "repel"   ? InteractionType::Repel :
                                                  InteractionType::Empty;

    cpu.hInteraction.point = s.interaction.point;
}
