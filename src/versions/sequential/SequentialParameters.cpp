#include <algorithm>

#include "versions/sequential/SequentialParameters.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


SequentialParameters::SequentialParameters(SimState& s,const Config& c) 
    : boids(s.boids), 
      obstacleBoidIndices(s.obstacleBoidIndices), 
      basicBoidIndices(s.basicBoidIndices), 
      predatorBoidIndices(s.predatorBoidIndices) {
    // Get boid count from length of boid array
    boidCount = static_cast<int>(s.boids.size());

    // Interaction
    interaction = s.interaction;

    // Dimensionality
    is2D = (s.dimensions.string() == "2D");

    // Radii
    basicBoidRadius = s.basicBoidRadius.number();
    predatorRadius = s.predatorRadius.number();
    obstacleRadius = s.obstacleRadius.number();

    // Boundary mode
    bounce = c.binary("bounce");
    bounceFactor = c.number("bounceFactor") / 100.0f;

    // World bounds
    worldX = s.worldX.number();
    worldY = s.worldY.number();
    worldZ = s.worldZ.number();
    float longestWorldEdge = std::max({worldX, worldY, is2D ? 0.0f : worldZ});

    // World diagonal
    float worldDiag;
    if (is2D)
        worldDiag = std::sqrt(worldX * worldX + worldY * worldY);
    else
        worldDiag = std::sqrt(worldX * worldX + worldY * worldY + worldZ * worldZ);

    // Precompute max distance between points in the world
    if (bounce) {
        maxDistanceBetweenPoints = worldDiag * 0.5f;
    } else {
        maxDistanceBetweenPoints = worldDiag;
    }
    maxDistanceBetweenPoints2 = maxDistanceBetweenPoints * maxDistanceBetweenPoints;

    // Core numeric constants
    eps = s.eps.number();
    dt = s.dt.number();

    // Vision ranges
    visionRangeBasic = worldDiag * (c.number("visionBasic") / 100.0f);
    visionRangePredator = worldDiag * (c.number("visionPredator") / 100.0f);
    visionRangeBasic2 = visionRangeBasic * visionRangeBasic;

    // Obstacle + mouse
    obstacleAvoidanceMultiplier = c.number("obstacleAvoidanceMultiplier");
    mouseInteractionMultiplier  = c.number("mouseInteractionMultiplier");

    // Flocking weights (normalized 0–1 later in behavior code)
    cohesionWeightBasic = std::clamp(c.number("cohesionBasic") / 100.0f, 0.0f, 1.0f);
    alignmentWeightBasic = std::clamp(c.number("alignmentBasic") / 100.0f, 0.0f, 1.0f);
    separationWeightBasic = std::clamp(c.number("separationBasic") / 100.0f, 0.0f, 1.0f);

    targetAttractionWeightBasic = std::clamp(c.number("targetAttractionBasic") / 100.0f, 0.0f, 1.0f);

    // Calculate max speed based on the longest world edge
    float maxSpeed = longestWorldEdge / 10.0f;

    // Basic boid speeds
    float cruisingSpeedBasicPercentage = std::clamp(c.number("cruisingSpeedBasic") / 100.0f, 0.0f, 1.0f);
    float maxSpeedBasicPercentage = std::clamp(c.number("maxSpeedBasic") / 100.0f, 0.0f, 1.0f);
    float minSpeedBasicPercentage = std::clamp(c.number("minSpeedBasic") / 100.0f, 0.0f, 1.0f);
    maxSpeedBasic = maxSpeed * maxSpeedBasicPercentage;
    minSpeedBasic = maxSpeed * minSpeedBasicPercentage;
    cruisingSpeedBasic = maxSpeedBasic * cruisingSpeedBasicPercentage;

    // Predator speeds
    float cruisingSpeedPredatorPercentage = std::clamp(c.number("cruisingSpeedPredator") / 100.0f, 0.0f, 1.0f);
    float maxSpeedPredatorPercentage = std::clamp(c.number("maxSpeedPredator") / 100.0f, 0.0f, 1.0f);
    float minSpeedPredatorPercentage = std::clamp(c.number("minSpeedPredator") / 100.0f, 0.0f, 1.0f);
    maxSpeedPredator = maxSpeed * maxSpeedPredatorPercentage;
    minSpeedPredator = maxSpeed * minSpeedPredatorPercentage;
    cruisingSpeedPredator = maxSpeedPredator * cruisingSpeedPredatorPercentage;
    
    // Predator stamina
    maxStaminaPredator = c.number("predatorMaxStamina");
    staminaRecoveryRatePredator = c.number("predatorStaminaRecoveryRate");
    staminaDrainRatePredator = c.number("predatorStaminaDrainRate");

    // Calculate max force based on max speed
    float maxForcePercentage = std::clamp(c.number("maxForce") / 100.0f, 0.0f, 1.0f);
    maxForce = maxSpeed * maxForcePercentage;

    // Dynamics
    drag  = std::clamp(c.number("drag") / 100.0f, 0.0f, 1.0f);
    noise = std::clamp(c.number("noise") / 100.0f, 0.0f, 1.0f);
    numStepsToStopDueToMaxDrag = 100.0f;

    // Neighbor selection
    maxNeighborsBasic = static_cast<float>(s.basicBoidCount) * (c.number("neighbourAccuracy") / 100.0f);

    // Cell parameters
    cellSize = std::max(visionRangeBasic, visionRangePredator); 
    if (cellSize > eps) {
        cellsX = static_cast<int>(std::ceil(worldX / cellSize));
        cellsY = static_cast<int>(std::ceil(worldY / cellSize));
        cellsZ = is2D ? 1 : static_cast<int>(std::ceil(worldZ / cellSize));
    } else {
        cellsX = 0;
        cellsY = 0;
        cellsZ = 0;
    }
}
