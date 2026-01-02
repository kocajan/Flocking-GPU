#include <algorithm>

#include "versions/sequential/SequentialParameters.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


SequentialParameters::SequentialParameters(SimState& s,const Config& c) 
    : boids(s.boids), 
      obstacleBoidIndices(s.boids.obstacleBoidIndices), 
      basicBoidIndices(s.boids.basicBoidIndices), 
      predatorBoidIndices(s.boids.predatorBoidIndices) {
    // Define helper function for converting percentage weights
    auto percentToWeight = [](float percent) {
        return std::clamp(percent / 100.0f, 0.0f, 1.0f);
    };

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
    bounceFactor = percentToWeight(c.number("bounceFactor"));

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
    visionRangeBasic = worldDiag * percentToWeight(c.number("visionBasic"));
    visionRangePredator = worldDiag * percentToWeight(c.number("visionPredator"));
    visionRangeBasic2 = visionRangeBasic * visionRangeBasic;

    // Obstacle + mouse
    obstacleAvoidanceMultiplier = c.number("obstacleAvoidanceMultiplier");
    mouseInteractionMultiplier  = c.number("mouseInteractionMultiplier");

    // Flocking weights (normalized 0–1 later in behavior code)
    cohesionWeightBasic = percentToWeight(c.number("cohesionBasic"));
    alignmentWeightBasic = percentToWeight(c.number("alignmentBasic"));
    separationWeightBasic = percentToWeight(c.number("separationBasic"));

    targetAttractionWeightBasic = percentToWeight(c.number("targetAttractionBasic"));

    // Calculate max speed based on the longest world edge
    float maxSpeed = longestWorldEdge / 10.0f;

    // Basic boid speeds
    float cruisingSpeedBasicPercentage = percentToWeight(c.number("cruisingSpeedBasic"));
    float maxSpeedBasicPercentage = percentToWeight(c.number("maxSpeedBasic"));
    float minSpeedBasicPercentage = percentToWeight(c.number("minSpeedBasic"));
    maxSpeedBasic = maxSpeed * maxSpeedBasicPercentage;
    minSpeedBasic = maxSpeed * minSpeedBasicPercentage;
    cruisingSpeedBasic = maxSpeedBasic * cruisingSpeedBasicPercentage;

    // Predator speeds
    float cruisingSpeedPredatorPercentage = percentToWeight(c.number("cruisingSpeedPredator"));
    float maxSpeedPredatorPercentage = percentToWeight(c.number("maxSpeedPredator"));
    float minSpeedPredatorPercentage = percentToWeight(c.number("minSpeedPredator"));
    maxSpeedPredator = maxSpeed * maxSpeedPredatorPercentage;
    minSpeedPredator = maxSpeed * minSpeedPredatorPercentage;
    cruisingSpeedPredator = maxSpeedPredator * cruisingSpeedPredatorPercentage;
    
    // Predator stamina
    maxStaminaPredator = c.number("predatorMaxStamina");
    staminaRecoveryRatePredator = c.number("predatorStaminaRecoveryRate");
    staminaDrainRatePredator = c.number("predatorStaminaDrainRate");

    // Calculate max force based on max speed
    float maxForcePercentage = percentToWeight(c.number("maxForce"));
    maxForce = maxSpeed * maxForcePercentage;

    // Dynamics
    drag = percentToWeight(c.number("drag"));
    noise = percentToWeight(c.number("noise"));
    numStepsToStopDueToMaxDrag = 100.0f;

    // Neighbor selection
    maxNeighborsBasic = static_cast<float>(s.boids.basicBoidCount) * percentToWeight(c.number("neighbourAccuracy"));
    
    // Cell parameters
    cellSize = std::max(visionRangeBasic, visionRangePredator); 
    if (cellSize > eps) {
        numCellsX = static_cast<int>(std::ceil(worldX / cellSize));
        numCellsY = static_cast<int>(std::ceil(worldY / cellSize));
        numCellsZ = is2D ? 1 : static_cast<int>(std::ceil(worldZ / cellSize));
    } else {
        cellSize = 0.0f;
        numCellsX = 0;
        numCellsY = 0;
        numCellsZ = 0;
    }
}

SequentialParameters::~SequentialParameters() {
    // Nothing to do here
}