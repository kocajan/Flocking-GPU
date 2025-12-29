#include <algorithm>

#include "versions/sequentialNaive/SequentialNaiveParameters.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


SequentialNaiveParameters::SequentialNaiveParameters(SimState& s,const Config& c) 
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

    // World bounds
    worldX = s.worldX.number();
    worldY = s.worldY.number();
    worldZ = s.worldZ.number();

    // Core numeric constants
    eps = s.eps.number();
    dt = s.dt.number();

    // Vision ranges
    float maxVisualRange = (is2D ? s.maxVisionRange2D : s.maxVisionRange3D);
    visualRangeBasic = maxVisualRange * (c.number("visionBasic") / 100.0f);
    visualRangePredator = maxVisualRange * (c.number("visionPredator") / 100.0f);
    visualRangeBasic2 = visualRangeBasic * visualRangeBasic;

    // Obstacle + mouse
    obstacleAvoidanceMultiplier = c.number("obstacleAvoidanceMultiplier");
    mouseInteractionMultiplier  = c.number("mouseInteractionMultiplier");

    // Flocking weights (normalized 0–1 later in behavior code)
    cohesionWeightBasic = std::clamp(c.number("cohesionBasic") / 100.0f, 0.0f, 1.0f);
    alignmentWeightBasic = std::clamp(c.number("alignmentBasic") / 100.0f, 0.0f, 1.0f);
    separationWeightBasic = std::clamp(c.number("separationBasic") / 100.0f, 0.0f, 1.0f);

    targetAttractionWeightBasic = std::clamp(c.number("targetAttractionBasic") / 100.0f, 0.0f, 1.0f);

    // Basic boid speeds
    cruisingSpeedBasic = c.number("cruisingSpeedBasic");
    maxSpeedBasic = c.number("maxSpeedBasic");
    minSpeedBasic = c.number("minSpeedBasic");

    // Predator speeds
    cruisingSpeedPredator = c.number("cruisingSpeedPredator");
    maxSpeedPredator = c.number("maxSpeedPredator");
    minSpeedPredator = c.number("minSpeedPredator");

    // Predator stamina
    maxStaminaPredator = c.number("predatorMaxStamina");
    staminaRecoveryRatePredator = c.number("predatorStaminaRecoveryRate");
    staminaDrainRatePredator = c.number("predatorStaminaDrainRate");

    // Dynamics
    maxForce = c.number("maxForce");
    drag  = std::clamp(c.number("drag") / 100.0f, 0.0f, 1.0f);
    noise = std::clamp(c.number("noise") / 100.0f, 0.0f, 1.0f);
    numStepsToStopDueToMaxDrag = 100.0f;

    // Boundary mode
    bounce = c.binary("bounce");
    bounceFactor = c.number("bounceFactor") / 100.0f;

    // Neighbor selection
    maxNeighborsBasic = static_cast<float>(s.basicBoidCount) * (c.number("neighbourAccuracy") / 100.0f);

    // Precompute max distance between points in the world
    if (is2D)
        maxDistanceBetweenPoints = std::sqrt(worldX*worldX + worldY*worldY);
    else
        maxDistanceBetweenPoints = std::sqrt(worldX*worldX + worldY*worldY + worldZ*worldZ);

    if (!bounce)
        maxDistanceBetweenPoints *= 0.5f;

    maxDistanceBetweenPoints2 = maxDistanceBetweenPoints * maxDistanceBetweenPoints;
}
