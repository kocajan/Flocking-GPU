/**
 * \file ParallelNaiveParameters.cu
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Implementation of CPU–GPU parameter initialization for the parallel-naive backend.
 *
 * Responsibilities:
 *  - derives simulation constants
 *  - allocates GPU memory
 *  - transfers boid buffers to device
 *  - synchronizes results back to CPU on destruction
 */

#include <cmath>
#include <iostream>
#include <algorithm>
#include <cuda_runtime.h>

#include "versions/parallelNaive/ParallelNaiveParameters.cuh"

#include "core/SimState.hpp"
#include "config/Config.hpp"
#include "utils/SimStepParallelUtils.cuh"


ParallelNaiveParameters::ParallelNaiveParameters(SimState& s, const Config& c)
    : cpu{.hBoids = s.boids} {

    // Define helper function for converting percentage weights
    auto percentToWeight = [](float percent) {
        return std::clamp(percent / 100.0f, 0.0f, 1.0f);
    };

    // ---------------------------------------------------------------------------
    // CPU parameters
    // ---------------------------------------------------------------------------
    cpu.blockSize = static_cast<int>(s.cudaBlockSize.number());

    // ---------------------------------------------------------------------------
    // GPU parameters - basic
    // ---------------------------------------------------------------------------
    // Flags
    gpu.is2D = (s.dimensions.string() == "2D");
    gpu.bounce = c.binary("bounce");
    gpu.bounceFactor = percentToWeight(c.number("bounceFactor"));

    // Radii
    gpu.basicBoidRadius = s.basicBoidRadius.number();
    gpu.predatorBoidRadius = s.predatorBoidRadius.number();
    gpu.obstacleBoidRadius = s.obstacleBoidRadius.number();

    // World bounds
    gpu.worldX = s.worldX.number();
    gpu.worldY = s.worldY.number();
    gpu.worldZ = s.worldZ.number();

    float longestWorldEdge = std::max({ gpu.worldX, gpu.worldY, gpu.is2D ? 0.0f : gpu.worldZ });

    float worldDiag = gpu.is2D
        ? std::sqrt(gpu.worldX*gpu.worldX + gpu.worldY*gpu.worldY)
        : std::sqrt(gpu.worldX*gpu.worldX +
                    gpu.worldY*gpu.worldY +
                    gpu.worldZ*gpu.worldZ);

    gpu.maxDistanceBetweenPoints = gpu.bounce ? worldDiag * 0.5f : worldDiag;
    gpu.maxDistanceBetweenPoints2 = gpu.maxDistanceBetweenPoints * gpu.maxDistanceBetweenPoints;

    // Core constants
    gpu.eps = s.eps.number();
    gpu.dt = s.dt.number();

    // Vision
    gpu.visionRangeBasic = worldDiag * percentToWeight(c.number("visionBasic"));
    gpu.visionRangePredator = worldDiag * percentToWeight(c.number("visionPredator"));
    gpu.visionRangeBasic2 = gpu.visionRangeBasic * gpu.visionRangeBasic;

    // Multipliers
    gpu.obstacleAvoidanceMultiplier = c.number("obstacleAvoidanceMultiplier");
    gpu.mouseInteractionMultiplier  = c.number("mouseInteractionMultiplier");

    // Flocking weights
    gpu.cohesionWeightBasic = percentToWeight(c.number("cohesionBasic"));
    gpu.alignmentWeightBasic = percentToWeight(c.number("alignmentBasic"));
    gpu.separationWeightBasic = percentToWeight(c.number("separationBasic"));
    gpu.targetAttractionWeightBasic = percentToWeight(c.number("targetAttractionBasic"));

    // Speed scaling relative to world size
    float maxSpeedWorld = longestWorldEdge / 10.0f;

    // Basic
    gpu.cruisingSpeedBasic = maxSpeedWorld * percentToWeight(c.number("cruisingSpeedBasic"));
    gpu.maxSpeedBasic = maxSpeedWorld * percentToWeight(c.number("maxSpeedBasic"));
    gpu.minSpeedBasic = maxSpeedWorld * percentToWeight(c.number("minSpeedBasic"));

    // Predator
    gpu.cruisingSpeedPredator = maxSpeedWorld * percentToWeight(c.number("cruisingSpeedPredator"));
    gpu.maxSpeedPredator = maxSpeedWorld * percentToWeight(c.number("maxSpeedPredator"));
    gpu.minSpeedPredator = maxSpeedWorld * percentToWeight(c.number("minSpeedPredator"));

    // Predator stamina
    gpu.maxStaminaPredator = c.number("predatorMaxStamina");
    gpu.staminaRecoveryRatePredator = c.number("predatorStaminaRecoveryRate");
    gpu.staminaDrainRatePredator = c.number("predatorStaminaDrainRate");

    // Dynamics
    gpu.drag = percentToWeight(c.number("drag"));
    gpu.noise = percentToWeight(c.number("noise"));
    gpu.numStepsToStopDueToMaxDrag = 100.0f;

    // Forces
    gpu.baseForce = maxSpeedWorld * percentToWeight(c.number("baseForce"));

    // Neighbor selection
    gpu.maxNeighborsBasic = static_cast<int>(static_cast<float>(cpu.hBoids.basicBoidCount) * percentToWeight(c.number("neighbourAccuracy")));

    // World interaction
    gpu.interaction.type = s.interaction.type;
    gpu.interaction.point = s.interaction.point;

    // ---------------------------------------------------------------------------
    // GPU parameters - Boids
    // ---------------------------------------------------------------------------
    // Port boid counts
    gpu.dBoids.basicBoidCount = cpu.hBoids.basicBoidCount;
    gpu.dBoids.predatorBoidCount = cpu.hBoids.predatorBoidCount;
    gpu.dBoids.obstacleBoidCount = cpu.hBoids.obstacleBoidCount;
    
    // Allocate device memory for boid data
    // - BASIC BOIDS
    const int numBasic = gpu.dBoids.basicBoidCount;
    const size_t basicArraysSize = static_cast<size_t>(sizeof(Vec3) * numBasic);
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.posBasic, basicArraysSize));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.velBasic, basicArraysSize));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.accBasic, basicArraysSize));

    CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetPointBasic, basicArraysSize));

    // - PREDATOR BOIDS
    const int numPredator = gpu.dBoids.predatorBoidCount;
    const size_t predatorArraysSize = static_cast<size_t>(sizeof(Vec3) * numPredator);
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.posPredator, predatorArraysSize));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.velPredator, predatorArraysSize));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.accPredator, predatorArraysSize));

    CHECK_ERROR(cudaMalloc(&gpu.dBoids.staminaPredator, sizeof(float) * numPredator));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.restingPredator, sizeof(uint8_t) * numPredator));

    CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetBoidIdxPredator, sizeof(int) * numPredator));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetBoidDistancePredator, sizeof(float) * numPredator));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetBoidTypePredator, sizeof(uint8_t) * numPredator));

    // - OBSTACLE BOIDS
    const int numObstacle = gpu.dBoids.obstacleBoidCount;
    const size_t obstacleArraysSize = static_cast<size_t>(sizeof(Vec3) * numObstacle);
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.posObstacle, obstacleArraysSize));

    // Copy boid data to device
    // - BASIC BOIDS
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.posBasic, cpu.hBoids.posBasic.data(), basicArraysSize, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.velBasic, cpu.hBoids.velBasic.data(), basicArraysSize, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.accBasic, cpu.hBoids.accBasic.data(), basicArraysSize, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetPointBasic, cpu.hBoids.targetPointBasic.data(), basicArraysSize, cudaMemcpyHostToDevice));

    // - PREDATOR BOIDS
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.posPredator, cpu.hBoids.posPredator.data(), predatorArraysSize, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.velPredator, cpu.hBoids.velPredator.data(), predatorArraysSize, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.accPredator, cpu.hBoids.accPredator.data(), predatorArraysSize, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(gpu.dBoids.staminaPredator, cpu.hBoids.staminaPredator.data(), sizeof(float) * numPredator, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.restingPredator, cpu.hBoids.restingPredator.data(), sizeof(uint8_t) * numPredator, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetBoidIdxPredator, cpu.hBoids.targetBoidIdxPredator.data(), sizeof(int) * numPredator, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetBoidDistancePredator, cpu.hBoids.targetBoidDistancePredator.data(), sizeof(float) * numPredator, cudaMemcpyHostToDevice));
    
    // - OBSTACLE BOIDS
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.posObstacle, cpu.hBoids.posObstacle.data(), obstacleArraysSize, cudaMemcpyHostToDevice));
}

ParallelNaiveParameters::~ParallelNaiveParameters() {
    // Port from GPU to CPU
    // - BASIC BOIDS
    const size_t basicArraysSize = static_cast<size_t>(sizeof(Vec3) * gpu.dBoids.basicBoidCount);

    CHECK_ERROR(cudaMemcpy(cpu.hBoids.posBasic.data(), gpu.dBoids.posBasic, basicArraysSize, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.velBasic.data(), gpu.dBoids.velBasic, basicArraysSize, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.accBasic.data(), gpu.dBoids.accBasic, basicArraysSize, cudaMemcpyDeviceToHost));

    // - PREDATOR BOIDS
    const size_t predatorArraysSize = static_cast<size_t>(sizeof(Vec3) * gpu.dBoids.predatorBoidCount);
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.posPredator.data(), gpu.dBoids.posPredator, predatorArraysSize, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.velPredator.data(), gpu.dBoids.velPredator, predatorArraysSize, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.accPredator.data(), gpu.dBoids.accPredator, predatorArraysSize, cudaMemcpyDeviceToHost));

    CHECK_ERROR(cudaMemcpy(cpu.hBoids.staminaPredator.data(), gpu.dBoids.staminaPredator, sizeof(float) * gpu.dBoids.predatorBoidCount, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.restingPredator.data(), gpu.dBoids.restingPredator, sizeof(uint8_t) * gpu.dBoids.predatorBoidCount, cudaMemcpyDeviceToHost));

    CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetBoidIdxPredator.data(), gpu.dBoids.targetBoidIdxPredator, sizeof(int) * gpu.dBoids.predatorBoidCount, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetBoidDistancePredator.data(), gpu.dBoids.targetBoidDistancePredator, sizeof(float) * gpu.dBoids.predatorBoidCount, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetBoidTypePredator.data(), gpu.dBoids.targetBoidTypePredator, sizeof(uint8_t) * gpu.dBoids.predatorBoidCount, cudaMemcpyDeviceToHost));

    // Free device memory
    // - BASIC BOIDS
    CHECK_ERROR(cudaFree(gpu.dBoids.posBasic));
    CHECK_ERROR(cudaFree(gpu.dBoids.velBasic));
    CHECK_ERROR(cudaFree(gpu.dBoids.accBasic));
    CHECK_ERROR(cudaFree(gpu.dBoids.targetPointBasic));
    
    // - PREDATOR BOIDS
    CHECK_ERROR(cudaFree(gpu.dBoids.posPredator));
    CHECK_ERROR(cudaFree(gpu.dBoids.velPredator));
    CHECK_ERROR(cudaFree(gpu.dBoids.accPredator));
    CHECK_ERROR(cudaFree(gpu.dBoids.staminaPredator));
    CHECK_ERROR(cudaFree(gpu.dBoids.restingPredator));
    CHECK_ERROR(cudaFree(gpu.dBoids.targetBoidIdxPredator));
    CHECK_ERROR(cudaFree(gpu.dBoids.targetBoidDistancePredator));
    CHECK_ERROR(cudaFree(gpu.dBoids.targetBoidTypePredator));

    // - OBSTACLE BOIDS
    CHECK_ERROR(cudaFree(gpu.dBoids.posObstacle));

    // Final synchronization
    CHECK_ERROR(cudaDeviceSynchronize());
}