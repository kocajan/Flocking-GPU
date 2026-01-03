#include <cmath>
#include <iostream>
#include <algorithm>
#include <cuda_runtime.h>

#include "versions/parallel/ParallelParameters.hpp"
#include "core/SimState.hpp"
#include "config/Config.hpp"


namespace {
    #define CHECK_ERROR( error ) ( HandleError( error, __FILE__, __LINE__ ) )

    static void HandleError(cudaError_t error, const char* file, int line) { 
        if (error != cudaSuccess) { 
            std::cout << cudaGetErrorString(error) << " in " << file << " at line " << line << std::endl; 
            int w = scanf(" "); 
            exit(EXIT_FAILURE); 
        } 
    }
} // anonymous namespace


ParallelParameters::ParallelParameters(SimState& s, const Config& c)
    : cpu{.hBoids = s.boids} {

    // Define helper function for converting percentage weights
    auto percentToWeight = [](float percent) {
        return std::clamp(percent / 100.0f, 0.0f, 1.0f);
    };

    // ################################################################
    // CPU parameters
    // ################################################################
    cpu.blockSize = 256;

    // ################################################################
    // GPU parameters - basic
    // ################################################################
    
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
    gpu.maxForce = maxSpeedWorld * percentToWeight(c.number("maxForce"));

    // Neighbor selection
    gpu.maxNeighborsBasic = static_cast<int>(static_cast<float>(cpu.hBoids.basicBoidCount) * percentToWeight(c.number("neighbourAccuracy")));

    // World interaction
    gpu.dInteraction.type = static_cast<uint8_t>(s.interaction.type);
    gpu.dInteraction.point = s.interaction.point;

    // ################################################################
    // GPU parameters - Boids
    // ################################################################
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

    // ################################################################
    // GPU parameters - Grid
    // ################################################################
    // Define spatial partitioning parameters
    gpu.dGrid.cellSize = std::max(gpu.visionRangeBasic, gpu.visionRangePredator);
    if (gpu.dGrid.cellSize > gpu.eps) {
        gpu.dGrid.numCellsX = static_cast<int>(std::ceil(gpu.worldX / gpu.dGrid.cellSize));
        gpu.dGrid.numCellsY = static_cast<int>(std::ceil(gpu.worldY / gpu.dGrid.cellSize));
        gpu.dGrid.numCellsZ = gpu.is2D ? 1 : static_cast<int>(std::ceil(gpu.worldZ / gpu.dGrid.cellSize));
        gpu.dGrid.totalCells = gpu.dGrid.numCellsX * gpu.dGrid.numCellsY * gpu.dGrid.numCellsZ;
    } else {
        gpu.dGrid.totalCells = 0;
        gpu.dGrid.cellSize = 0.0f;
        gpu.dGrid.numCellsX = 0;
        gpu.dGrid.numCellsY = 0;
        gpu.dGrid.numCellsZ = 0;
    }
    
    // Allocate device memory for grid data
    // - BASIC BOIDS
    size_t capacityBasic = 0;
    if (numBasic > 0) {
        capacityBasic = 1;
        while (capacityBasic < numBasic) capacityBasic <<= 1;
    }

    CHECK_ERROR(cudaMalloc(&gpu.dGrid.hashBasic, sizeof(int) * capacityBasic));
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.indexBasic, sizeof(int) * capacityBasic));

    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellStartBasic, sizeof(int) * gpu.dGrid.totalCells));
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellEndBasic, sizeof(int) * gpu.dGrid.totalCells));

    // - PREDATOR BOIDS
    size_t capacityPredator = 0;
    if (numPredator > 0) {
        capacityPredator = 1;
        while (capacityPredator < numPredator) capacityPredator <<= 1;
    }

    CHECK_ERROR(cudaMalloc(&gpu.dGrid.hashPredator, sizeof(int) * capacityPredator));
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.indexPredator, sizeof(int) * capacityPredator));

    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellStartPredator, sizeof(int) * gpu.dGrid.totalCells));
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellEndPredator, sizeof(int) * gpu.dGrid.totalCells));

    // - OBSTACLE BOIDS
    size_t capacityObstacle = 0;
    if (numObstacle > 0) {
        capacityObstacle = 1;
        while (capacityObstacle < numObstacle) capacityObstacle <<= 1;
    }

    CHECK_ERROR(cudaMalloc(&gpu.dGrid.hashObstacle, sizeof(int) * capacityObstacle));
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.indexObstacle, sizeof(int) * capacityObstacle));

    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellStartObstacle, sizeof(int) * gpu.dGrid.totalCells));
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellEndObstacle, sizeof(int) * gpu.dGrid.totalCells));

    // - SPATIAL PARTITIONING
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellX, sizeof(int) * gpu.dGrid.totalCells));
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellY, sizeof(int) * gpu.dGrid.totalCells));
    CHECK_ERROR(cudaMalloc(&gpu.dGrid.cellZ, sizeof(int) * gpu.dGrid.totalCells));
}

ParallelParameters::~ParallelParameters() {
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

    // - GRID - BASIC BOIDS
    CHECK_ERROR(cudaFree(gpu.dGrid.hashBasic));
    CHECK_ERROR(cudaFree(gpu.dGrid.indexBasic));
    CHECK_ERROR(cudaFree(gpu.dGrid.cellStartBasic));
    CHECK_ERROR(cudaFree(gpu.dGrid.cellEndBasic));

    // - GRID - PREDATOR BOIDS
    CHECK_ERROR(cudaFree(gpu.dGrid.hashPredator));
    CHECK_ERROR(cudaFree(gpu.dGrid.indexPredator));
    CHECK_ERROR(cudaFree(gpu.dGrid.cellStartPredator));
    CHECK_ERROR(cudaFree(gpu.dGrid.cellEndPredator));

    // - GRID - OBSTACLE BOIDS
    CHECK_ERROR(cudaFree(gpu.dGrid.hashObstacle));
    CHECK_ERROR(cudaFree(gpu.dGrid.indexObstacle));
    CHECK_ERROR(cudaFree(gpu.dGrid.cellStartObstacle));
    CHECK_ERROR(cudaFree(gpu.dGrid.cellEndObstacle));

    // - GRID - SPATIAL PARTITIONING
    CHECK_ERROR(cudaFree(gpu.dGrid.cellX));
    CHECK_ERROR(cudaFree(gpu.dGrid.cellY));
    CHECK_ERROR(cudaFree(gpu.dGrid.cellZ));
}
