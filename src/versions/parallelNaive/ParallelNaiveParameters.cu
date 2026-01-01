#include <cmath>
#include <iostream>
#include <algorithm>
#include <cuda_runtime.h>

#include "versions/parallelNaive/ParallelNaiveParameters.hpp"
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


ParallelNaiveParameters::ParallelNaiveParameters(SimState& s, const Config& c)
    : cpu{.hBoids = s.boids} {

    // Define helper function for converting percentage weights
    auto percentToWeight = [](float percent) {
        return std::clamp(percent / 100.0f, 0.0f, 1.0f);
    };

    // ################################################################
    // CPU parameters
    // ################################################################
    cpu.blockSize = 256;
    cpu.gridSize = (static_cast<int>(cpu.hBoids.count) + cpu.blockSize - 1) / cpu.blockSize;

    // ################################################################
    // GPU parameters - basic
    // ################################################################
    // Boid count
    gpu.boidCount = static_cast<int>(cpu.hBoids.count);
    
    // Flags
    gpu.is2D = (s.dimensions.string() == "2D");
    gpu.bounce = c.binary("bounce");
    gpu.bounceFactor = percentToWeight(c.number("bounceFactor"));

    // Radii
    gpu.basicBoidRadius = s.basicBoidRadius.number();
    gpu.predatorRadius  = s.predatorRadius.number();
    gpu.obstacleRadius  = s.obstacleRadius.number();

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
    gpu.dBoids.count = gpu.boidCount;
    gpu.dBoids.basicBoidCount = static_cast<int>(cpu.hBoids.basicBoidCount);
    gpu.dBoids.predatorBoidCount = static_cast<int>(cpu.hBoids.predatorBoidCount);
    gpu.dBoids.obstacleBoidCount = static_cast<int>(cpu.hBoids.obstacleBoidCount);
    
    // Allocate device memory for boid data
    const size_t N = gpu.boidCount;
    const size_t vec3ArraySize = sizeof(Vec3) * N;
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.pos, vec3ArraySize));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.vel, vec3ArraySize));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.acc, vec3ArraySize));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetPoint, vec3ArraySize));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.type, sizeof(uint8_t) * N));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.stamina, sizeof(float) * N));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.resting, sizeof(uint8_t) * N));

    CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetBoidIdx, sizeof(int) * N));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetBoidDistance, sizeof(float) * N));

    CHECK_ERROR(cudaMalloc(&gpu.dBoids.basicBoidIndices, sizeof(size_t) * cpu.hBoids.basicBoidCount));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.predatorBoidIndices, sizeof(size_t) * cpu.hBoids.predatorBoidCount));
    CHECK_ERROR(cudaMalloc(&gpu.dBoids.obstacleBoidIndices, sizeof(size_t) * cpu.hBoids.obstacleBoidCount));

    // Copy boid data to device
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.pos, cpu.hBoids.pos.data(), vec3ArraySize, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.vel, cpu.hBoids.vel.data(), vec3ArraySize, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.acc, cpu.hBoids.acc.data(), vec3ArraySize, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetPoint, cpu.hBoids.targetPoint.data(), vec3ArraySize, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(gpu.dBoids.type, cpu.hBoids.type.data(), sizeof(uint8_t) * N, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.stamina, cpu.hBoids.stamina.data(), sizeof(float) * N, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.resting, cpu.hBoids.resting.data(), sizeof(uint8_t) * N, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetBoidIdx, cpu.hBoids.targetBoidIdx.data(), sizeof(int) * N, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetBoidDistance, cpu.hBoids.targetBoidDistance.data(), sizeof(float) * N, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(gpu.dBoids.basicBoidIndices, cpu.hBoids.basicBoidIndices.data(), sizeof(size_t) * cpu.hBoids.basicBoidCount, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.predatorBoidIndices, cpu.hBoids.predatorBoidIndices.data(), sizeof(size_t) * cpu.hBoids.predatorBoidCount, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(gpu.dBoids.obstacleBoidIndices, cpu.hBoids.obstacleBoidIndices.data(), sizeof(size_t) * cpu.hBoids.obstacleBoidCount, cudaMemcpyHostToDevice));
}

ParallelNaiveParameters::~ParallelNaiveParameters() {
    // Check for any CUDA errors
    CHECK_ERROR(cudaPeekAtLastError());
    CHECK_ERROR(cudaDeviceSynchronize());

    // Port from GPU to CPU
    const size_t N = cpu.hBoids.count;
    const size_t vec3ArraySize = sizeof(Vec3) * N;
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.pos.data(), gpu.dBoids.pos, vec3ArraySize, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.vel.data(), gpu.dBoids.vel, vec3ArraySize, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.acc.data(), gpu.dBoids.acc, vec3ArraySize, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetPoint.data(), gpu.dBoids.targetPoint, vec3ArraySize, cudaMemcpyDeviceToHost));

    CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetBoidIdx.data(), gpu.dBoids.targetBoidIdx, sizeof(int) * N, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetBoidDistance.data(), gpu.dBoids.targetBoidDistance, sizeof(float) * N, cudaMemcpyDeviceToHost));

    CHECK_ERROR(cudaMemcpy(cpu.hBoids.stamina.data(), gpu.dBoids.stamina, sizeof(float) * N, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(cpu.hBoids.resting.data(), gpu.dBoids.resting, sizeof(uint8_t) * N, cudaMemcpyDeviceToHost));

    // Free device memory
    CHECK_ERROR(cudaFree(gpu.dBoids.pos));
    CHECK_ERROR(cudaFree(gpu.dBoids.vel));
    CHECK_ERROR(cudaFree(gpu.dBoids.acc));
    CHECK_ERROR(cudaFree(gpu.dBoids.targetPoint));
    CHECK_ERROR(cudaFree(gpu.dBoids.type));
    CHECK_ERROR(cudaFree(gpu.dBoids.stamina));
    CHECK_ERROR(cudaFree(gpu.dBoids.resting));

    CHECK_ERROR(cudaFree(gpu.dBoids.targetBoidIdx));
    CHECK_ERROR(cudaFree(gpu.dBoids.targetBoidDistance));

    CHECK_ERROR(cudaFree(gpu.dBoids.basicBoidIndices));
    CHECK_ERROR(cudaFree(gpu.dBoids.predatorBoidIndices));
    CHECK_ERROR(cudaFree(gpu.dBoids.obstacleBoidIndices));
}