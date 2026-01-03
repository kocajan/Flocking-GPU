// #include <cmath>
// #include <iostream>
// #include <algorithm>
// #include <cuda_runtime.h>

// #include "versions/parallel/ParallelParameters.hpp"
// #include "core/SimState.hpp"
// #include "config/Config.hpp"


// namespace {
//     #define CHECK_ERROR( error ) ( HandleError( error, __FILE__, __LINE__ ) )

//     static void HandleError(cudaError_t error, const char* file, int line) { 
//         if (error != cudaSuccess) { 
//             std::cout << cudaGetErrorString(error) << " in " << file << " at line " << line << std::endl; 
//             int w = scanf(" "); 
//             exit(EXIT_FAILURE); 
//         } 
//     }
// } // anonymous namespace


// ParallelParameters::ParallelParameters(SimState& s, const Config& c)
//     : cpu{.hBoids = s.boids} {

//     // Define helper function for converting percentage weights
//     auto percentToWeight = [](float percent) {
//         return std::clamp(percent / 100.0f, 0.0f, 1.0f);
//     };

//     // ################################################################
//     // CPU parameters
//     // ################################################################
//     cpu.blockSize = 256;

//     // ################################################################
//     // GPU parameters - basic
//     // ################################################################
    
//     // Flags
//     gpu.is2D = (s.dimensions.string() == "2D");
//     gpu.bounce = c.binary("bounce");
//     gpu.bounceFactor = percentToWeight(c.number("bounceFactor"));

//     // Radii
//     gpu.basicBoidRadius = s.basicBoidRadius.number();
//     gpu.predatorRadius  = s.predatorRadius.number();
//     gpu.obstacleRadius  = s.obstacleRadius.number();

//     // World bounds
//     gpu.worldX = s.worldX.number();
//     gpu.worldY = s.worldY.number();
//     gpu.worldZ = s.worldZ.number();

//     float longestWorldEdge = std::max({ gpu.worldX, gpu.worldY, gpu.is2D ? 0.0f : gpu.worldZ });

//     float worldDiag = gpu.is2D
//         ? std::sqrt(gpu.worldX*gpu.worldX + gpu.worldY*gpu.worldY)
//         : std::sqrt(gpu.worldX*gpu.worldX +
//                     gpu.worldY*gpu.worldY +
//                     gpu.worldZ*gpu.worldZ);

//     gpu.maxDistanceBetweenPoints = gpu.bounce ? worldDiag * 0.5f : worldDiag;
//     gpu.maxDistanceBetweenPoints2 = gpu.maxDistanceBetweenPoints * gpu.maxDistanceBetweenPoints;

//     // Core constants
//     gpu.eps = s.eps.number();
//     gpu.dt = s.dt.number();

//     // Vision
//     gpu.visionRangeBasic = worldDiag * percentToWeight(c.number("visionBasic"));
//     gpu.visionRangePredator = worldDiag * percentToWeight(c.number("visionPredator"));
//     gpu.visionRangeBasic2 = gpu.visionRangeBasic * gpu.visionRangeBasic;

//     // Multipliers
//     gpu.obstacleAvoidanceMultiplier = c.number("obstacleAvoidanceMultiplier");
//     gpu.mouseInteractionMultiplier  = c.number("mouseInteractionMultiplier");

//     // Flocking weights
//     gpu.cohesionWeightBasic = percentToWeight(c.number("cohesionBasic"));
//     gpu.alignmentWeightBasic = percentToWeight(c.number("alignmentBasic"));
//     gpu.separationWeightBasic = percentToWeight(c.number("separationBasic"));
//     gpu.targetAttractionWeightBasic = percentToWeight(c.number("targetAttractionBasic"));

//     // Speed scaling relative to world size
//     float maxSpeedWorld = longestWorldEdge / 10.0f;

//     // Basic
//     gpu.cruisingSpeedBasic = maxSpeedWorld * percentToWeight(c.number("cruisingSpeedBasic"));
//     gpu.maxSpeedBasic = maxSpeedWorld * percentToWeight(c.number("maxSpeedBasic"));
//     gpu.minSpeedBasic = maxSpeedWorld * percentToWeight(c.number("minSpeedBasic"));

//     // Predator
//     gpu.cruisingSpeedPredator = maxSpeedWorld * percentToWeight(c.number("cruisingSpeedPredator"));
//     gpu.maxSpeedPredator = maxSpeedWorld * percentToWeight(c.number("maxSpeedPredator"));
//     gpu.minSpeedPredator = maxSpeedWorld * percentToWeight(c.number("minSpeedPredator"));

//     // Predator stamina
//     gpu.maxStaminaPredator = c.number("predatorMaxStamina");
//     gpu.staminaRecoveryRatePredator = c.number("predatorStaminaRecoveryRate");
//     gpu.staminaDrainRatePredator = c.number("predatorStaminaDrainRate");

//     // Dynamics
//     gpu.drag = percentToWeight(c.number("drag"));
//     gpu.noise = percentToWeight(c.number("noise"));
//     gpu.numStepsToStopDueToMaxDrag = 100.0f;

//     // Forces
//     gpu.maxForce = maxSpeedWorld * percentToWeight(c.number("maxForce"));

//     // Neighbor selection
//     gpu.maxNeighborsBasic = static_cast<int>(static_cast<float>(cpu.hBoids.basicBoidCount) * percentToWeight(c.number("neighbourAccuracy")));

//     // World interaction
//     gpu.dInteraction.type = static_cast<uint8_t>(s.interaction.type);
//     gpu.dInteraction.point = s.interaction.point;

//     // ################################################################
//     // GPU parameters - Boids
//     // ################################################################
//     // Port boid counts
//     gpu.dBoids.allBoidCount = cpu.hBoids.allBoidCount;
//     gpu.dBoids.basicBoidCount = cpu.hBoids.basicBoidCount;
//     gpu.dBoids.predatorBoidCount = cpu.hBoids.predatorBoidCount;
//     gpu.dBoids.obstacleBoidCount = cpu.hBoids.obstacleBoidCount;

//     // Define spatial partitioning parameters
//     gpu.cellSize = std::max(gpu.visionRangeBasic, gpu.visionRangePredator);
//     if (gpu.cellSize > gpu.eps) {
//         gpu.numCellsX = static_cast<int>(std::ceil(gpu.worldX / gpu.cellSize));
//         gpu.numCellsY = static_cast<int>(std::ceil(gpu.worldY / gpu.cellSize));
//         gpu.numCellsZ = gpu.is2D ? 1 : static_cast<int>(std::ceil(gpu.worldZ / gpu.cellSize));
//         gpu.totalCells = gpu.numCellsX * gpu.numCellsY * gpu.numCellsZ;
//     } else {
//         gpu.totalCells = 0;
//         gpu.cellSize = 0.0f;
//         gpu.numCellsX = 0;
//         gpu.numCellsY = 0;
//         gpu.numCellsZ = 0;
//     }
    
//     // Allocate device memory for boid data
//     int B = gpu.dBoids.allBoidCount;
//     size_t capacityBasic = 1;
//     while (capacityBasic < gpu.dBoids.basicBoidCount) capacityBasic <<= 1;
//     size_t capacityPredator = 1;
//     while (capacityPredator < gpu.dBoids.predatorBoidCount) capacityPredator <<= 1;
//     size_t capacityObstacle = 1;
//     while (capacityObstacle < gpu.dBoids.obstacleBoidCount) capacityObstacle <<= 1;

//     const size_t C = gpu.totalCells;
//     const size_t vec3ArraySize = sizeof(Vec3) * B;

//     if (B > 0) {
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.pos, vec3ArraySize));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.vel, vec3ArraySize));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.acc, vec3ArraySize));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetPoint, vec3ArraySize));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.type, sizeof(uint8_t) * B));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.stamina, sizeof(float) * B));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.resting, sizeof(uint8_t) * B));

//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetBoidIdx, sizeof(int) * B));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.targetBoidDistance, sizeof(float) * B));

//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.basicBoidIndices, sizeof(int) * cpu.hBoids.basicBoidCount));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.predatorBoidIndices, sizeof(int) * cpu.hBoids.predatorBoidCount));
//         CHECK_ERROR(cudaMalloc(&gpu.dBoids.obstacleBoidIndices, sizeof(int) * cpu.hBoids.obstacleBoidCount));
//     }

//     if (capacityBasic > 0) {
//         CHECK_ERROR(cudaMalloc(&gpu.dHashBasic, sizeof(int) * capacityBasic));
//         CHECK_ERROR(cudaMalloc(&gpu.dIndexBasic, sizeof(int) * capacityBasic));
//     }

//     if (capacityPredator > 0) {
//         CHECK_ERROR(cudaMalloc(&gpu.dHashPredator, sizeof(int) * capacityPredator));
//         CHECK_ERROR(cudaMalloc(&gpu.dIndexPredator, sizeof(int) * capacityPredator));
//     }

//     if (capacityObstacle > 0) {
//         CHECK_ERROR(cudaMalloc(&gpu.dHashObstacle, sizeof(int) * capacityObstacle));
//         CHECK_ERROR(cudaMalloc(&gpu.dIndexObstacle, sizeof(int) * capacityObstacle));
//     }

//     if (C > 0) {
//         CHECK_ERROR(cudaMalloc(&gpu.dCellStartBasic, sizeof(int) * C));
//         CHECK_ERROR(cudaMalloc(&gpu.dCellEndBasic, sizeof(int) * C));
//         CHECK_ERROR(cudaMalloc(&gpu.dCellStartPredator, sizeof(int) * C));
//         CHECK_ERROR(cudaMalloc(&gpu.dCellEndPredator, sizeof(int) * C));
//         CHECK_ERROR(cudaMalloc(&gpu.dCellStartObstacle, sizeof(int) * C));
//         CHECK_ERROR(cudaMalloc(&gpu.dCellEndObstacle, sizeof(int) * C));

//         CHECK_ERROR(cudaMalloc(&gpu.dCellX, sizeof(int) * C));
//         CHECK_ERROR(cudaMalloc(&gpu.dCellY, sizeof(int) * C));
//         CHECK_ERROR(cudaMalloc(&gpu.dCellZ, sizeof(int) * C));
//     }

//     // Copy boid data to device
//     if (B > 0) {
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.pos, cpu.hBoids.pos.data(), vec3ArraySize, cudaMemcpyHostToDevice));
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.vel, cpu.hBoids.vel.data(), vec3ArraySize, cudaMemcpyHostToDevice));
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.acc, cpu.hBoids.acc.data(), vec3ArraySize, cudaMemcpyHostToDevice));
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetPoint, cpu.hBoids.targetPoint.data(), vec3ArraySize, cudaMemcpyHostToDevice));

//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.type, cpu.hBoids.type.data(), sizeof(uint8_t) * B, cudaMemcpyHostToDevice));
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.stamina, cpu.hBoids.stamina.data(), sizeof(float) * B, cudaMemcpyHostToDevice));
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.resting, cpu.hBoids.resting.data(), sizeof(uint8_t) * B, cudaMemcpyHostToDevice));

//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetBoidIdx, cpu.hBoids.targetBoidIdx.data(), sizeof(int) * B, cudaMemcpyHostToDevice));
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.targetBoidDistance, cpu.hBoids.targetBoidDistance.data(), sizeof(float) * B, cudaMemcpyHostToDevice));

//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.basicBoidIndices, cpu.hBoids.basicBoidIndices.data(), sizeof(int) * cpu.hBoids.basicBoidCount, cudaMemcpyHostToDevice));
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.predatorBoidIndices, cpu.hBoids.predatorBoidIndices.data(), sizeof(int) * cpu.hBoids.predatorBoidCount, cudaMemcpyHostToDevice));
//         CHECK_ERROR(cudaMemcpy(gpu.dBoids.obstacleBoidIndices, cpu.hBoids.obstacleBoidIndices.data(), sizeof(int) * cpu.hBoids.obstacleBoidCount, cudaMemcpyHostToDevice));
//     }
// }

// ParallelParameters::~ParallelParameters() {
//     // Port from GPU to CPU
//     const int B = gpu.dBoids.allBoidCount;
//     const size_t vec3ArraySize = static_cast<size_t>(sizeof(Vec3) * B);

//     if (B > 0) {
//         CHECK_ERROR(cudaMemcpy(cpu.hBoids.pos.data(), gpu.dBoids.pos, vec3ArraySize, cudaMemcpyDeviceToHost));
//         CHECK_ERROR(cudaMemcpy(cpu.hBoids.vel.data(), gpu.dBoids.vel, vec3ArraySize, cudaMemcpyDeviceToHost));
//         CHECK_ERROR(cudaMemcpy(cpu.hBoids.acc.data(), gpu.dBoids.acc, vec3ArraySize, cudaMemcpyDeviceToHost));
//         CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetPoint.data(), gpu.dBoids.targetPoint, vec3ArraySize, cudaMemcpyDeviceToHost));

//         CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetBoidIdx.data(), gpu.dBoids.targetBoidIdx, sizeof(int) * B, cudaMemcpyDeviceToHost));
//         CHECK_ERROR(cudaMemcpy(cpu.hBoids.targetBoidDistance.data(), gpu.dBoids.targetBoidDistance, sizeof(float) * B, cudaMemcpyDeviceToHost));

//         CHECK_ERROR(cudaMemcpy(cpu.hBoids.stamina.data(), gpu.dBoids.stamina, sizeof(float) * B, cudaMemcpyDeviceToHost));
//         CHECK_ERROR(cudaMemcpy(cpu.hBoids.resting.data(), gpu.dBoids.resting, sizeof(uint8_t) * B, cudaMemcpyDeviceToHost));
//     }

//     // Free device memory
//     CHECK_ERROR(cudaFree(gpu.dBoids.pos));
//     CHECK_ERROR(cudaFree(gpu.dBoids.vel));
//     CHECK_ERROR(cudaFree(gpu.dBoids.acc));
//     CHECK_ERROR(cudaFree(gpu.dBoids.targetPoint));
//     CHECK_ERROR(cudaFree(gpu.dBoids.type));
//     CHECK_ERROR(cudaFree(gpu.dBoids.stamina));
//     CHECK_ERROR(cudaFree(gpu.dBoids.resting));

//     CHECK_ERROR(cudaFree(gpu.dBoids.targetBoidIdx));
//     CHECK_ERROR(cudaFree(gpu.dBoids.targetBoidDistance));

//     CHECK_ERROR(cudaFree(gpu.dBoids.basicBoidIndices));
//     CHECK_ERROR(cudaFree(gpu.dBoids.predatorBoidIndices));
//     CHECK_ERROR(cudaFree(gpu.dBoids.obstacleBoidIndices));

//     CHECK_ERROR(cudaFree(gpu.dHashBasic));
//     CHECK_ERROR(cudaFree(gpu.dIndexBasic));
//     CHECK_ERROR(cudaFree(gpu.dHashPredator));
//     CHECK_ERROR(cudaFree(gpu.dIndexPredator));
//     CHECK_ERROR(cudaFree(gpu.dHashObstacle));
//     CHECK_ERROR(cudaFree(gpu.dIndexObstacle));
//     CHECK_ERROR(cudaFree(gpu.dCellStartBasic));
//     CHECK_ERROR(cudaFree(gpu.dCellEndBasic));
//     CHECK_ERROR(cudaFree(gpu.dCellStartPredator));
//     CHECK_ERROR(cudaFree(gpu.dCellEndPredator));
//     CHECK_ERROR(cudaFree(gpu.dCellStartObstacle));
//     CHECK_ERROR(cudaFree(gpu.dCellEndObstacle));

//     CHECK_ERROR(cudaFree(gpu.dCellX));
//     CHECK_ERROR(cudaFree(gpu.dCellY));
//     CHECK_ERROR(cudaFree(gpu.dCellZ));
// }
