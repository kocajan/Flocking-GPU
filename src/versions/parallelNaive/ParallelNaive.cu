#include <cstdio>
#include <iostream>
#include <cuda_runtime.h>

#include "versions/parallelNaive/ParallelNaive.hpp"
#include "versions/parallelNaive/ParallelNaiveParameters.hpp"


#define CHECK_ERROR( error ) ( HandleError( error, __FILE__, __LINE__ ) )


static void HandleError(cudaError_t error, const char* file, int line) { 
    if (error != cudaSuccess) { 
        std::cout << cudaGetErrorString(error) << " in " << file << " at line " << line << std::endl; 
        scanf(" "); 
        exit(EXIT_FAILURE); 
    } 
}


__global__ void simulationStepParallelNaiveKernel(ParallelNaiveParameters::GPUParams params);


void simulationStepParallelNaive(ParallelNaiveParameters& params) {
    const int N = params.cpu.hBoidCount;
    if (N == 0 || params.cpu.gridSize == 0)
        return;

    const size_t vec3Size = sizeof(Vec3) * N;

    //
    // ============================================================
    // 1) Allocate device SoA buffers
    // ============================================================
    //
    DeviceBoids d{};
    Boids&      h = params.cpu.hBoids;

    CHECK_ERROR(cudaMalloc(&d.pos,         vec3Size));
    CHECK_ERROR(cudaMalloc(&d.vel,         vec3Size));
    CHECK_ERROR(cudaMalloc(&d.acc,         vec3Size));
    CHECK_ERROR(cudaMalloc(&d.targetPoint, vec3Size));

    CHECK_ERROR(cudaMalloc(&d.type,        sizeof(uint8_t) * N));
    CHECK_ERROR(cudaMalloc(&d.stamina,     sizeof(float)   * N));
    CHECK_ERROR(cudaMalloc(&d.resting,     sizeof(uint8_t) * N));

    CHECK_ERROR(cudaMalloc(&d.targetBoidIdx,      sizeof(int)   * N));
    CHECK_ERROR(cudaMalloc(&d.targetBoidDistance, sizeof(float) * N));

    CHECK_ERROR(cudaMalloc(&d.basicBoidIndices,    sizeof(size_t) * h.basicBoidCount));
    CHECK_ERROR(cudaMalloc(&d.predatorBoidIndices, sizeof(size_t) * h.predatorBoidCount));
    CHECK_ERROR(cudaMalloc(&d.obstacleBoidIndices, sizeof(size_t) * h.obstacleBoidCount));

    //
    // ============================================================
    // 2) Upload arrays (host → device)
    // ============================================================
    //
    CHECK_ERROR(cudaMemcpy(d.pos,         h.pos.data(),         vec3Size, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(d.vel,         h.vel.data(),         vec3Size, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(d.acc,         h.acc.data(),         vec3Size, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(d.targetPoint, h.targetPoint.data(), vec3Size, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(d.type,        h.type.data(),        sizeof(uint8_t) * N, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(d.stamina,     h.stamina.data(),     sizeof(float)   * N, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(d.resting,     h.resting.data(),     sizeof(uint8_t) * N, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(d.targetBoidIdx,      h.targetBoidIdx.data(),      sizeof(int)   * N, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(d.targetBoidDistance, h.targetBoidDistance.data(), sizeof(float) * N, cudaMemcpyHostToDevice));

    CHECK_ERROR(cudaMemcpy(d.basicBoidIndices,    h.basicBoidIndices.data(),    sizeof(size_t) * h.basicBoidCount,    cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(d.predatorBoidIndices, h.predatorBoidIndices.data(), sizeof(size_t) * h.predatorBoidCount, cudaMemcpyHostToDevice));
    CHECK_ERROR(cudaMemcpy(d.obstacleBoidIndices, h.obstacleBoidIndices.data(), sizeof(size_t) * h.obstacleBoidCount, cudaMemcpyHostToDevice));

    //
    // ============================================================
    // 3) Fill GPUParams from CPUParams
    // ============================================================
    //
    ParallelNaiveParameters::GPUParams g{};

    g.dBoids     = d;
    g.dBoidCount = N;

    g.dIs2D   = params.cpu.hIs2D;
    g.dBounce = params.cpu.hBounce;

    g.dBasicBoidRadius = params.cpu.hBasicBoidRadius;
    g.dPredatorRadius  = params.cpu.hPredatorRadius;
    g.dObstacleRadius  = params.cpu.hObstacleRadius;

    g.dWorldX = params.cpu.hWorldX;
    g.dWorldY = params.cpu.hWorldY;
    g.dWorldZ = params.cpu.hWorldZ;

    g.dEps = params.cpu.hEps;
    g.dDt  = params.cpu.hDt;

    g.dVisionRangeBasic    = params.cpu.hVisionRangeBasic;
    g.dVisionRangeBasic2   = params.cpu.hVisionRangeBasic2;
    g.dVisionRangePredator = params.cpu.hVisionRangePredator;

    g.dMaxForce                   = params.cpu.hMaxForce;
    g.dObstacleAvoidanceMultiplier = params.cpu.hObstacleAvoidanceMultiplier;
    g.dMouseInteractionMultiplier  = params.cpu.hMouseInteractionMultiplier;

    g.dCohesionWeightBasic        = params.cpu.hCohesionWeightBasic;
    g.dAlignmentWeightBasic       = params.cpu.hAlignmentWeightBasic;
    g.dSeparationWeightBasic      = params.cpu.hSeparationWeightBasic;
    g.dTargetAttractionWeightBasic= params.cpu.hTargetAttractionWeightBasic;

    g.dCruisingSpeedBasic = params.cpu.hCruisingSpeedBasic;
    g.dMaxSpeedBasic      = params.cpu.hMaxSpeedBasic;
    g.dMinSpeedBasic      = params.cpu.hMinSpeedBasic;

    g.dCruisingSpeedPredator = params.cpu.hCruisingSpeedPredator;
    g.dMaxSpeedPredator      = params.cpu.hMaxSpeedPredator;
    g.dMinSpeedPredator      = params.cpu.hMinSpeedPredator;

    g.dMaxStaminaPredator        = params.cpu.hMaxStaminaPredator;
    g.dStaminaRecoveryRatePredator = params.cpu.hStaminaRecoveryRatePredator;
    g.dStaminaDrainRatePredator    = params.cpu.hStaminaDrainRatePredator;

    g.dDrag  = params.cpu.hDrag;
    g.dNoise = params.cpu.hNoise;
    g.dNumStepsToStopDueToMaxDrag = params.cpu.hNumStepsToStopDueToMaxDrag;

    g.dBounceFactor = params.cpu.hBounceFactor;

    g.dMaxNeighborsBasic = params.cpu.hMaxNeighborsBasic;

    g.dMaxDistanceBetweenPoints  = params.cpu.hMaxDistanceBetweenPoints;
    g.dMaxDistanceBetweenPoints2 = params.cpu.hMaxDistanceBetweenPoints2;

    // interaction → packed device struct
    g.dInteraction.type   = static_cast<uint8_t>(params.cpu.hInteraction.type);
    g.dInteraction.point  = params.cpu.hInteraction.point;

    //
    // ============================================================
    // 4) Launch kernel
    // ============================================================
    //
    simulationStepParallelNaiveKernel<<<params.cpu.gridSize,
                                         params.cpu.blockSize>>>(g);

    CHECK_ERROR(cudaPeekAtLastError());
    CHECK_ERROR(cudaDeviceSynchronize());

    //
    // ============================================================
    // 5) Download results (device → host)
    // ============================================================
    //
    CHECK_ERROR(cudaMemcpy(h.pos.data(), d.pos, vec3Size, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(h.vel.data(), d.vel, vec3Size, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(h.acc.data(), d.acc, vec3Size, cudaMemcpyDeviceToHost));

    CHECK_ERROR(cudaMemcpy(h.targetBoidIdx.data(),      d.targetBoidIdx,      sizeof(int)   * N, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(h.targetBoidDistance.data(), d.targetBoidDistance, sizeof(float) * N, cudaMemcpyDeviceToHost));

    CHECK_ERROR(cudaMemcpy(h.stamina.data(), d.stamina, sizeof(float)   * N, cudaMemcpyDeviceToHost));
    CHECK_ERROR(cudaMemcpy(h.resting.data(), d.resting, sizeof(uint8_t) * N, cudaMemcpyDeviceToHost));

    //
    // ============================================================
    // 6) Free device buffers
    // ============================================================
    //
    cudaFree(d.pos);
    cudaFree(d.vel);
    cudaFree(d.acc);
    cudaFree(d.targetPoint);

    cudaFree(d.type);
    cudaFree(d.stamina);
    cudaFree(d.resting);

    cudaFree(d.targetBoidIdx);
    cudaFree(d.targetBoidDistance);

    cudaFree(d.basicBoidIndices);
    cudaFree(d.predatorBoidIndices);
    cudaFree(d.obstacleBoidIndices);
}
