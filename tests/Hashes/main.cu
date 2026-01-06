/**
 * \file tests/Hashes/main.cu
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Unit tests for Hashes kernels.
 */

#include <cuda_runtime.h>
#include <vector>
#include <cassert>
#include <iostream>


// ---------------------- TEST FRAMEWORK ----------------------
void check(cudaError_t e) {
    if (e != cudaSuccess) {
        std::cerr << "CUDA error: " << cudaGetErrorString(e) << "\n";
        std::exit(1);
    }
}

// Structures minimal for the tests
struct Vec3 { float x,y,z; };

struct DeviceGrid {
    float cellSize;
    int numCellsX;
    int numCellsY;
    int numCellsZ;
};

struct GPUParams {
    DeviceGrid dGrid;
    bool is2D;
    bool bounce;
    float worldX;
    float worldY;
    float worldZ;
};

__global__ void kernelComputeHashes(GPUParams params, Vec3* dPos, int boidCount, int* dHash, int* dIndex);

int flattenRef(int cX,int cY,int cZ, int nx, int ny) {
    // From coordinates to linear index
    return (cZ * ny + cY) * nx + cX;
}

void runTest(const char* name, const std::vector<Vec3>& positions, 
             GPUParams params, const std::vector<int>& expected) {
    std::cout << "[TEST] " << name << "\n";

    const int n = (int)positions.size();

    Vec3* dPos{};
    int *dHash{}, *dIndex{};
    std::vector<int> hHash(n), hIndex(n);

    check(cudaMalloc(&dPos,  n*sizeof(Vec3)));
    check(cudaMalloc(&dHash, n*sizeof(int)));
    check(cudaMalloc(&dIndex,n*sizeof(int)));

    check(cudaMemcpy(dPos, positions.data(),
                     n*sizeof(Vec3), cudaMemcpyHostToDevice));

    int threads = 128;
    int blocks = (n + threads - 1) / threads;

    kernelComputeHashes<<<blocks, threads>>>(params, dPos, n, dHash, dIndex);
    check(cudaDeviceSynchronize());

    check(cudaMemcpy(hHash.data(),  dHash,  n*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(hIndex.data(), dIndex, n*sizeof(int), cudaMemcpyDeviceToHost));

    for (int i = 0; i < n; i++) {
        assert(hIndex[i] == i);
        assert(hHash[i] == expected[i]);
    }

    std::cout << "  OK\n";

    cudaFree(dPos);
    cudaFree(dHash);
    cudaFree(dIndex);
}

// ---------------------- TEST CASES ----------------------

void test2D_wrap() {
    GPUParams p{};
    p.is2D = true;
    p.bounce = false;

    p.dGrid.cellSize = 1;
    p.dGrid.numCellsX = 4;
    p.dGrid.numCellsY = 4;
    p.dGrid.numCellsZ = 1;

    // world == grid span
    p.worldX = p.dGrid.numCellsX * p.dGrid.cellSize; // 4
    p.worldY = p.dGrid.numCellsY * p.dGrid.cellSize; // 4

    std::vector<Vec3> pos = {
        {0.1f, 0.1f, 0},   // (0, 0)
        {3.9f, 0.2f, 0},   // (3, 0)
        {4.1f, 0.2f, 0},   // wrap -> (0, 0)
        {-0.2f, 2.2f, 0},  // wrap -> (3, 2)
    };

    std::vector<int> expect = {
        flattenRef(0, 0, 0, 4, 4),
        flattenRef(3, 0, 0, 4, 4),
        flattenRef(0, 0, 0, 4, 4),
        flattenRef(3, 2, 0, 4, 4),
    };

    runTest("2D wrap mode", pos, p, expect);
}

void test2D_bounce() {
    GPUParams p{};
    p.is2D = true;
    p.bounce = true;

    p.dGrid.cellSize = 1;
    p.dGrid.numCellsX = 4;
    p.dGrid.numCellsY = 4;

    p.worldX = p.dGrid.numCellsX * p.dGrid.cellSize; // 4
    p.worldY = p.dGrid.numCellsY * p.dGrid.cellSize; // 4

    std::vector<Vec3> pos = {
        {-5.f, 1.1f, 0},  // -> clamp x=0
        {12.f, 2.9f, 0},  // -> clamp x=3
    };

    std::vector<int> expect = {
        flattenRef(0, 1, 0, 4, 4),
        flattenRef(3, 2, 0, 4, 4),
    };

    runTest("2D bounce clamp", pos, p, expect);
}

void test3D_wrap() {
    GPUParams p{};
    p.is2D = false;
    p.bounce = false;

    p.dGrid.cellSize = 1;
    p.dGrid.numCellsX = 2;
    p.dGrid.numCellsY = 2;
    p.dGrid.numCellsZ = 2;

    p.worldX = p.dGrid.numCellsX * p.dGrid.cellSize; // 2
    p.worldY = p.dGrid.numCellsY * p.dGrid.cellSize; // 2
    p.worldZ = p.dGrid.numCellsZ * p.dGrid.cellSize; // 2

    std::vector<Vec3> pos = {
        {0.1f, 0.1f, 0.1f},  // (0,0,0)
        {1.9f, 1.9f, 1.9f},  // (1,1,1)
        {2.1f, 2.1f, 2.1f},  // wrap -> (0,0,0)
    };

    std::vector<int> expect = {
        flattenRef(0, 0, 0, 2, 2),
        flattenRef(1, 1, 1, 2, 2),
        flattenRef(0, 0, 0, 2, 2),
    };

    runTest("3D wrap mode", pos, p, expect);
}

int main() {
    test2D_wrap();
    test2D_bounce();
    test3D_wrap();

    std::cout << "All minimal-struct hash tests passed.\n";
    return 0;
}
