/**
 * \file tests/CellReset/main.cu
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Unit tests for CellReset kernels.
 */

#include <cuda_runtime.h>
#include <cassert>
#include <vector>
#include <iostream>

struct DeviceGrid;

__global__ void kernelResetCells(DeviceGrid dGrid);
__global__ void kernelBuildCellRanges(int boidCount, int* dHash, int* dCellStart, int* dCellEnd);


void check(cudaError_t err) {
    if (err != cudaSuccess) {
        std::cerr << "CUDA error: " << cudaGetErrorString(err) << std::endl;
        std::exit(1);
    }
}

// DeviceGrid structure adjusted to be minimal for the tests
struct DeviceGrid {
    int* cellStartBasic; int* cellEndBasic;

    int* cellStartPredator; int* cellEndPredator;

    int* cellStartObstacle; int* cellEndObstacle;

    int* cellX; int* cellY; int* cellZ;

    int numCellsX, numCellsY, numCellsZ;
    int totalCells;
};

// --------- TESTS ---------

void testKernelResetCells() {
    std::cout << "[TEST] kernelResetCells\n";

    const int nx = 2, ny = 2, nz = 2;
    const int totalCells = nx * ny * nz;

    DeviceGrid g{};
    g.numCellsX = nx;
    g.numCellsY = ny;
    g.numCellsZ = nz;
    g.totalCells = totalCells;

    auto alloc = [&](int** p) {
        check(cudaMalloc(p, totalCells * sizeof(int)));
    };

    alloc(&g.cellStartBasic);   alloc(&g.cellEndBasic);
    alloc(&g.cellStartPredator); alloc(&g.cellEndPredator);
    alloc(&g.cellStartObstacle); alloc(&g.cellEndObstacle);
    alloc(&g.cellX); alloc(&g.cellY); alloc(&g.cellZ);

    // Launch
    int threads = 128;
    int blocks = (totalCells + threads - 1) / threads;
    kernelResetCells<<<blocks, threads>>>(g);
    check(cudaDeviceSynchronize());

    // Copy back
    std::vector<int> sB(totalCells), eB(totalCells);
    std::vector<int> sP(totalCells), eP(totalCells);
    std::vector<int> sO(totalCells), eO(totalCells);
    std::vector<int> cx(totalCells), cy(totalCells), cz(totalCells);

    check(cudaMemcpy(sB.data(), g.cellStartBasic, totalCells*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(eB.data(), g.cellEndBasic,   totalCells*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(sP.data(), g.cellStartPredator, totalCells*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(eP.data(), g.cellEndPredator,   totalCells*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(sO.data(), g.cellStartObstacle, totalCells*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(eO.data(), g.cellEndObstacle,   totalCells*sizeof(int), cudaMemcpyDeviceToHost));

    check(cudaMemcpy(cx.data(), g.cellX, totalCells*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(cy.data(), g.cellY, totalCells*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(cz.data(), g.cellZ, totalCells*sizeof(int), cudaMemcpyDeviceToHost));

    // Validate reset + coordinate mapping
    for (int h = 0; h < totalCells; h++) {
        assert(sB[h] == -1 && eB[h] == -1);
        assert(sP[h] == -1 && eP[h] == -1);
        assert(sO[h] == -1 && eO[h] == -1);

        int x = h % nx;
        int t = h / nx;
        int y = t % ny;
        int z = t / ny;

        assert(cx[h] == x);
        assert(cy[h] == y);
        assert(cz[h] == z);
    }

    std::cout << "  OK\n";

    cudaFree(g.cellStartBasic); cudaFree(g.cellEndBasic);
    cudaFree(g.cellStartPredator); cudaFree(g.cellEndPredator);
    cudaFree(g.cellStartObstacle); cudaFree(g.cellEndObstacle);
    cudaFree(g.cellX); cudaFree(g.cellY); cudaFree(g.cellZ);
}

void testKernelBuildCellRanges() {
    std::cout << "[TEST] kernelBuildCellRanges\n";

    // Sorted cell hash list
    // - cells: 0 0 0 | 1 1 | 3
    std::vector<int> hHash = {0, 0, 0, 1, 1, 3};
    int boidCount = (int)hHash.size();
    const int numCells = 4;

    std::vector<int> hStart(numCells, -1);
    std::vector<int> hEnd(numCells, -1);

    int *dHash=nullptr, *dStart=nullptr, *dEnd=nullptr;
    check(cudaMalloc(&dHash,  boidCount*sizeof(int)));
    check(cudaMalloc(&dStart, numCells*sizeof(int)));
    check(cudaMalloc(&dEnd,   numCells*sizeof(int)));

    check(cudaMemcpy(dHash, hHash.data(), boidCount*sizeof(int), cudaMemcpyHostToDevice));
    check(cudaMemcpy(dStart, hStart.data(), numCells*sizeof(int), cudaMemcpyHostToDevice));
    check(cudaMemcpy(dEnd,   hEnd.data(),   numCells*sizeof(int), cudaMemcpyHostToDevice));

    int threads = 128;
    int blocks = (boidCount + threads - 1) / threads;
    kernelBuildCellRanges<<<blocks, threads>>>(boidCount, dHash, dStart, dEnd);
    check(cudaDeviceSynchronize());

    check(cudaMemcpy(hStart.data(), dStart, numCells*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(hEnd.data(),   dEnd,   numCells*sizeof(int), cudaMemcpyDeviceToHost));

    // Expected:
    // - cell 0 -> [0,3)
    // - cell 1 -> [3,5)
    // - cell 2 -> -1
    // - cell 3 -> [5,6)
    assert(hStart[0] == 0 && hEnd[0] == 3);
    assert(hStart[1] == 3 && hEnd[1] == 5);
    assert(hStart[2] == -1 && hEnd[2] == -1);
    assert(hStart[3] == 5 && hEnd[3] == 6);

    std::cout << "  OK\n";

    cudaFree(dHash);
    cudaFree(dStart);
    cudaFree(dEnd);
}

int main() {
    testKernelResetCells();
    testKernelBuildCellRanges();
    std::cout << "All tests passed.\n";
    return 0;
}
