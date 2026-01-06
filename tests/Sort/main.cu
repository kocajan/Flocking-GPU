#include <cuda_runtime.h>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <random>
#include <utility>
#include <vector>


void sortBoidsByHash(int boidCount, int* dHash, int* dIndex, int blockSize);

// -------------------- UTILITY FUNCTIONS --------------------

void check(cudaError_t err) {
    if (err != cudaSuccess) {
        std::cerr << "CUDA error: " << cudaGetErrorString(err) << "\n";
        std::exit(1);
    }
}

int nextPow2(int n) {
    int N = 1;
    while (N < n) N <<= 1;
    return N;
}

// -------------------- SMALL RANDOM TEST --------------------

void testSmallFixed() {
    std::cout << "[TEST] small set of fixed numbers\n";

    const int boidCount = 5;
    const int N = nextPow2(boidCount);   // = 8

    assert(N == 8);

    const int blockSize = 16;

    printf("  - boidCount=%d, N=%d\n", boidCount, N);

    // ----- Hardcoded test data -----

    std::vector<int> hHash = {
        42, 5, 17, 5, 99
    };

    std::vector<int> hIndex(boidCount);
    for (int i = 0; i < boidCount; ++i)
        hIndex[i] = i;

    // Allocate full N size for device buffers
    hHash.resize(N);
    hIndex.resize(N);

    // ----- CPU reference sort -----
    std::vector<std::pair<int,int>> ref;
    ref.reserve(boidCount);

    for (int i = 0; i < boidCount; ++i)
        ref.emplace_back(hHash[i], hIndex[i]);

    std::sort(ref.begin(), ref.end(),
        [](const auto& a, const auto& b){
            if (a.first != b.first) return a.first < b.first;
            return a.second < b.second;
        });

    // ----- Device buffers -----
    int *dHash=nullptr, *dIndex=nullptr;
    check(cudaMalloc(&dHash,  N * sizeof(int)));
    check(cudaMalloc(&dIndex, N * sizeof(int)));

    check(cudaMemcpy(dHash,  hHash.data(),  N*sizeof(int), cudaMemcpyHostToDevice));
    check(cudaMemcpy(dIndex, hIndex.data(), N*sizeof(int), cudaMemcpyHostToDevice));

    // ----- GPU sort -----
    sortBoidsByHash(boidCount, dHash, dIndex, blockSize);

    // ----- Copy back -----
    check(cudaMemcpy(hHash.data(),  dHash,  N*sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(hIndex.data(), dIndex, N*sizeof(int), cudaMemcpyDeviceToHost));

    cudaFree(dHash);
    cudaFree(dIndex);

    // ----- Build GPU slice -----
    std::vector<std::pair<int,int>> gpu;
    gpu.reserve(boidCount);

    for (int i = 0; i < boidCount; ++i)
        gpu.emplace_back(hHash[i], hIndex[i]);

    // ----- Canonical sort for comparison -----
    std::sort(gpu.begin(), gpu.end(),
        [](const auto& a, const auto& b){
            if (a.first != b.first) return a.first < b.first;
            return a.second < b.second;
        });

    // ----- Validate & print mismatch -----
    printf("  - sorted results:\n");
    printf("    CPU:\n");
    for (const auto& p : ref) {
        printf("      - (hash=%d, index=%d)\n", p.first, p.second);
    }

    printf("    GPU:\n");
    for (const auto& p : gpu) {
        printf("      - (hash=%d, index=%d)\n", p.first, p.second);
    }

    bool ok = true;
    for (int i = 0; i < boidCount; ++i) {
        if (ref[i] != gpu[i]) {
            ok = false;
            break;
        }
    }

    if (!ok) {
        std::cout << "\n[FAIL] small fixed test failed\n";
        std::exit(1);
    }

    std::cout << "  OK\n";
}

// -------------------- LARGE RANDOM DATA TEST --------------------

void testLargeRandom() {
    std::cout << "[TEST] large set of random numbers\n";

    const int boidCount = 200000;
    const int N = nextPow2(boidCount);
    const int blockSize = 256;

    std::vector<int> hHash(N), hIndex(N);

    // Generate 'boidCount' random keys
    std::mt19937 rng(12345); // fixed seed for reproducibility
    std::uniform_int_distribution<int> dist(0, 1000000);
    for (int i = 0; i < boidCount; ++i) {
        hHash[i] = dist(rng);
        hIndex[i] = i;
    }

    // CPU reference sort of pairs (hash, index)
    std::vector<std::pair<int,int>> ref;
    ref.reserve(boidCount);
    for (int i = 0; i < boidCount; ++i) {
        ref.emplace_back(hHash[i], hIndex[i]);
    }

    std::sort(ref.begin(), ref.end(),
              [](const auto& a, const auto& b) {
                  if (a.first != b.first) return a.first < b.first;
                  return a.second < b.second;
              });

    // Device buffers (size N)
    int* dHash = nullptr;
    int* dIndex = nullptr;
    check(cudaMalloc(&dHash,  N * sizeof(int)));
    check(cudaMalloc(&dIndex, N * sizeof(int)));

    check(cudaMemcpy(dHash,  hHash.data(),  N * sizeof(int), cudaMemcpyHostToDevice));
    check(cudaMemcpy(dIndex, hIndex.data(), N * sizeof(int), cudaMemcpyHostToDevice));

    // GPU sort
    sortBoidsByHash(boidCount, dHash, dIndex, blockSize);

    // Copy back first boidCount entries
    check(cudaMemcpy(hHash.data(),  dHash,  N * sizeof(int), cudaMemcpyDeviceToHost));
    check(cudaMemcpy(hIndex.data(), dIndex, N * sizeof(int), cudaMemcpyDeviceToHost));

    // First validate non-decreasing order of hashes
    for (int i = 1; i < boidCount; ++i) {
        assert(hHash[i-1] <= hHash[i]);
    } 

    // Run cannonical sort on GPU slice for comparison
    std::vector<std::pair<int,int>> gpu;
    gpu.reserve(boidCount);

    for (int i = 0; i < boidCount; ++i)
        gpu.emplace_back(hHash[i], hIndex[i]);

    std::sort(gpu.begin(), gpu.end(),
        [](const auto& a, const auto& b){
            if (a.first != b.first) return a.first < b.first;
            return a.second < b.second;
        });

    for (int i = 0; i < boidCount; ++i) {
        // Hash and index match reference
        assert(gpu[i].first  == ref[i].first);
        assert(gpu[i].second == ref[i].second);

        // Ensure non-decreasing order
        if (i > 0) {
            assert(gpu[i-1].first <= gpu[i].first);
        }
    }

    std::cout << "  OK\n";

    cudaFree(dHash);
    cudaFree(dIndex);
}

int main() {
    testSmallFixed();
    testLargeRandom();

    std::cout << "All sorting tests passed.\n";
    return 0;
}
