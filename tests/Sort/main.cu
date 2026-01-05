#include <cuda_runtime.h>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

// Host declaration of the sorting function implemented in BoidSorting.cu
void sortBoidsByHash(int boidCount, int* dHash, int* dIndex, int blockSize);

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
    std::cout << "[TEST] small fixed\n";

    const int boidCount = 5;
    const int N = nextPow2(boidCount);   // = 8
    const int blockSize = 4;

    printf("  boidCount=%d, N=%d\n", boidCount, N);

    // ----- Hardcoded test data -----
    // (unsorted hashes + identity indices)

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

    // // ----- Canonical sort for comparison -----
    // std::sort(gpu.begin(), gpu.end(),
    //     [](const auto& a, const auto& b){
    //         if (a.first != b.first) return a.first < b.first;
    //         return a.second < b.second;
    //     });

    // ----- Validate & print mismatch -----
    bool ok = true;

    for (int i = 0; i < boidCount; ++i) {
        printf(" GPU sorted [%d]: (hash=%d, index=%d)\n", i, hHash[i], hIndex[i]);
        printf(" CPU reference[%d]: (hash=%d, index=%d)\n", i, ref[i].first, ref[i].second);
        printf("\n");
        if (gpu[i] != ref[i]) {
            ok = false;

            std::cout << "\nMismatch at position " << i << "\n";
            std::cout << "  GPU: (" << gpu[i].first << ", " << gpu[i].second << ")\n";
            std::cout << "  CPU: (" << ref[i].first << ", " << ref[i].second << ")\n\n";

            std::cout << "GPU sorted result:\n";
            for (int j = 0; j < boidCount; ++j)
                std::cout << "  ["<<j<<"] (" << gpu[j].first << ", " << gpu[j].second << ")\n";

            std::cout << "\nCPU sorted reference:\n";
            for (int j = 0; j < boidCount; ++j)
                std::cout << "  ["<<j<<"] (" << ref[j].first << ", " << ref[j].second << ")\n";

            break;
        }
    }

    if (!ok) {
        std::cout << "\n[FAIL] small fixed test failed\n";
        std::exit(1);
    }

    std::cout << "  OK\n";
}

// -------------------- LARGE UNIQUE DATA TEST --------------------

void testLargeUnique() {
    std::cout << "[TEST] large unique\n";

    const int boidCount = 200000;      // "large" test
    const int N = nextPow2(boidCount);
    const int blockSize = 256;

    std::vector<int> hHash(N), hIndex(N);

    // Generate unique keys 0..boidCount-1 and shuffle
    std::vector<int> keys(boidCount);
    for (int i = 0; i < boidCount; ++i) {
        keys[i] = i;
    }

    std::mt19937 rng(98765);
    std::shuffle(keys.begin(), keys.end(), rng);

    for (int i = 0; i < boidCount; ++i) {
        hHash[i]  = keys[i];
        hIndex[i] = i;   // original index
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

    // Validate: exact match with CPU ref (no duplicate keys, so order is unique)
    for (int i = 0; i < boidCount; ++i) {
        // hashes must be in the same sorted order
        assert(hHash[i]  == ref[i].first);
        assert(hIndex[i] == ref[i].second);
        // also ensure non-decreasing
        if (i > 0) {
            assert(hHash[i-1] <= hHash[i]);
        }
    }

    std::cout << "  OK\n";

    cudaFree(dHash);
    cudaFree(dIndex);
}

int main() {
    testSmallFixed();
    testLargeUnique();

    std::cout << "All sorting tests passed.\n";
    return 0;
}
