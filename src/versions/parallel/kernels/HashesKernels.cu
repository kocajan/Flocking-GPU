// #include "versions/parallel/ParallelParameters.hpp"


// // Device helpers mirroring SpatialGrid logic
// __device__ inline int worldToCellIndexDevice(float pos, float worldSize, int cellCount, float cellSize, bool bounce) {
//     // Handle bounce case
//     if (bounce) {
//         // Get cell index
//         int cell = (int)floorf(pos / cellSize);

//         // Clamp to valid range
//         if (cell < 0) {
//             cell = 0;
//         }
        
//         if (cell >= cellCount) { 
//             cell = cellCount - 1;
//         }

//         return cell;
//     }

//     // Wrap around the position
//     if (pos < 0.0f) {
//         pos += worldSize;
//     } else if (pos >= worldSize) {
//         pos -= worldSize;
//     }

//     // Calculate cell index
//     int cell = (int)floorf(pos / cellSize);

//     // Wrap around the cell index
//     if (cell < 0) {           
//         cell += cellCount;
//     } else if (cell >= cellCount) {
//         cell -= cellCount;
//     }

//     return cell;
// }

// __device__ inline int flattenIndexDevice(int cX, int cY, int cZ, int numCellsX, int numCellsY) {
//     return (cZ * numCellsY + cY) * numCellsX + cX;
// }

// __global__ void kernelComputeHashes(ParallelParameters::GPUParams params, int boidCount, int* dHash, int* dIndex, int* boidIndices) {
//     // Compute global boid index
//     int currentTypeBoidIdx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (currentTypeBoidIdx >= boidCount) {
//         return;
//     }
//     // Get current boid index
//     int currentBoidIdx = boidIndices[currentTypeBoidIdx];

//     // Get current boids position
//     const Vec3 pos = params.dBoids.pos[currentBoidIdx];

//     // Compute cell indices
//     int cX = worldToCellIndexDevice(pos.x, params.worldX, params.numCellsX, params.cellSize, params.bounce);
//     int cY = worldToCellIndexDevice(pos.y, params.worldY, params.numCellsY, params.cellSize, params.bounce);
//     int cZ = 0;
//     if (!params.is2D) {
//         cZ = worldToCellIndexDevice(pos.z, params.worldZ, params.numCellsZ, params.cellSize, params.bounce);
//     }

//     // Linear hash
//     int hash = flattenIndexDevice(cX, cY, cZ, params.numCellsX, params.numCellsY);

//     // Write out
//     dHash[currentTypeBoidIdx] = hash;
//     dIndex[currentTypeBoidIdx] = currentBoidIdx;
// }