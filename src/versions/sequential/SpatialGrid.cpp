#include <cmath>
#include <algorithm>
#include <unordered_set>

#include "versions/sequential/SpatialGrid.hpp"


SpatialGrid::SpatialGrid(const SequentialParameters& params)
    : cellSize(params.cellSize),
      numCellsX(params.numCellsX),
      numCellsY(params.numCellsY),
      numCellsZ(params.numCellsZ),
      is2D(params.is2D),
      bounce(params.bounce),
      isBuilt(false)
{
    if (numCellsX <= 0 || numCellsY <= 0 || numCellsZ <= 0) {
        hasZeroCells = true;
        numCellsX = numCellsY = numCellsZ = 0;
    } else {
        hasZeroCells = false;
        size_t total = static_cast<size_t>(numCellsX) * numCellsY * numCellsZ;
        basicCells.resize(total);
        predatorCells.resize(total);
        obstacleCells.resize(total);
    }

    // Reserve scratch buffer capacity up-front
    scratch.reserve(1024);
}

int SpatialGrid::flattenIndex(int cx, int cy, int cz) const {
    return (cz * numCellsY + cy) * numCellsX + cx;
}

int SpatialGrid::worldToCellIndex(float p, float worldSize, int cellCount) const {
    if (bounce) {
        int c = (int)std::floor(p / cellSize);
        return std::clamp(c, 0, cellCount - 1);
    }

    if (p < 0) p += worldSize;
    if (p >= worldSize) p -= worldSize;

    int c = (int)std::floor(p / cellSize);

    if (c < 0)      c += cellCount;
    if (c >= cellCount) c -= cellCount;

    return c;
}

void SpatialGrid::insertPointObject(std::vector<Cell>& grid, const SequentialParameters& params, int idx) {
    const Vec3& p = params.boids.pos[idx];

    int cx = worldToCellIndex(p.x, params.worldX, numCellsX);
    int cy = worldToCellIndex(p.y, params.worldY, numCellsY);
    int cz = is2D ? 0 : worldToCellIndex(p.z, params.worldZ, numCellsZ);
    grid[flattenIndex(cx,cy,cz)].items.push_back(idx);
}


void SpatialGrid::insertRadialObject(std::vector<Cell>& grid, const SequentialParameters& params, int idx, float radius) {
    const Vec3& p = params.boids.pos[idx];

    float minX = p.x - radius;
    float maxX = p.x + radius;
    float minY = p.y - radius;
    float maxY = p.y + radius;

    int cx0 = worldToCellIndex(minX, params.worldX, numCellsX);
    int cx1 = worldToCellIndex(maxX, params.worldX, numCellsX);
    int cy0 = worldToCellIndex(minY, params.worldY, numCellsY);
    int cy1 = worldToCellIndex(maxY, params.worldY, numCellsY);

    int cz0 = 0, cz1 = 0;

    if (!is2D) {
        float minZ = p.z - radius;
        float maxZ = p.z + radius;

        cz0 = worldToCellIndex(minZ, params.worldZ, numCellsZ);
        cz1 = worldToCellIndex(maxZ, params.worldZ, numCellsZ);
    }

    for (int cx = cx0; cx <= cx1; ++cx)
    for (int cy = cy0; cy <= cy1; ++cy)
    for (int cz = cz0; cz <= cz1; ++cz) {
        grid[flattenIndex(cx,cy,cz)].items.push_back(idx);
    }
}


void SpatialGrid::build(const SequentialParameters& params) {
    if (hasZeroCells) {
        isBuilt = true;
        return;
    }

    if (isBuilt) {
        for (auto& c : basicCells)    c.items.clear();
        for (auto& c : predatorCells) c.items.clear();
        for (auto& c : obstacleCells) c.items.clear();
    }

    for (int i = 0; i < params.boidCount; ++i) {
        const BoidType type = static_cast<BoidType>(params.boids.type[i]);

        switch (type) {
            case BoidType::Basic:
                insertPointObject(basicCells, params, i);
                break;
            case BoidType::Predator:
                insertPointObject(predatorCells, params, i);
                break;
            case BoidType::Obstacle:
                insertRadialObject(obstacleCells, params, i, params.obstacleRadius);
                break;
            default:
                break;
        }
    }

    isBuilt = true;
}

const std::vector<int>& SpatialGrid::getNeighborIndices(const SequentialParameters& params, 
                                                        int boidIndex, BoidType neighborType) const {
    scratch.clear();

    if (hasZeroCells)
        return scratch;

    const std::vector<Cell>* grid =
        (neighborType == BoidType::Basic)    ? &basicCells :
        (neighborType == BoidType::Predator) ? &predatorCells :
        (neighborType == BoidType::Obstacle) ? &obstacleCells :
                                               nullptr;

    if (!grid) {
        printf("Warning: Requested neighbor type has no associated grid. (type=%d)\n", static_cast<int>(neighborType));
        return scratch;
    }

    const Vec3& p = params.boids.pos[boidIndex];

    int cx = worldToCellIndex(p.x, params.worldX, numCellsX);
    int cy = worldToCellIndex(p.y, params.worldY, numCellsY);
    int cz = is2D ? 0 : worldToCellIndex(p.z, params.worldZ, numCellsZ);
    int zmin = is2D ? 0 : -1;
    int zmax = is2D ? 0 :  1;

    for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
    for (int dz = zmin; dz <= zmax; ++dz) {
        int nx = cx + dx;
        int ny = cy + dy;
        int nz = cz + dz;

        if (bounce) {
            if (nx < 0 || nx >= numCellsX ||
                ny < 0 || ny >= numCellsY ||
                nz < 0 || nz >= numCellsZ)
                continue;
        } else {
            if (nx < 0) nx += numCellsX;
            if (nx >= numCellsX) nx -= numCellsX;

            if (ny < 0) ny += numCellsY;
            if (ny >= numCellsY) ny -= numCellsY;

            if (!is2D) {
                if (nz < 0) nz += numCellsZ;
                if (nz >= numCellsZ) nz -= numCellsZ;
            } else {
                nz = 0;
            }
        }

        const auto& cell = (*grid)[flattenIndex(nx, ny, nz)].items;

        if (!cell.empty())
            scratch.insert(scratch.end(), cell.begin(), cell.end());
    }

    if (neighborType == BoidType::Obstacle) {
        std::unordered_set<int> uniqueIndices(scratch.begin(), scratch.end());
        scratch.assign(uniqueIndices.begin(), uniqueIndices.end());
    }

    return scratch;
}

