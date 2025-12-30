#include <cmath>
#include <algorithm>
#include <unordered_set>

#include "versions/sequential/SpatialGrid.hpp"


SpatialGrid::SpatialGrid(const SequentialParameters& params)
    : cellSize(params.cellSize),
      cellsX(params.cellsX),
      cellsY(params.cellsY),
      cellsZ(params.cellsZ),
      is2D(params.is2D),
      bounce(params.bounce),
      isBuilt(false)
{
    if (cellsX <= 0 || cellsY <= 0 || cellsZ <= 0) {
        hasZeroCells = true;
        cellsX = cellsY = cellsZ = 0;
    } else {
        hasZeroCells = false;
        size_t total = static_cast<size_t>(cellsX) * cellsY * cellsZ;
        basicCells.resize(total);
        predatorCells.resize(total);
        obstacleCells.resize(total);
    }

    // Reserve scratch buffer capacity up-front
    scratch.reserve(1024);
}

int SpatialGrid::flattenIndex(int cx, int cy, int cz) const {
    return (cz * cellsY + cy) * cellsX + cx;
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
    const Boid& b = params.boids[idx];

    int cx = worldToCellIndex(b.pos.x, params.worldX, cellsX);
    int cy = worldToCellIndex(b.pos.y, params.worldY, cellsY);
    int cz = is2D ? 0 : worldToCellIndex(b.pos.z, params.worldZ, cellsZ);

    grid[flattenIndex(cx,cy,cz)].items.push_back(idx);
}


void SpatialGrid::insertRadialObject(std::vector<Cell>& grid, const SequentialParameters& params, int idx, float radius) {
    const Boid& b = params.boids[idx];

    float minX = b.pos.x - radius;
    float maxX = b.pos.x + radius;
    float minY = b.pos.y - radius;
    float maxY = b.pos.y + radius;

    int cx0 = worldToCellIndex(minX, params.worldX, cellsX);
    int cx1 = worldToCellIndex(maxX, params.worldX, cellsX);
    int cy0 = worldToCellIndex(minY, params.worldY, cellsY);
    int cy1 = worldToCellIndex(maxY, params.worldY, cellsY);

    int cz0 = 0, cz1 = 0;

    if (!is2D) {
        float minZ = b.pos.z - radius;
        float maxZ = b.pos.z + radius;

        cz0 = worldToCellIndex(minZ, params.worldZ, cellsZ);
        cz1 = worldToCellIndex(maxZ, params.worldZ, cellsZ);
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

    for (int i = 0; i < params.boidCount; ++i)
    {
        const Boid& b = params.boids[i];

        switch (b.type) {
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

    const Boid& b = params.boids[boidIndex];

    int cx = worldToCellIndex(b.pos.x, params.worldX, cellsX);
    int cy = worldToCellIndex(b.pos.y, params.worldY, cellsY);
    int cz = is2D ? 0 : worldToCellIndex(b.pos.z, params.worldZ, cellsZ);

    int zmin = is2D ? 0 : -1;
    int zmax = is2D ? 0 :  1;

    for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
    for (int dz = zmin; dz <= zmax; ++dz) {
        int nx = cx + dx;
        int ny = cy + dy;
        int nz = cz + dz;

        if (bounce) {
            if (nx < 0 || nx >= cellsX ||
                ny < 0 || ny >= cellsY ||
                nz < 0 || nz >= cellsZ)
                continue;
        } else {
            if (nx < 0) nx += cellsX;
            if (nx >= cellsX) nx -= cellsX;

            if (ny < 0) ny += cellsY;
            if (ny >= cellsY) ny -= cellsY;

            if (!is2D) {
                if (nz < 0) nz += cellsZ;
                if (nz >= cellsZ) nz -= cellsZ;
            } else {
                nz = 0;
            }
        }

        const auto& cell = (*grid)[flattenIndex(nx,ny,nz)].items;

        if (!cell.empty())
            scratch.insert(scratch.end(), cell.begin(), cell.end());
    }

    if (neighborType == BoidType::Obstacle) {
        std::unordered_set<int> uniqueIndices(scratch.begin(), scratch.end());
        scratch.assign(uniqueIndices.begin(), uniqueIndices.end());
    }

    return scratch;
}

