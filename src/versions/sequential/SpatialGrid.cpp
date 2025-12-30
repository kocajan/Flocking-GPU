#include <cmath>
#include <algorithm>

#include "versions/sequential/SpatialGrid.hpp"


SpatialGrid::SpatialGrid(const SequentialParameters& params)
    : cellSize(params.cellSize),
      cellsX(params.cellsX),
      cellsY(params.cellsY),
      cellsZ(params.cellsZ),
      is2D(params.is2D),
      bounce(params.bounce) {

    isBuilt = false;

    if (cellsX <= 0 || cellsY <= 0 || cellsZ <= 0) {
        hasZeroCells = true;
        cellsX = 0;
        cellsY = 0;
        cellsZ = 0;
    } else {
        hasZeroCells = false;
        cells.resize(cellsX * cellsY * cellsZ);
    }
}


int SpatialGrid::flattenIndex(int cx, int cy, int cz) const {
    return (cz * cellsY + cy) * cellsX + cx;
}

int SpatialGrid::worldToCellIndex(float p, float worldSize, int cellCount) const {
    if (bounce) {
        int c = (int)std::floor(p / cellSize);
        return std::clamp(c, 0, cellCount - 1);
    }

    // Wrapping mode
    if (p < 0) p += worldSize;
    if (p >= worldSize) p -= worldSize;

    int c = (int)std::floor(p / cellSize);

    if (c < 0)      c += cellCount;
    if (c >= cellCount) c -= cellCount;

    return c;
}

void SpatialGrid::insertPointObject(const SequentialParameters& params, int idx) {
    if (hasZeroCells)
        return;
    const Boid& b = params.boids[idx];

    int cx = worldToCellIndex(b.pos.x, params.worldX, cellsX);
    int cy = worldToCellIndex(b.pos.y, params.worldY, cellsY);
    int cz = is2D ? 0 : worldToCellIndex(b.pos.z, params.worldZ, cellsZ);

    cells[flattenIndex(cx,cy,cz)].push_back(idx);
}

void SpatialGrid::insertRadialObject(const SequentialParameters& params, int idx, float radius) {
    if (hasZeroCells)
        return;
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
    for (int cz = cz0; cz <= cz1; ++cz)
    {
        cells[flattenIndex(cx,cy,cz)].push_back(idx);
    }
}

void SpatialGrid::build(const SequentialParameters& params) {
    if (hasZeroCells) {
        isBuilt = true;
        return;
    }

    for (auto& c : cells)
        c.clear();

    for (int i = 0; i < params.boidCount; ++i) {
        const Boid& b = params.boids[i];

        float radius = 0.0f;
        if (b.type == BoidType::Obstacle) {
            radius = params.obstacleRadius;
        } else if (b.type == BoidType::Basic) {
            radius = params.basicBoidRadius;
        } else if (b.type == BoidType::Predator) {
            radius = params.predatorRadius;
        } else {
            printf("Warning: Unknown boid type encountered in SpatialGrid build. (type=%d)\n", static_cast<int>(b.type));
            continue;
        }

        insertRadialObject(params, i, radius);
    }
    isBuilt = true;
}

std::unordered_set<int> SpatialGrid::getNeighborIndices(const SequentialParameters& params, int boidIndex) const {
    if (hasZeroCells) {
        return {};
    }
    const Boid& b = params.boids[boidIndex];

    int cx = worldToCellIndex(b.pos.x, params.worldX, cellsX);
    int cy = worldToCellIndex(b.pos.y, params.worldY, cellsY);
    int cz = is2D ? 0 : worldToCellIndex(b.pos.z, params.worldZ, cellsZ);

    std::unordered_set<int> result;
    result.reserve(64);

    int zmin = is2D ? 0 : -1;
    int zmax = is2D ? 0 :  1;

    for (int dx=-1; dx<=1; ++dx)
    for (int dy=-1; dy<=1; ++dy)
    for (int dz=zmin; dz<=zmax; ++dz) {
        int nx = cx + dx;
        int ny = cy + dy;
        int nz = cz + dz;

        if (bounce) {
            // Hard boundaries
            if (nx < 0 || nx >= cellsX || ny < 0 || ny >= cellsY || nz < 0 || nz >= cellsZ)
                continue;
        } else {
            // Wrapping mode
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

        const auto& cell = cells[flattenIndex(nx,ny,nz)];

        if (cell.empty()) {
            continue;
        }

        for (int idx : cell) {
            result.insert(idx);
        }
    }

    return result;
}
