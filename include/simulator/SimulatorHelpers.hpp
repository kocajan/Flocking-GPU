#pragma once

#include <vector>
#include <cstdint>

#include "core/Boids.hpp"
#include "core/SimState.hpp"




//  Spawn or remove boids to regulate population towards target
void regulateBoidPopulation(
    SimState& s,
    BoidType type,
    int& count,
    int target
);

// Spawn multiple boids of given type at random positions
void spawnBoids(
    SimState& s,
    BoidType type,
    int& count,
    int howMany,
    float x = -1,
    float y = -1,
    float z = -1
);

// Spawn a single boid of given type at specified position and velocity
void spawnBoid(SimState& s, BoidType type, const Vec3& pos, const Vec3& vel);

// Spawn multiple boids of given type at random positions
void removeBoids(
    Boids& b,
    BoidType type,
    int& count,
    int howMany
);

// Delete all boids of given type within radius of (x,y) 
int deleteBoidsInRadius(
    SimState& s,
    BoidType type,
    int& count,
    float x,
    float y,
    float radius
);

