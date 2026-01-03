#pragma once

#include <vector>
#include <cstdint>

#include "core/Types.hpp"
#include "core/SimState.hpp"


namespace simulator {

    //  Spawn or remove boids to regulate population towards target
    void regulateType(
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

    // Spawn multiple boids of given type at random positions
    void removeBoids(
        Boids& b,
        BoidType type,
        int& count,
        int howMany
    );

    // Delete all boids of given type within radius of (x,y) 
    int deleteAllBoidsOfTypeInRadius(
        SimState& s,
        BoidType type,
        int& count,
        float x,
        float y,
        float radius
    );
}
