/**
 * \file BoidPopulationRegulationUtils.hpp
 * \author Jan Koča
 * \date 05-01-2026
 * \brief Utilities for spawning, deleting, and regulating boid populations.
 *
 * Provides helpers used by simulation and interaction systems to:
 *  - spawn boids at random or specified positions
 *  - remove boids using slot-swap deletion
 *  - delete boids within a world-space radius
 *  - regulate population toward a target value each step
 */

#pragma once

#include <vector>
#include <cstdint>

#include "core/Boids.hpp"
#include "core/SimState.hpp"

/**
 * \brief Gradually adjust boid population toward a target count.
 *
 * Applies clamping via `maxBoidPopulationChangeRate` to avoid large jumps.
 *
 * \param[in,out] s Simulation state.
 * \param[in] type Type of boid to regulate.
 * \param[in,out] count Current population counter for the type.
 * \param[in] target Desired population count.
 */
void regulateBoidPopulation(
    SimState& s,
    BoidType type,
    int& count,
    int target
);

/**
 * \brief Spawn multiple boids of a given type.
 *
 * When coordinates are negative, positions are sampled uniformly at random
 * inside the world bounds. Velocity is sampled from
 * `initialAxialSpeedRange`. Z components are zeroed when in 2D mode.
 *
 * \param[in,out] s Simulation state.
 * \param[in] type Boid type to spawn.
 * \param[in,out] count Population counter to increment.
 * \param[in] howMany Number of boids to spawn.
 * \param[in] x Optional fixed X coordinate.
 * \param[in] y Optional fixed Y coordinate.
 * \param[in] z Optional fixed Z coordinate.
 */
void spawnBoids(
    SimState& s,
    BoidType type,
    int& count,
    int howMany,
    float x = -1,
    float y = -1,
    float z = -1
);

/**
 * \brief Spawn a single boid at a specified position and velocity.
 *
 * Does not modify population counters (caller must adjust them).
 */
void spawnBoid(SimState& s, BoidType type, const Vec3& pos, const Vec3& vel);

/**
 * \brief Remove boids from the end of the SoA storage.
 *
 * Uses last-element pop semantics; does not preserve ordering.
 */
void removeBoids(
    Boids& b,
    BoidType type,
    int& count,
    int howMany
);

/**
 * \brief Delete all boids of given type within a world-space radius.
 *
 * Implements slot-swap deletion to keep arrays compact.
 *
 * \return Number of boids removed.
 */
int deleteBoidsInRadius(
    SimState& s,
    BoidType type,
    int& count,
    float x,
    float y,
    float radius
);
