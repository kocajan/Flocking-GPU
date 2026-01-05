/**
 * \file BoidPopulationRegulation.cpp
 * \author Jan Koča
 * \date 01-05-2026
 * \brief Implementation of high-level population regulation routine.
 */

#include <algorithm>
#include <random>
#include <cassert>

#include "simulator/BoidPopulationRegulation.hpp"

#include "core/Types.hpp"
#include "core/SimState.hpp"
#include "simulator/BoidPopulationRegulationUtils.hpp"

void regulateBoidPopulation(SimState& s) {
    // Regulate basic boids
    regulateBoidPopulation(
        s,
        BoidType::Basic,
        s.boids.basicBoidCount,
        static_cast<int>(s.basicBoidCountTarget.number())
    );

    // Regulate predator boids
    regulateBoidPopulation(
        s,
        BoidType::Predator,
        s.boids.predatorBoidCount,
        static_cast<int>(s.predatorBoidCountTarget.number())
    );

    // Optionally clear all obstacles
    if (s.deleteObstacles.binary()) {
        removeBoids(
            s.boids,
            BoidType::Obstacle,
            s.boids.obstacleBoidCount,
            s.boids.obstacleBoidCount
        );
    }
}
