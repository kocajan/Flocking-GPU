#include <algorithm>
#include <random>
#include <cassert>

#include "simulator/BoidPopulationRegulation.hpp"

#include "core/Types.hpp"
#include "core/SimState.hpp"
#include "simulator/SimulatorHelpers.hpp"


void regulateBoidPopulation(SimState& s) {
    regulateBoidPopulation(
        s,
        BoidType::Basic,
        s.boids.basicBoidCount,
        static_cast<int>(s.basicBoidCountTarget.number())
    );

    regulateBoidPopulation(
        s,
        BoidType::Predator,
        s.boids.predatorBoidCount,
        static_cast<int>(s.predatorBoidCountTarget.number())
    );

    if (s.deleteObstacles.binary()) {
        removeBoids(
            s.boids,
            BoidType::Obstacle,
            s.boids.obstacleBoidCount,
            s.boids.obstacleBoidCount
        );
    }
}
