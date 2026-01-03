#include <algorithm>
#include <random>
#include <cassert>

#include "simulator/BoidPopulationRegulation.hpp"

#include "core/Types.hpp"
#include "core/SimState.hpp"
#include "simulator/SimulatorHelpers.hpp"


void regulateBoidPopulation(SimState& s) {
    simulator::regulateType(
        s,
        BoidType::Basic,
        s.boids.basicBoidCount,
        static_cast<int>(s.basicBoidCountTarget.number())
    );

    simulator::regulateType(
        s,
        BoidType::Predator,
        s.boids.predatorBoidCount,
        static_cast<int>(s.predatorBoidCountTarget.number())
    );

    if (s.deleteObstacles.binary()) {
        simulator::removeBoids(
            s.boids,
            BoidType::Obstacle,
            s.boids.obstacleBoidCount,
            s.boids.obstacleBoidCount
        );
    }
}
