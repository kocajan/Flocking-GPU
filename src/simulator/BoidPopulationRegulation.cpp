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
        s.boids.basicBoidIndices,
        s.boids.basicBoidCount,
        s.basicBoidCountTarget
    );

    simulator::regulateType(
        s,
        BoidType::Predator,
        s.boids.predatorBoidIndices,
        s.boids.predatorBoidCount,
        s.predatorBoidCountTarget
    );

    if (s.deleteObstacles.binary())
    {
        simulator::removeBoids(
            s.boids,
            s.boids.obstacleBoidIndices,
            s.boids.obstacleBoidCount,
            (int)s.boids.obstacleBoidCount
        );
    }
}
