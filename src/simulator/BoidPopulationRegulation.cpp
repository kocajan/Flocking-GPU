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
        s.basicBoidIndices,
        s.basicBoidCount,
        s.basicBoidCountTarget,
        s.basicBoidRadius,
        s.basicBoidColor
    );

    simulator::regulateType(
        s,
        BoidType::Predator,
        s.predatorBoidIndices,
        s.predatorBoidCount,
        s.predatorBoidCountTarget,
        s.predatorRadius,
        s.predatorBoidColor
    );

    if (s.deleteObstacles.binary()) {
        simulator::removeBoids(
            s.boids,
            s.freeBoidIndices,
            s.obstacleBoidIndices,
            s.obstacleBoidCount,
            static_cast<int>(s.obstacleBoidCount)
        );
    }
}
