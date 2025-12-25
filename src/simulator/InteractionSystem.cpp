#include <cmath>
#include <vector>
#include <string>
#include <cassert>
#include <algorithm>

#include "simulator/InteractionSystem.hpp"

#include "gui/GUI.hpp"
#include "core/Types.hpp"
#include "core/SimState.hpp"
#include "simulator/SimulatorHelpers.hpp"


static void applyEffect(SimState& s, const std::string& effect, float x, float y) {
    // Dispatch effect
    if (effect == "Spawn Predator") {
        simulator::spawnPredator(s, x, y);
    } else if (effect == "Delete Predator") {
        int removed = simulator::deleteAllInRadius(
            s,
            s.predatorBoidIndices,
            s.predatorBoidCount,
            x, y,
            s.predatorRadius.number()
        );

        s.predatorBoidCountTarget.number() =
            std::max(0.f,
                s.predatorBoidCountTarget.number() - float(removed));
    } else if (effect == "Draw Obstacle") {
        simulator::spawnObstacle(s, x, y);
    } else if (effect == "Erase Obstacle") {
        simulator::deleteAllInRadius(
            s,
            s.obstacleBoidIndices,
            s.obstacleBoidCount,
            x, y,
            s.obstacleRadius.number()
        );
    } else if (effect == "Attract" || effect == "Repel") {
        s.interaction.type = (effect == "Attract") ? InteractionType::Attract : InteractionType::Repel;
    }
}

void applyInteraction(SimState& simState, const MouseInteractionEvent& interaction) {
    // Create interaction placeholder
    Vec3 interactionPosition = { interaction.worldX, interaction.worldY, 0.0f };
    InteractionType interactionType = InteractionType::Empty;
    simState.interaction.point = interactionPosition;
    simState.interaction.type = interactionType;

    // Check whether the interaction is active
    if (!interaction.active) {
        return;
    }

    const std::string& effect = (interaction.type == MouseInteractionType::LeftClickOnWorld) ? simState.leftMouseEffect.string() : simState.rightMouseEffect.string();

    applyEffect(simState, effect, interaction.worldX, interaction.worldY);
}
