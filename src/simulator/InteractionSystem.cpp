#include <cmath>
#include <vector>
#include <chrono>
#include <string>
#include <cassert>
#include <algorithm>

#include "simulator/InteractionSystem.hpp"

#include "gui/GUI.hpp"
#include "core/Types.hpp"
#include "core/SimState.hpp"
#include "simulator/SimulatorHelpers.hpp"


static void applyEffect(SimState& s, const std::string& effect, float x, float y) {
    // Get the boids
    Boids& b = s.boids;

    // Dispatch effect
    if (effect == "Spawn Predator") {
        const auto& targetRange = s.predatorBoidCountTarget.numberRange();
        if (s.predatorBoidCountTarget.number() < targetRange.max) {
            simulator::spawnBoids(s, BoidType::Predator, b.predatorBoidCount, 1, x, y);
            s.predatorBoidCountTarget.number() += 1.0f;
        }
    } else if (effect == "Delete Predator") {
        int removed = simulator::deleteAllBoidsOfTypeInRadius(
            s,
            BoidType::Predator,
            b.predatorBoidCount,
            x, y,
            s.predatorBoidRadius.number() * 2.0f
        );
        s.predatorBoidCountTarget.number() = std::max(0.0f, s.predatorBoidCountTarget.number() - float(removed));
    } else if (effect == "Draw Obstacle") {
        simulator::spawnBoids(s, BoidType::Obstacle, b.obstacleBoidCount, 1, x, y);
    } else if (effect == "Erase Obstacle") {
        simulator::deleteAllBoidsOfTypeInRadius(
            s,
            BoidType::Obstacle,
            b.obstacleBoidCount,
            x, y,
            s.obstacleBoidRadius.number() * 2.0f
        );
    } else if (effect == "Attract" || effect == "Repel") {
        s.interaction.type = (effect == "Attract") ? InteractionType::Attract : InteractionType::Repel;
    }
}

// Returns true if timestamp is within window from now
inline bool recent(const std::chrono::high_resolution_clock::time_point& timestamp1, 
                   const std::chrono::high_resolution_clock::time_point& timestamp2,
                std::chrono::milliseconds window = std::chrono::milliseconds(100)) {
    return (timestamp2 - timestamp1) < window;
}

// Returns true if points are within relative tolerance of world size
inline bool nearSamePoint(float ax, float ay, float bx, float by, float worldX, float worldY, float relTol = 0.01f) {
    const float tolX = worldX * relTol;
    const float tolY = worldY * relTol;

    return std::abs(ax - bx) < tolX && std::abs(ay - by) < tolY;
}

void applyInteraction(SimState& simState, MouseInteractionEvent& interaction) {
    // Create interaction placeholder
    Vec3 interactionPosition = { interaction.worldX, interaction.worldY, 0.0f };
    InteractionType interactionType = InteractionType::Empty;
    simState.interaction.point = interactionPosition;
    simState.interaction.type = interactionType;

    // Check whether the interaction is active
    if (!interaction.active) {
        return;
    }

    // Determine effect based on interaction type        
    const std::string& effect = (interaction.type == MouseInteractionType::LeftClickOnWorld) ? simState.leftMouseEffect.string() : simState.rightMouseEffect.string();

    // For the spawning effects, avoid multiple spawns at the same location
    if (effect == "Spawn Predator" || effect == "Draw Obstacle") {
        if (recent(interaction.lastTimestamp, interaction.timestamp)) {
            if (nearSamePoint(interaction.worldX, interaction.worldY, interaction.lastWorldX, interaction.lastWorldY, 
                            simState.worldX.number(), simState.worldY.number(), 0.01f)) {
                return;
            }
        }
        interaction.lastWorldX = interaction.worldX;
        interaction.lastWorldY = interaction.worldY;
        interaction.lastTimestamp = interaction.timestamp;
    }

    applyEffect(simState, effect, interaction.worldX, interaction.worldY);
}
