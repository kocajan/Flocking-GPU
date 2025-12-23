#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cassert>

#include "core/SimState.hpp"
#include "gui/GUI.hpp"
#include "boids/Boid.hpp"

// ============================================================
// Tunables
// ============================================================

static constexpr float INTERACTION_RADIUS = 6.0f;
static constexpr float INTERACTION_RADIUS2 =
    INTERACTION_RADIUS * INTERACTION_RADIUS;

// ============================================================
// Small helpers
// ============================================================

static inline float dist2(float x1, float y1, float x2, float y2) {
    const float dx = x1 - x2;
    const float dy = y1 - y2;
    return dx * dx + dy * dy;
}

// ============================================================
// Allocation helpers (same model as population regulation)
// ============================================================

static size_t allocateBoidSlot(SimState& s) {
    if (!s.freeBoidIndices.empty()) {
        size_t idx = s.freeBoidIndices.back();
        s.freeBoidIndices.pop_back();
        return idx;
    }
    s.boids.emplace_back();
    return s.boids.size() - 1;
}

static void freeBoidSlot(SimState& s, size_t idx) {
#ifndef NDEBUG
    assert(idx < s.boids.size());
#endif
    s.boids[idx].type = BoidType::Empty;
    s.freeBoidIndices.push_back(idx);
}

// ============================================================
// Color parsing
// ============================================================

static Color parseColorString(const std::string& s) {
    if (s == "White")  return {1,1,1,1};
    if (s == "Black")  return {0,0,0,1};
    if (s == "Red")    return {1,0,0,1};
    if (s == "Green")  return {0,1,0,1};
    if (s == "Blue")   return {0,0,1,1};
    return {0.5f,0.5f,0.5f,1};
}

// ============================================================
// Max-limit helpers
// ============================================================

static bool canIncreaseTarget(const ConfigParameter& p) {
    const auto& r = std::get<NumberRange>(p.range);
    return p.number() < r.max;
}

// ============================================================
// Spawn helpers
// ============================================================

static void spawnPredator(SimState& s, float x, float y) {
    if (!canIncreaseTarget(s.predatorBoidCountTarget))
        return;

    size_t idx = allocateBoidSlot(s);
    Boid& b = s.boids[idx];

    b.type   = BoidType::Predator;
    b.pos    = { x, y, 0.0f };
    b.vel    = {};
    b.acc    = {};
    b.radius = s.predatorRadius.number();
    b.color  = parseColorString(s.predatorBoidColor.string());

    s.predatorBoidIndices.push_back(idx);
    ++s.predatorBoidCount;
    s.predatorBoidCountTarget.number() += 1.0f;
}

static void spawnObstacle(SimState& s, float x, float y) {
    size_t idx = allocateBoidSlot(s);
    Boid& b = s.boids[idx];

    b.type   = BoidType::Obstacle;
    b.pos    = { x, y, 0.0f };
    b.vel    = {};
    b.acc    = {};
    b.radius = s.obstacleRadius.number();
    b.color  = parseColorString(s.obstacleBoidColor.string());

    s.obstacleBoidIndices.push_back(idx);
    ++s.obstacleBoidCount;
}

// ============================================================
// Deletion helpers (DELETE ALL IN RADIUS)
// ============================================================

static int deleteAllInRadius(
    SimState& s,
    std::vector<size_t>& indices,
    uint64_t& count,
    float x,
    float y
) {
    int removed = 0;

    for (size_t i = 0; i < indices.size(); ) {
        const size_t idx = indices[i];
        const Boid& b = s.boids[idx];

        if (dist2(b.pos.x, b.pos.y, x, y) <= INTERACTION_RADIUS2) {
            freeBoidSlot(s, idx);
            indices[i] = indices.back();
            indices.pop_back();
            --count;
            ++removed;
        } else {
            ++i;
        }
    }

    return removed;
}

// ============================================================
// Effect dispatcher
// ============================================================

static void applyEffect(SimState& s, const std::string& effect, float x, float y) {
    // Dispatch effect
    if (effect == "Spawn Predator") {
        spawnPredator(s, x, y);
    } else if (effect == "Delete Predator") {
        int removed = deleteAllInRadius(
            s,
            s.predatorBoidIndices,
            s.predatorBoidCount,
            x, y
        );

        s.predatorBoidCountTarget.number() =
            std::max(0.f,
                s.predatorBoidCountTarget.number() - float(removed));
    } else if (effect == "Draw Obstacle") {
        spawnObstacle(s, x, y);
    } else if (effect == "Erase Obstacle") {
        deleteAllInRadius(
            s,
            s.obstacleBoidIndices,
            s.obstacleBoidCount,
            x, y
        );
    } else if (effect == "Attract" || effect == "Repel") {
        s.interaction.type = (effect == "Attract") ? InteractionType::Attract : InteractionType::Repel;
    }
}

// ============================================================
// Public entry point
// ============================================================
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
