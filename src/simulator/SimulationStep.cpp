#include "core/SimState.hpp"
#include "config/SimConfig.hpp"
#include "boids/Boid.hpp"

#include <cmath>
#include <algorithm>

// ------------------------------------------------------------
// Small helpers (local to this file)
// ------------------------------------------------------------
static float sqrLen(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

static Vec3 normalize(const Vec3& v) {
    const float l2 = sqrLen(v);
    if (l2 <= 0.0f) return {0, 0, 0};
    const float inv = 1.0f / std::sqrt(l2);
    return {v.x * inv, v.y * inv, v.z * inv};
}

static Vec3 setMagnitude(const Vec3& v, float mag) {
    const float l2 = sqrLen(v);
    if (l2 <= 0.0f) return {0, 0, 0};
    const float inv = mag / std::sqrt(l2);
    return {v.x * inv, v.y * inv, v.z * inv};
}

static Vec3 limitMagnitude(const Vec3& v, float maxMag) {
    const float l2 = sqrLen(v);
    const float max2 = maxMag * maxMag;
    if (l2 <= max2) return v;
    const float inv = maxMag / std::sqrt(l2);
    return {v.x * inv, v.y * inv, v.z * inv};
}

// ------------------------------------------------------------
// simulationStep
// ------------------------------------------------------------
void simulationStep(
    SimState& simState,
    const SimConfig& simConfig
) {
    const float dt = simState.dt.number();

    const float worldX = simState.worldX.number();
    const float worldY = simState.worldY.number();
    const float worldZ = simState.worldZ.number();

    const bool is2D = (simState.dimensions.string() == "2D");

    // --------------------------------------------------------
    // Read config (version-safe)
    // --------------------------------------------------------
    const float vision =
        simConfig.has("vision") ? simConfig.number("vision") : 0.0f;
    const float vision2 = vision * vision;

    const float alignmentW =
        simConfig.has("alignment") ? simConfig.number("alignment") : 0.0f;
    const float cohesionW =
        simConfig.has("cohesion") ? simConfig.number("cohesion") : 0.0f;
    const float separationW =
        simConfig.has("separation") ? simConfig.number("separation") : 0.0f;

    const float bias =
        simConfig.has("bias") ? simConfig.number("bias") : 1.0f;

    const float maxSpeed =
        simConfig.has("max_speed") ? simConfig.number("max_speed") : 0.0f;
    const float minSpeed =
        simConfig.has("min_speed") ? simConfig.number("min_speed") : 0.0f;
    const float maxForce =
        simConfig.has("max_force") ? simConfig.number("max_force") : 0.0f;

    const float drag =
        simConfig.has("drag") ? simConfig.number("drag") : 0.0f;

    const float noise =
        simConfig.has("noise") ? simConfig.number("noise") : 0.0f;

    const bool bounce =
        simConfig.has("bounce") ? simConfig.binary("bounce") : true;

    const float noiseRange =
        (noise > 0.0f)
            ? static_cast<float>(M_PI / 80.0) * noise
            : 0.0f;

    // --------------------------------------------------------
    // 1. Integrate velocity (using previous acceleration)
    // --------------------------------------------------------
    for (Boid& b : simState.boids) {
        if (b.type == BoidType::Obstacle)
            continue;

        b.vel.x += b.acc.x * dt;
        b.vel.y += b.acc.y * dt;
        b.vel.z += b.acc.z * dt;

        // drag
        if (drag > 0.0f) {
            b.vel.x *= (1.0f - drag);
            b.vel.y *= (1.0f - drag);
            b.vel.z *= (1.0f - drag);
        }

        // noise (XY rotation)
        if (noiseRange > 0.0f) {
            const float a =
                ((float)rand() / RAND_MAX * 2.0f - 1.0f) * noiseRange;

            const float cs = std::cos(a);
            const float sn = std::sin(a);

            const float vx = b.vel.x * cs - b.vel.y * sn;
            const float vy = b.vel.x * sn + b.vel.y * cs;

            b.vel.x = vx;
            b.vel.y = vy;
        }

        // speed clamp
        float v2 = sqrLen(b.vel);

        if (v2 > 0.0f && maxSpeed > 0.0f) {
            const float max2 = maxSpeed * maxSpeed;
            if (v2 > max2) {
                const float inv = maxSpeed / std::sqrt(v2);
                b.vel.x *= inv;
                b.vel.y *= inv;
                b.vel.z *= inv;
                v2 = max2;
            }
        }

        if (v2 > 0.0f && minSpeed > 0.0f) {
            const float min2 = minSpeed * minSpeed;
            if (v2 < min2) {
                const float inv = minSpeed / std::sqrt(v2);
                b.vel.x *= inv;
                b.vel.y *= inv;
                b.vel.z *= inv;
            }
        }
    }

    // --------------------------------------------------------
    // 2. Integrate position + bounds
    // --------------------------------------------------------
    for (Boid& b : simState.boids) {
        if (b.type == BoidType::Obstacle)
            continue;

        b.pos.x += b.vel.x * dt;
        b.pos.y += b.vel.y * dt;
        b.pos.z += b.vel.z * dt;

        if (bounce) {
            if (b.pos.x < 0.0f || b.pos.x > worldX) {
                b.vel.x *= -1.0f;
                b.pos.x = std::clamp(b.pos.x, 0.0f, worldX);
            }
            if (b.pos.y < 0.0f || b.pos.y > worldY) {
                b.vel.y *= -1.0f;
                b.pos.y = std::clamp(b.pos.y, 0.0f, worldY);
            }
            if (!is2D) {
                if (b.pos.z < 0.0f || b.pos.z > worldZ) {
                    b.vel.z *= -1.0f;
                    b.pos.z = std::clamp(b.pos.z, 0.0f, worldZ);
                }
            } else {
                b.pos.z = 0.0f;
                b.vel.z = 0.0f;
            }
        } else {
            if (b.pos.x < 0.0f) b.pos.x += worldX;
            if (b.pos.x > worldX) b.pos.x -= worldX;
            if (b.pos.y < 0.0f) b.pos.y += worldY;
            if (b.pos.y > worldY) b.pos.y -= worldY;

            if (!is2D) {
                if (b.pos.z < 0.0f) b.pos.z += worldZ;
                if (b.pos.z > worldZ) b.pos.z -= worldZ;
            } else {
                b.pos.z = 0.0f;
                b.vel.z = 0.0f;
            }
        }
    }

    // --------------------------------------------------------
    // 3. Compute NEW accelerations (naive O(N²))
    // --------------------------------------------------------
    for (Boid& b : simState.boids) {
        if (b.type == BoidType::Obstacle) {
            b.acc = {0, 0, 0};
            continue;
        }

        Vec3 align{0, 0, 0};
        Vec3 coh{0, 0, 0};
        Vec3 sep{0, 0, 0};

        int count = 0;

        for (const Boid& o : simState.boids) {
            if (&b == &o || o.type == BoidType::Obstacle)
                continue;

            const Vec3 d{
                b.pos.x - o.pos.x,
                b.pos.y - o.pos.y,
                b.pos.z - o.pos.z
            };

            const float d2 = sqrLen(d);
            if (d2 <= 0.0f || (vision > 0.0f && d2 > vision2))
                continue;

            // alignment (biased)
            const float dot =
                normalize(b.vel).x * normalize(o.vel).x +
                normalize(b.vel).y * normalize(o.vel).y +
                normalize(b.vel).z * normalize(o.vel).z;

            const float w = std::pow(bias, dot);
            align.x += o.vel.x * w;
            align.y += o.vel.y * w;
            align.z += o.vel.z * w;

            // cohesion
            coh.x += o.pos.x;
            coh.y += o.pos.y;
            coh.z += o.pos.z;

            // separation (inverse distance)
            const float inv = 1.0f / (d2 + 1e-5f);
            sep.x += d.x * inv;
            sep.y += d.y * inv;
            sep.z += d.z * inv;

            ++count;
        }

        b.acc = {0, 0, 0};

        if (count > 0) {
            // alignment
            Vec3 a = setMagnitude(align, maxSpeed);
            a.x -= b.vel.x;
            a.y -= b.vel.y;
            a.z -= b.vel.z;
            a = limitMagnitude(a, maxForce);

            // cohesion
            coh.x = coh.x / count - b.pos.x;
            coh.y = coh.y / count - b.pos.y;
            coh.z = coh.z / count - b.pos.z;
            Vec3 c = setMagnitude(coh, maxSpeed);
            c.x -= b.vel.x;
            c.y -= b.vel.y;
            c.z -= b.vel.z;
            c = limitMagnitude(c, maxForce);

            // separation
            Vec3 s = setMagnitude(sep, maxSpeed);
            s.x -= b.vel.x;
            s.y -= b.vel.y;
            s.z -= b.vel.z;
            s = limitMagnitude(s, maxForce);

            b.acc.x += a.x * alignmentW;
            b.acc.y += a.y * alignmentW;
            b.acc.z += a.z * alignmentW;

            b.acc.x += c.x * cohesionW;
            b.acc.y += c.y * cohesionW;
            b.acc.z += c.z * cohesionW;

            b.acc.x += s.x * separationW;
            b.acc.y += s.y * separationW;
            b.acc.z += s.z * separationW;
        }
    }
}
