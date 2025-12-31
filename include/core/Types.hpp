#pragma once

#include <cstdint>
#include <vector>
#include <variant>
#include <string>
#include <cassert>

//
// ============================================================================
//  Basic math + color types
// ============================================================================
//

struct Vec3 {
    float x;
    float y;
    float z;
};

struct Color {
    int r;
    int g;
    int b;
    int a;
};


//
// ============================================================================
//  Interaction types (runtime mouse / environment actions)
// ============================================================================
//

enum class InteractionType : uint8_t {
    Repel,
    Attract,
    Empty
};

struct Interaction {
    InteractionType type{InteractionType::Empty};
    Vec3 point{0.0f, 0.0f, 0.0f};
};

struct DeviceInteraction {
    uint8_t type;
    Vec3 point;
};


//
// ============================================================================
//  Boid classification
// ============================================================================
//

enum class BoidType : uint8_t {
    Basic,
    Predator,
    Obstacle,
    Empty
};


//
// ============================================================================
//  Boid population container (Structure of Arrays)
// ============================================================================
//
//  Notes:
//   - Each field is stored in its own contiguous array for cache efficiency.
//   - This layout is designed for parallel CPU and CUDA execution.
//   - Index lists allow fast per-type iteration.
//   - Free-list supports reuse without immediate compaction.
//
struct Boids {

    //
    // Core SoA fields
    //

    std::vector<Vec3> pos;          // position
    std::vector<Vec3> vel;          // velocity
    std::vector<Vec3> acc;          // accumulated acceleration
    std::vector<Vec3> targetPoint;  // local target

    std::vector<uint8_t> type;      // BoidType (stored compactly)
    std::vector<float> stamina;
    std::vector<uint8_t> resting;

    std::vector<int>   targetBoidIdx;
    std::vector<float> targetBoidDistance;

    size_t count = 0;               // total occupied entries


    //
    // Bookkeeping by boid type
    //

    uint64_t basicBoidCount    = 0;
    uint64_t predatorBoidCount = 0;
    uint64_t obstacleBoidCount = 0;

    std::vector<size_t> basicBoidIndices;
    std::vector<size_t> predatorBoidIndices;
    std::vector<size_t> obstacleBoidIndices;


    //
    // Free-list for index reuse / deferred compaction
    //

    std::vector<size_t> freeBoidIndices;

    //==========================================================================
    // Lifecycle helpers
    //==========================================================================

    void resize(size_t n) {
        count = n;

        pos.resize(n);
        vel.resize(n);
        acc.resize(n);
        targetPoint.resize(n);

        type.resize(n);
        stamina.resize(n);
        resting.resize(n);

        targetBoidIdx.resize(n);
        targetBoidDistance.resize(n);
    }

    void clear() {
        count = 0;

        pos.clear();
        vel.clear();
        acc.clear();
        targetPoint.clear();

        type.clear();
        stamina.clear();
        resting.clear();

        targetBoidIdx.clear();
        targetBoidDistance.clear();

        basicBoidCount    = 0;
        predatorBoidCount = 0;
        obstacleBoidCount = 0;

        basicBoidIndices.clear();
        predatorBoidIndices.clear();
        obstacleBoidIndices.clear();

        freeBoidIndices.clear();
    }

    void assertConsistent() const {
        const size_t n = count;

        assert(pos.size() == n);
        assert(vel.size() == n);
        assert(acc.size() == n);
        assert(targetPoint.size() == n);

        assert(type.size() == n);
        assert(stamina.size() == n);
        assert(resting.size() == n);

        assert(targetBoidIdx.size() == n);
        assert(targetBoidDistance.size() == n);
    }
};

// GPU-side mirror of Boids struct
struct DeviceBoids {
    // Core SoA fields
    Vec3*    pos      = nullptr;
    Vec3*    vel      = nullptr;
    Vec3*    acc      = nullptr;
    Vec3*    targetPoint = nullptr;

    uint8_t* type     = nullptr;
    float*   stamina  = nullptr;
    uint8_t* resting  = nullptr;

    int*     targetBoidIdx       = nullptr;
    float*   targetBoidDistance  = nullptr;

    size_t   count   = 0;

    // Type index lists
    size_t*  basicBoidIndices    = nullptr;
    size_t*  predatorBoidIndices = nullptr;
    size_t*  obstacleBoidIndices = nullptr;

    uint64_t basicBoidCount    = 0;
    uint64_t predatorBoidCount = 0;
    uint64_t obstacleBoidCount = 0;
};

//
// ============================================================================
//  Config parameter types (UI + configuration system)
// ============================================================================
//

enum class ParamType {
    Number,
    Binary,
    String,
    Enum
};

//
// Preferred rendering style in GUI
//
enum class ParamRender {
    Checkbox,
    Button,
    ToggleButton,
    Slider,
    Drag,
    Input
};


//
// ============================================================================
//  Parameter ranges
// ============================================================================
//

// Number parameter limits
struct NumberRange {
    float min  = 0.0f;
    float max  = 1.0f;
    float step = 0.01f;
};

// Boolean parameter flags
struct BinaryRange {
    bool allowTrue  = true;
    bool allowFalse = true;
};

// String parameter constraints
struct StringRange {
    bool freeText = true;                 // allow arbitrary input
    std::vector<std::string> options;     // optional allowed values
};

// Enum parameter options
struct EnumRange {
    std::vector<std::string> options;
};


//
// ============================================================================
//  Unified parameter storage
// ============================================================================
//

using ParamValue = std::variant<
    bool,
    int,
    float,
    std::string
>;

using ParamRange = std::variant<
    NumberRange,
    BinaryRange,
    StringRange,
    EnumRange
>;
