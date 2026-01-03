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

struct Boids {
    // Core SoA fields
    // - Basic boids
    std::vector<Vec3> posBasic;
    std::vector<Vec3> velBasic;
    std::vector<Vec3> accBasic;
    std::vector<Vec3> targetPointBasic;

    // - Predator boids
    std::vector<Vec3> posPredator;
    std::vector<Vec3> velPredator;
    std::vector<Vec3> accPredator;
    std::vector<Vec3> targetPointPredator;

    std::vector<float> staminaPredator;
    std::vector<uint8_t> restingPredator;

    std::vector<int> targetBoidIdxPredator;
    std::vector<float> targetBoidDistancePredator;
    std::vector<BoidType> targetBoidTypePredator;

    // - Obstacle boids
    std::vector<Vec3> posObstacle;

    // Bookkeeping by boid type
    int basicBoidCount = 0;
    int predatorBoidCount = 0;
    int obstacleBoidCount = 0;

    // Lifecycle helpers
    void resizeBasic(int n) {
        basicBoidCount = n;

        posBasic.resize(n);
        velBasic.resize(n);
        accBasic.resize(n);
        targetPointBasic.resize(n);
    }

    void resizePredator(int n) {
        predatorBoidCount = n;

        posPredator.resize(n);
        velPredator.resize(n);
        accPredator.resize(n);
        targetPointPredator.resize(n);

        staminaPredator.resize(n);
        restingPredator.resize(n);

        targetBoidIdxPredator.resize(n);
        targetBoidDistancePredator.resize(n);
        targetBoidTypePredator.resize(n);
    }

    void resizeObstacle(int n) {
        obstacleBoidCount = n;

        posObstacle.resize(n);
    }

    void clear() {
        posBasic.clear();
        velBasic.clear();
        accBasic.clear();
        targetPointBasic.clear();

        posPredator.clear();
        velPredator.clear();
        accPredator.clear();
        targetPointPredator.clear();    
        staminaPredator.clear();
        restingPredator.clear();
        targetBoidIdxPredator.clear();
        targetBoidDistancePredator.clear();
        targetBoidTypePredator.clear();

        posObstacle.clear();

        basicBoidCount = 0;
        predatorBoidCount = 0;
        obstacleBoidCount = 0;
    }
};

// GPU-side mirror of Boids struct
struct DeviceBoids {
    // Core SoA fields
    // - Basic boids
    Vec3* posBasic = nullptr;
    Vec3* velBasic = nullptr;
    Vec3* accBasic = nullptr;
    Vec3* targetPointBasic = nullptr;

    // - Predator boids
    Vec3* posPredator = nullptr;
    Vec3* velPredator = nullptr;
    Vec3* accPredator = nullptr;
    Vec3* targetPointPredator = nullptr;

    float* staminaPredator = nullptr;
    uint8_t* restingPredator = nullptr;

    int* targetBoidIdxPredator = nullptr;
    float* targetBoidDistancePredator = nullptr;
    uint8_t* targetBoidTypePredator = nullptr;

    // - Obstacle boids
    Vec3* posObstacle = nullptr;

    // Bookkeeping by boid type
    int basicBoidCount = 0;
    int predatorBoidCount = 0;
    int obstacleBoidCount = 0;
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
