#pragma once
#include <cmath>

inline float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

inline float length2(float x, float y) {
    return x * x + y * y;
}

inline float length(float x, float y) {
    return std::sqrt(length2(x, y));
}
