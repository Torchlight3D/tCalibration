#pragma once

#include <stdint.h>

// Underflow of exponential is common practice in numerical routines,
// so handle it here.

inline constexpr float fastpow2(float p)
{
    float offset = (p < 0) ? 1.0f : 0.0f;
    float clipp = (p < -126) ? -126.0f : p;
    int w = clipp;
    float z = clipp - w + offset;
    union
    {
        uint32_t i;
        float f;
    } v = {static_cast<uint32_t>((1 << 23) * (clipp + 121.2740575f +
                                              27.7280233f / (4.84252568f - z) -
                                              1.49012907f * z))};

    return v.f;
}

inline constexpr float fastexp(float p) { return fastpow2(1.442695040f * p); }
