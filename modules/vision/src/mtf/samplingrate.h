#pragma once

#include <cstddef>

inline constexpr int SAMPLES_PER_PIXEL = 32;
inline constexpr size_t FFT_SIZE = int(16) * SAMPLES_PER_PIXEL;
inline constexpr int NYQUIST_FREQ = FFT_SIZE / 16;
