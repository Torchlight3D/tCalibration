#pragma once

#include <cstddef>

constexpr int SAMPLES_PER_PIXEL = 32;
constexpr size_t FFT_SIZE = int(16) * SAMPLES_PER_PIXEL;
constexpr int NYQUIST_FREQ = FFT_SIZE / 16;
