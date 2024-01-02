#pragma once

#include "imu_data_interval.h"
#include "imu_data.h"

namespace thoht {

namespace {
constexpr size_t kMinSampleCount{3};
}

template <typename T>
DataInterval DataInterval::fromPeriod(const ImuReadings_<T>& samples,
                                      double start, double end)
{
    // TODO: take care of exception
    if (start < 0 || end <= start) {
        throw std::invalid_argument("Invalid timestamps");
    }
    if (samples.size() < kMinSampleCount) {
        throw std::invalid_argument("Invalid data samples vector");
    }

    const int startIdx =
        start <= samples.front().timestamp() ? 0 : samples.findTimestamp(start);
    const int endIdx = end >= samples.back().timestamp()
                           ? static_cast<int>(samples.size() - 1)
                           : samples.findTimestamp(end);

    return {startIdx, endIdx};
}

template <typename T>
DataInterval DataInterval::initialInterval(const ImuReadings_<T>& samples,
                                           double duration)
{
    // TODO: take care of exception
    if (duration <= 0) {
        throw std::invalid_argument("Invalid interval duration");
    }
    if (samples.size() < kMinSampleCount) {
        throw std::invalid_argument("Invalid data samples vector");
    }

    const double end = samples.front().timestamp() + duration;
    const int endIdx = end >= samples.back().timestamp()
                           ? static_cast<int>(samples.size() - 1)
                           : samples.findTimestamp(end);

    return {0, endIdx};
}

template <typename T>
DataInterval DataInterval::finalInterval(const ImuReadings_<T>& samples,
                                         double duration)
{
    // TODO: take care of exception
    if (duration <= 0) {
        throw std::invalid_argument("Invalid interval duration");
    }
    if (samples.size() < kMinSampleCount) {
        throw std::invalid_argument("Invalid data samples vector");
    }

    const double start = samples.back().timestamp() - duration;
    // FIXME: use isApprox0
    const int startIdx = start <= 0. ? 0 : samples.findTimestamp(start);

    return {startIdx, static_cast<int>(samples.size() - 1)};
}

} // namespace thoht
