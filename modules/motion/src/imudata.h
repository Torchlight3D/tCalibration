﻿#pragma once

#include <Eigen/Core>

#include "imudatainterval.h"

namespace tl {

template <typename T>
class ImuSample_
{
public:
    explicit ImuSample_(const std::array<T, 3>& data = {T(0)}) : _xyz(data) {}

    inline const auto* data() const { return _xyz.data(); }

    inline const auto& x() const { return _xyz[0]; }
    inline const auto& y() const { return _xyz[1]; }
    inline const auto& z() const { return _xyz[2]; }

private:
    std::array<T, 3> _xyz;
};

using ImuSample = ImuSample_<double>;

using Timestamp = double;
using Timeline = std::vector<Timestamp>;

template <typename T>
class ImuReading_
{
public:
    ImuReading_() = default;
    ImuReading_(double timestamp, const Eigen::Vector3<T>& vec)
        : ImuReading_(timestamp, vec.x(), vec.y(), vec.z())
    {
    }
    ImuReading_(double timestamp, const std::array<T, 3>& data)
        : _data(data), _stamp(timestamp)
    {
    }
    ImuReading_(double timestamp, T x, T y, T z)
        : ImuReading_(timestamp, std::array{x, y, z})
    {
    }

    inline ImuReading_<T>& operator+=(const Eigen::Vector3<T>& vec)
    {
        Eigen::Map<Eigen::Vector3<T>>{_data.data()} += vec;
        return *this;
    }
    inline ImuReading_<T>& operator-=(const Eigen::Vector3<T>& vec)
    {
        return *this + (-vec);
    }

    inline const auto& x() const { return _data.x(); }
    inline const auto& y() const { return _data.y(); }
    inline const auto& z() const { return _data.z(); }
    inline const auto& operator[](int index) const { return _data[index]; }

    inline const auto* data() const { return _data.data(); }
    inline Eigen::Vector3<T> asVector() const
    {
        return Eigen::Map<const Eigen::Vector3<T>>{_data.data()};
    }

    inline const double& timestamp() const { return _stamp; }

private:
    ImuSample_<T> _data;
    double _stamp{0.}; // in sec
};

using ImuReading = ImuReading_<double>;
using AccReading = ImuReading;
using GyroReading = ImuReading;

struct ImuData
{
    AccReading acc;
    GyroReading gyro;
    double temperature{0.};
};

using ImuDataList = std::vector<ImuData>;

template <typename T>
class ImuReadings_
{
public:
    using ImuReading_t = ImuReading_<T>;

    ImuReadings_() = default;

    /// Helpers
    DataInterval validInterval(const DataInterval& interval) const;

    /// Element access
    const std::vector<ImuReading_t>& d() const { return _samples; }
    std::vector<ImuReading_t>& d() { return _samples; }

    const ImuReading_t& operator[](size_t i) const { return _samples[i]; }
    ImuReading_t& operator[](size_t i) { return _samples[i]; }

    const ImuReading_t& front() const { return _samples.front(); }
    const ImuReading_t& back() const { return _samples.back(); }

    /// Capacity
    size_t size() const { return _samples.size(); }
    void reserve(size_t size) { _samples.reserve(size); }

    /// Modifiers
    void clear() { _samples.clear(); }

    /// Statistics
    Eigen::Vector3d mean(const DataInterval& interval = {}) const;
    Eigen::Vector3d variance(const DataInterval& interval = {}) const;

    /// Lookup
    size_t findTimestamp(double timestamp) const;

    ImuReadings_<T> extractSamples(
        const std::vector<DataInterval>& intervals,
        std::vector<DataInterval>& extractedIntervals,
        int minIntervalSize = 100, bool onlyMeans = false) const;

    std::vector<DataInterval> extractStaticIntervals(
        double threshold, int windowSize = 101) const;

    Eigen::MatrixX3<T> toMatrix() const;

    Timeline timeline() const;

private:
    std::vector<ImuReading_t> _samples;
};

using ImuReadings = ImuReadings_<double>;

using AccDatas = ImuReadings;
using GyroDatas = ImuReadings;

struct ImuDatas
{
    AccDatas acc;
    GyroDatas gyro;
    Timeline timeline;

    // TODO:
    // 1. Add iterator to support forloop
    // 2. assert/check vector sizes are consistent
    void reserve(size_t size)
    {
        acc.reserve(size);
        gyro.reserve(size);
        timeline.reserve(size);
    }

    size_t size() const { return timeline.size(); }

    void push_back(const ImuData& data)
    {
        // Assume acc & gyro timestamp are consistent
        acc.d().push_back(data.acc);
        gyro.d().push_back(data.gyro);
        timeline.push_back(data.acc.timestamp());
    }

    ImuData at(size_t i) const
    {
        assert(i < size());
        return {.acc = acc.d()[i], .gyro = gyro.d()[i]};
    }

    void clear()
    {
        acc.clear();
        gyro.clear();
        timeline.clear();
    }
};

// TODO: Put this closer to SplineErrorWeighting
struct SplineWeightingData
{
    double r3_dt, r3_var;
    double so3_dt, so3_var;
};

inline double calcSampleRate(const Timeline& timeline)
{
    return (timeline.size() - 1.) / (timeline.back() - timeline.front());
}

} // namespace tl

#include "imudata.hpp"
