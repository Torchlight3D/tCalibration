#pragma once

#include "imudata.h"

namespace tl {

template <typename T>
DataInterval ImuReadings_<T>::validInterval(const DataInterval& interval) const
{
    // TODO: double check
    const int start = std::max(interval.start, 0);
    const int end = [&interval, &start, this] {
        auto last = static_cast<int>(size() - 1);
        return (interval.end >= start && interval.end <= last) ? interval.end
                                                               : last;
    }();

    return {start, end};
}

template <typename T>
Eigen::Vector3d ImuReadings_<T>::mean(const DataInterval& interval) const
{
    const auto validInterval = this->validInterval(interval);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int i = validInterval.start; i <= validInterval.end; ++i) {
        mean += d()[i].asVector().template cast<double>();
    }

    mean /= validInterval.size();

    return mean;
}

template <typename T>
Eigen::Vector3d ImuReadings_<T>::variance(const DataInterval& interval) const
{
    const auto validInterval = this->validInterval(interval);
    const auto mean = this->mean(validInterval);
    Eigen::Vector3d variance = Eigen::Vector3d::Zero();
    for (int i = validInterval.start; i <= validInterval.end; ++i) {
        const auto diff = d()[i].asVector().template cast<double>() - mean;
        variance += (diff.array() * diff.array()).matrix();
    }
    variance /= (validInterval.size() - 1);

    return variance;
}

template <typename T>
size_t ImuReadings_<T>::findTimestamp(double ts) const
{
    size_t start{0};
    size_t end{size() - 1};
    while (end - start > 1) {
        const size_t mid = start + (end - start) / 2;
        if (ts > _samples[mid].timestamp()) {
            start = mid;
        }
        else {
            end = mid;
        }
    }

    // choose the closer one
    if (ts - _samples[start].timestamp() < _samples[end].timestamp() - ts) {
        return start;
    }

    return end;
}

template <typename T>
std::vector<DataInterval> ImuReadings_<T>::extractStaticIntervals(
    double threshold, int winSize) const
{
    constexpr int kMinWinSize{11};
    winSize = std::max(winSize, kMinWinSize);
    if (winSize & 1) {
        winSize++;
    }

    if (winSize >= size()) {
        return {};
    }

    const int halfWinSize = winSize / 2;

    std::vector<DataInterval> intervals;
    DataInterval current{};
    bool findStart{true};
    for (int i = halfWinSize; i < size() - halfWinSize; ++i) {
        const auto var = variance({i - halfWinSize, i + halfWinSize});
        const double norm = var.norm();

        if (findStart) {
            if (norm < threshold) {
                current.start = i;
                findStart = false;
            }
        }
        else {
            // TODO: use isApprox
            if (norm >= threshold) {
                current.end = i - 1;
                findStart = true;
                intervals.push_back(current);
            }
        }
    }

    // If the last interval has not been included in the intervals vector
    if (!findStart) {
        current.end = size() - halfWinSize - 1;
        intervals.push_back(current);
    }

    return intervals;
}

template <typename T>
ImuReadings_<T> ImuReadings_<T>::extractSamples(
    const std::vector<DataInterval>& intervals,
    std::vector<DataInterval>& extractedIntervals, int minIntervalSize,
    bool meansOnly) const
{
    const auto numValidInterval =
        std::count_if(intervals.begin(), intervals.end(),
                      [&minIntervalSize](const DataInterval& interval) {
                          return interval.size() >= minIntervalSize;
                      });

    ImuReadings_<T> extractedSamples;
    extractedSamples.reserve(meansOnly ? numValidInterval
                                       : numValidInterval * minIntervalSize);
    extractedIntervals.clear();
    extractedIntervals.reserve(numValidInterval);
    for (const auto& interval : intervals) {
        const int intervalSize = interval.size();
        if (intervalSize >= minIntervalSize) {
            extractedIntervals.push_back(interval);
            if (meansOnly) {
                double timestamp = _samples[interval.center()].timestamp();
                extractedSamples.d().emplace_back(timestamp, mean(interval));
            }
            else {
                for (int i{interval.start};
                     i < interval.start + minIntervalSize; i++) {
                    extractedSamples.d().push_back(_samples[i]);
                }
            }
        }
    }

    return extractedSamples;
}

template <typename T>
Eigen::MatrixX3<T> ImuReadings_<T>::toMatrix() const
{
    const auto size = _samples.size();
    Eigen::MatrixX3<T> mat{size, 3};
    for (size_t r{0}; r < size; ++r) {
        mat.row(r) = Eigen::RowVector3<T>{_samples[r].data()};
    }

    return mat;
}

template <typename T>
Timeline ImuReadings_<T>::timeline() const
{
    Timeline t;
    t.reserve(size());
    for (const auto& sample : _samples) {
        t.emplace_back(sample.timestamp());
    }

    return t;
}

} // namespace tl
