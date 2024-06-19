#pragma once

namespace tl {

template <typename T>
class ImuReadings_;

// TODO:
// 1. Consider using size_t -> int
// 2. Can't explicit, why?

struct DataInterval
{
    int start, end;

    DataInterval(int start = -1, int end = -1) : start(start), end(end) {}

    inline bool isValid() const { return start >= 0 && end >= start; }

    inline int size() const { return end - start + 1; }

    inline int center() const { return start + size() / 2; }

    template <typename T>
    static DataInterval fromPeriod(const ImuReadings_<T>& samples, double start,
                                   double end);

    template <typename T>
    static DataInterval initialInterval(const ImuReadings_<T>& samples,
                                        double duration);

    template <typename T>
    static DataInterval finalInterval(const ImuReadings_<T>& samples,
                                      double duration);
};

} // namespace tl

#include "imudatainterval.impl.hpp"
