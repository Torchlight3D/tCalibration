#pragma once

#include "threadpool.h"

class Stride_range
{
public:
    Stride_range(size_t start, size_t end, size_t stride);

    size_t begin() const;

    size_t end() const;

    size_t& increment(size_t& v) const;

    template <class T>
    static void parallel_for(T& ftor, ThreadPool& tp, size_t ceiling)
    {
        size_t stride = std::min(ceiling, tp.size());
        size_t rpt = ceiling / stride; // rows per thread
        size_t remainder = ceiling - rpt * stride;

        std::vector<std::future<void>> futures;
        for (size_t b = 0; b < stride; b++) {
            size_t lower = b;
            size_t upper = b < remainder ? lower + rpt * stride
                                         : lower + (rpt - 1) * stride;
            Stride_range sr(lower, upper, stride);

            futures.emplace_back(tp.enqueue([sr, &ftor] { ftor(sr); }));
        }
        for (const auto& future : futures) {
            future.wait();
        }
    }

    size_t first;
    size_t last;
    size_t stride;
};
