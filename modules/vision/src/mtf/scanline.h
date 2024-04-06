#pragma once

#include <algorithm>

class scanline
{
public:
    scanline(int in_start = 0, int in_end = 0) : start(in_start), end(in_end) {}

    inline void update(int x)
    {
        start = std::min(start, x);
        end = std::max(end, x);
    }

    int start;
    int end;
};
