#pragma once

struct Ordered_point
{
    double first;
    double second;

    // No explicit on purpose
    Ordered_point(double in_first = 0, double in_second = 0)
        : first(in_first), second(in_second)
    {
    }

    bool operator<(const Ordered_point& b) const { return first < b.first; }
};
