#pragma once

#include <functional>

namespace thoht {

class BrentSolver
{
public:
    struct Options
    {
    };

    using Function = std::function<double(double)>;
    double solve(Function func, double a, double b);
};

// TODO: add helper

} // namespace thoht
