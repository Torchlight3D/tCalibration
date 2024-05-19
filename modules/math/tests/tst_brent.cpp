#include <cmath>

#include <gtest/gtest.h>

#include <tMath/Solvers/BrentSolver>

#include "test_utils.h"

using namespace tl;

namespace {
constexpr double kDoubleTolerance{1e-5};
}

TEST(BrentSolver, Basics)
{
    // f(x) = sin(x) - 0.5x
    auto F1 = [](double x) -> double { return std::sin(x) - 0.5 * x; };

    // f(x) = 2x - exp(-x)
    auto F2 = [](double x) -> double { return 2. * x - std::exp(-x); };

    BrentSolver solver;
    EXPECT_NEAR(solver.solve(F1, 1., 2.), 1.89549, kDoubleTolerance);
    EXPECT_NEAR(solver.solve(F2, 0., 1.), 0.351734, kDoubleTolerance);
}

TEST(BrentSolver, Polynomial)
{
    // f(x) = x^3 + x^2 - 5x + 3
    Polynome poly{1., 1., -5., 3.};

    using std::placeholders::_1;
    auto func = std::bind(&Polynome::operator(), &poly, _1);

    BrentSolver solver;
    EXPECT_NEAR(solver.solve(func, -5., -2.), -3., kDoubleTolerance);
    EXPECT_NEAR(solver.solve(func, 0., 3.), 1., kDoubleTolerance);
}
