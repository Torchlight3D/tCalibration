#include "math.h"

namespace tl {

namespace math {

double fast_atan2(double y, double x)
{
    double coeff_1 = four_pi<double>;
    double coeff_2 = 3 * coeff_1;
    double abs_y = fabs(y) + 1e-10; // kludge to prevent 0/0 condition

    double angle;

    if (x >= 0) {
        double r = (x - abs_y) / (x + abs_y);
        angle = coeff_1 - coeff_1 * r;
    }
    else {
        double r = (x + abs_y) / (abs_y - x);
        angle = coeff_2 - coeff_1 * r;
    }

    return (y < 0) ? -angle : angle; // negate if in quad III or IV
}

bool solveQuadraticEquation(double a, double b, double c, double& x1,
                            double& x2)
{
    if (isApprox0(a)) {
        x1 = -c / b;
        x2 = -c / b;
        return true;
    }

    double delta2 = b * b - 4. * a * c;
    if (delta2 < 0.) {
        return false;
    }

    double delta = std::sqrt(delta2);
    x1 = (-b + delta) / (2. * a);
    x2 = (-b - delta) / (2. * a);

    return true;
}

} // namespace math

} // namespace tl
