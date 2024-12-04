#include "loessfit.h"

#include <algorithm>
#include <cmath>

#include "common_types.h"

namespace {
constexpr int kMinPointsToFit = 8;
}

double loess_core(std::vector<Ordered_point>& ordered, size_t start_idx,
                  size_t end_idx, double mid, cv::Point2d& sol)
{
    double rsq = 0;

    int n = end_idx - start_idx;

    if (n < kMinPointsToFit) {
        sol.x = 0;
        sol.y = 0;
        return 1e10;
    }

    double span = std::max(ordered[end_idx - 1].first - mid,
                           mid - ordered[start_idx].first);
    std::vector<double> sig(n, 1.0);
    for (int i = 0; i < n; i++) {
        double d = std::abs((ordered[i + start_idx].first - mid) / span) / 1.2;
        if (d > 1.0) {
            sig[i] = 20;
        }
        else {
            sig[i] = 1.0 / ((1 - d * d) * (1 - d * d) * (1 - d * d) + 1);
        }
    }

    double sx = 0;
    double sy = 0;
    double ss = 0;

    for (int i = 0; i < n; i++) {
        double weight = 1.0 / SQR(sig[i]);
        ss += weight;
        sx += ordered[i + start_idx].first * weight;
        sy += ordered[i + start_idx].second * weight;
    }
    double sxoss = sx / ss;

    double st2 = 0;
    double b = 0;
    for (int i = 0; i < n; i++) {
        double t = (ordered[i + start_idx].first - sxoss) / sig[i];
        st2 += t * t;
        b += t * ordered[i + start_idx].second / sig[i];
    }
    b /= st2;
    double a = (sy - sx * b) / ss;
    sol.x = a;
    sol.y = b;
    for (int i = 0; i < n; i++) {
        double r = (ordered[i + start_idx].first * sol.y + sol.x) -
                   ordered[i + start_idx].second;
        rsq += std::abs(r); // m-estimate of goodness-of-fit
    }
    return rsq / double(n);
}
