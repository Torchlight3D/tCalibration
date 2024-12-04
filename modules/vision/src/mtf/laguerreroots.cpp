#include "laguerreroots.h"

#include <algorithm>
#include <limits>

bool laguerre(const std::vector<cplex>& a, cplex& x, int& its)
{
    constexpr int MR = 8;
    constexpr int MT = 10;
    constexpr int MAXIT = MT * MR;
    constexpr double EPS = std::numeric_limits<double>::epsilon();
    constexpr double frac[MR + 1]{0.0,  0.5,  0.25, 0.75, 0.13,
                                  0.38, 0.62, 0.88, 1.0};

    int m = a.size() - 1;
    for (int iter = 1; iter <= MAXIT; iter++) {
        its = iter;
        cplex b = a[m];
        double err = std::abs(b);
        cplex d(0, 0);
        cplex f(0, 0);
        double abx = std::abs(x);
        for (int j = m - 1; j >= 0; j--) {
            f = x * f + d;
            d = x * d + b;
            b = x * b + a[j];
            err = std::abs(b) + abx * err;
        }
        err *= EPS;
        if (std::abs(b) <= err)
            return true; // on the root
        cplex g = d / b;
        cplex g2 = g * g;
        cplex h = g2 - 2.0 * f / b;
        cplex sq = std::sqrt(double(m - 1) * (double(m) * h - g2));
        cplex gp = g + sq;
        cplex gm = g - sq;
        double abp = std::abs(gp);
        double abm = std::abs(gm);
        gp = (abp < abm) ? gm : gp;
        cplex dx = std::max(abp, abm) > 0.0 ? double(m) / gp
                                            : std::polar(1 + abx, double(iter));
        cplex x1 = x - dx;
        if (x == x1)
            return true;
        if (iter % MT != 0) {
            x = x1;
        }
        else {
            x -= frac[iter / MT] * dx;
        }
    }
    // no convergence ...
    return false;
}

void lroots(const std::vector<cplex>& a, std::vector<cplex>& roots, bool polish)
{
    const double EPS = 1e-14;
    int its;

    int m = a.size() - 1;
    std::vector<cplex> ad(a);
    for (int j = m - 1; j >= 0; j--) {
        cplex x(0, 0);
        std::vector<cplex> ad_v(j + 2);
        for (int jj = 0; jj < j + 2; jj++) {
            ad_v[jj] = ad[jj];
        }
        laguerre(ad_v, x, its);
        if (std::abs(x.imag()) <= 2.0 * EPS * std::abs(x.real())) {
            x = cplex(x.real(), 0.0);
        }
        roots[j] = x;
        cplex b = ad[j + 1];
        for (int jj = j; jj >= 0; jj--) { // deflate the poly
            cplex c = ad[jj];
            ad[jj] = b;
            b = x * b + c;
        }
    }

    if (polish) {
        for (int j = 0; j < m; j++) {
            laguerre(a, roots[j], its);
        }
    }
}
