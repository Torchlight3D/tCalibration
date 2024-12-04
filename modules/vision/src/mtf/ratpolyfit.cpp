#include "ratpolyfit.h"

#include <format>

#include <Eigen/Dense>
#include <glog/logging.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

Ratpoly_fit::Ratpoly_fit(const std::vector<Sample>& data, int order_n,
                         int order_m, bool silent)
    : data(data),
      order_n(order_n),
      order_m(order_m),
      base_value(1.0),
      ysf(1),
      pscale(0.1),
      silent(silent),
      evaluation_count(0)
{
}

int Ratpoly_fit::dimension() const { return (order_n + 1 + order_m); }

double Ratpoly_fit::rpeval(const Eigen::VectorXd& v, double x) const
{
    double top_val = v[0];
    double p = x;
    for (int n = 1; n <= order_n; n++) {
        top_val += cheb(n, p) * v[n];
    }
    double bot_val = base_value;
    p = x;
    for (int m = 0; m < order_m; m++) {
        bot_val += cheb(m + 1, p) * v[m + order_n + 1];
    }

    return top_val / bot_val;
}

Eigen::VectorXd Ratpoly_fit::rp_deriv(const Eigen::VectorXd& v, double x,
                                      double& f) const
{
    // if x falls on a pole, we are in trouble
    // and should probably just return the zero vector?

    // TODO: we can probably combine this function with rp_deriv_eval ??

    VectorXd dp = VectorXd::Zero(v.rows());
    VectorXd dq = VectorXd::Zero(v.rows());

    double top_val = v[0];
    dp[0] = 1;
    for (int n = 1; n <= order_n; n++) {
        dp[n] = cheb(n, x);
        top_val += cheb(n, x) * v[n];
    }
    double bot_val = base_value;
    for (int m = 0; m < order_m; m++) {
        dq[m + order_n + 1] = cheb(m + 1, x);
        bot_val += cheb(m + 1, x) * v[m + order_n + 1];
    }

    double den = bot_val * bot_val;
    if (den < 1e-12) {
        return VectorXd::Zero(v.rows());
    }

    f = top_val / bot_val;
    den = 1.0 / den;

    return (dp * bot_val - top_val * dq) * den;
}

double Ratpoly_fit::cheb(int order, double x) const
{
    double y = 0;
    switch (order) {
        case 0:
            y = 1;
            break;
        case 1:
            y = x;
            break;
        case 2:
            y = 2 * x * x - 1;
            break;
        case 3:
            y = 4 * x * x * x - 3 * x;
            break;
        case 4:
            y = 8 * x * x * x * x - 8 * x * x + 1;
            break;
        case 5:
            y = 16 * x * x * x * x * x - 20 * x * x * x + 5 * x;
            break;
        case 6:
            y = 32 * x * x * x * x * x * x - 48 * x * x * x * x + 18 * x * x -
                1;
            break;
        default:
            fprintf(stderr, "Unsupported Cheb poly order requested!\n");
            break;
    }
    return y;
}

double Ratpoly_fit::evaluate(Eigen::VectorXd& v)
{
    double err = 0;
    for (size_t i = 0; i < data.size(); i++) {
        double w = data[i].weight * data[i].yweight;
        double z = rpeval(v, scale(data[i].x));
        double e = data[i].y * ysf - z;
        err += e * e * w;
    }
    evaluation_count++;
    return err * 0.5;
}

Eigen::VectorXd Ratpoly_fit::gauss_newton_direction(Eigen::VectorXd& v,
                                                    Eigen::VectorXd& deriv,
                                                    double& fsse)
{
    MatrixXd J(data.size(), v.rows());
    J.setZero();
    fsse = 0;

    VectorXd r(data.size());
    for (size_t m = 0; m < data.size(); m++) {
        double w = data[m].weight * data[m].yweight;
        double fx = 0;

        J.row(m) = rp_deriv(v, scale(data[m].x), fx);
        double e = fx - data[m].y * ysf;
        r[m] = e * w;
        fsse += e * e * w;
    }

    VectorXd direction =
        J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-r);

    fsse *= 0.5;
    deriv = J.transpose() * r;
    evaluation_count++;
    return direction;
}

Eigen::VectorXd Ratpoly_fit::gauss_newton_armijo(Eigen::VectorXd& v)
{
    const double tau = 0.5;
    const double c = 1e-4;
    double fx = 0;

    VectorXd grad;
    VectorXd next;
    VectorXd pk;
    for (int k = 0; k < 50; k++) {
        double alpha = 1.0;
        pk = gauss_newton_direction(v, grad, fx);

        double target = fx + c * alpha * pk.dot(grad);

        // iteratively step close until we have a
        // sufficient decrease (Armijo condition)
        int max_steps = 30;
        next = v + alpha * pk;
        while (evaluate(next) > target && --max_steps > 0) {
            target = fx + c * alpha * pk.dot(grad);
            alpha *= tau;
            next = v + alpha * pk;
        }

        double stepsize = pk.array().abs().maxCoeff() * std::abs(alpha);
        if (stepsize < 5e-8) {
            break;
        }
        v = next;
    }
    return v;
}

double Ratpoly_fit::peak(const Eigen::VectorXd& v) const
{
    double xmin = 1e50;
    double xmax = -1e50;
    for (size_t i = 0; i < data.size(); i++) {
        xmin = std::min(data[i].x, xmin);
        xmax = std::max(data[i].x, xmax);
    }

    // bracket the maximum
    double peak_z = 0;
    double peak_x = (xmin + xmax) * 0.5;
    double step = (xmax - xmin) / 20.0;
    for (double x = xmin; x <= xmax; x += step) {
        double z = rpeval(v, scale(x));
        if (z > peak_z) {
            peak_x = x;
            peak_z = z;
        }
    }

    // golden section search
    constexpr double phi = 0.61803398874989;
    double lower = peak_x - 2 * step;
    double upper = peak_x + 2 * step;
    double c = upper - phi * (upper - lower);
    double d = lower + phi * (upper - lower);
    const double tol = 1e-10;
    while ((upper - lower) > tol) {
        double fc = rpeval(v, scale(c));
        double fd = rpeval(v, scale(d));
        if (fc > fd) {
            upper = d;
            d = c;
            c = upper - phi * (upper - lower);
        }
        else {
            lower = c;
            c = d;
            d = lower + phi * (upper - lower);
        }
    }
    return 0.5 * (upper + lower);
}

bool Ratpoly_fit::has_poles(const Eigen::VectorXd& v) const
{
    double xmin = 1e50;
    double xmax = -1e50;
    for (size_t i = 0; i < data.size(); i++) {
        xmin = std::min(data[i].x, xmin);
        xmax = std::max(data[i].x, xmax);
    }

    // ensure the bounds are slightly wider than the actual data
    double span = xmax - xmin;
    xmin -= pscale * span;
    xmax += pscale * span;

    // now compute roots of bottom polynomial
    switch (order_m) {
        case 0:
            return false; // cannot have poles
        case 1: {
            double pole = unscale(-1.0 / v[order_n + 1]);

            if (!silent && pole >= xmin && pole <= xmax) {
                LOG(INFO) << std::format("pole at {} on [{}, {}]", pole, xmin,
                                         xmax);
            }

            return pole >= xmin && pole <= xmax;
        }
        case 2: {
            // since this is a chebyshev polynomial
            // a(x^2 - 1) -> c' = c - a
            double a = v[order_n + 2];
            double b = v[order_n + 1];
            double c = base_value - v[order_n + 2];
            double sb = b < 0 ? -1 : 1;
            double q = -0.5 * (b + sb * sqrt(b * b - 4 * a * c));
            double pole1 = unscale(q / a);
            double pole2 = unscale(c / q);

            if (!silent && ((pole1 >= xmin && pole1 <= xmax) ||
                            (pole2 >= xmin && pole2 <= xmax))) {
                LOG(INFO) << std::format("pole at {} or {} on [{}, {}]", pole1,
                                         pole2, xmin, xmax);
            }

            return (pole1 >= xmin && pole1 <= xmax) ||
                   (pole2 >= xmin && pole2 <= xmax);
        }
        default:
            // TODO: see NR chapter 5.6 for cubic roots
            LOG(ERROR) << std::format(
                "Warning: no implementation to compute roots of "
                "order-{} polynomial",
                order_m);
            return false;
    };
}
