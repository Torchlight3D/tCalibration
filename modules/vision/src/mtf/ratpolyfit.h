#pragma once

#include <Eigen/Dense>

class Sample
{
public:
    Sample(double x, double y, double weight, double yweight = 1)
        : x(x), y(y), weight(weight), yweight(yweight)
    {
    }

    bool operator<(const Sample& b) const { return x < b.x; }

    double x;
    double y;
    double weight;
    double yweight;
};

class Ratpoly_fit
{
public:
    Ratpoly_fit(const std::vector<Sample>& data, int order_n, int order_m,
                bool silent = false)
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

    virtual int dimension() { return (order_n + 1 + order_m); }

    inline double rpeval(const Eigen::VectorXd& v, double x)
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

    inline Eigen::VectorXd rp_deriv(const Eigen::VectorXd& v, double x,
                                    double& f)
    {
        // if x falls on a pole, we are in trouble
        // and should probably just return the zero vector?

        // TODO: we can probably combine this function with rp_deriv_eval ??

        Eigen::VectorXd dp = Eigen::VectorXd::Zero(v.rows());
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(v.rows());

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
            return Eigen::VectorXd::Zero(v.rows());
        }

        f = top_val / bot_val;
        den = 1.0 / den;

        return (dp * bot_val - top_val * dq) * den;
    }

    const std::vector<Sample>& get_data() const { return data; }

    inline double cheb(int order, double x)
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
                y = 32 * x * x * x * x * x * x - 48 * x * x * x * x +
                    18 * x * x - 1;
                break;
            default:
                fprintf(stderr, "Unsupported Cheb poly order requested!\n");
                break;
        }
        return y;
    }

    double evaluate(Eigen::VectorXd& v);
    Eigen::VectorXd evaluate_derivative(Eigen::VectorXd& v);
    Eigen::VectorXd gauss_newton_direction(Eigen::VectorXd& v,
                                           Eigen::VectorXd& deriv,
                                           double& fsse);
    Eigen::VectorXd gauss_newton_armijo(Eigen::VectorXd& v);
    double peak(const Eigen::VectorXd& v);
    bool has_poles(const Eigen::VectorXd& v);

    const std::vector<Sample>& data;
    int order_n;
    int order_m;
    double base_value;

    double ysf;
    double pscale;
    bool silent;

    double xs_min;
    double xs_scale;

    inline double scale(double x) const { return (x - xs_min) * xs_scale; }

    inline double unscale(double x) const { return x / xs_scale + xs_min; }

    unsigned long long evaluations() { return evaluation_count; }

protected:
    unsigned long long evaluation_count;
};
