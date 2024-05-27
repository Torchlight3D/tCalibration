#pragma once

#include <Eigen/Dense>

class Sample
{
public:
    Sample(double x, double y, double weight, double yweight = 1.)
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
                bool silent = false);

    virtual int dimension() const;

    double rpeval(const Eigen::VectorXd& v, double x) const;

    Eigen::VectorXd rp_deriv(const Eigen::VectorXd& v, double x,
                             double& f) const;

    const std::vector<Sample>& get_data() const { return data; }

    double evaluate(Eigen::VectorXd& v);
    Eigen::VectorXd evaluate_derivative(Eigen::VectorXd& v);
    Eigen::VectorXd gauss_newton_direction(Eigen::VectorXd& v,
                                           Eigen::VectorXd& deriv,
                                           double& fsse);
    Eigen::VectorXd gauss_newton_armijo(Eigen::VectorXd& v);
    double peak(const Eigen::VectorXd& v) const;
    bool has_poles(const Eigen::VectorXd& v) const;

    inline double scale(double x) const { return (x - xs_min) * xs_scale; }

    inline double unscale(double x) const { return x / xs_scale + xs_min; }

    unsigned long long evaluations() const { return evaluation_count; }

    // TODO: static
    inline double cheb(int order, double x) const
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

public:
    const std::vector<Sample>& data;
    int order_n;
    int order_m;
    double base_value;

    double ysf;
    double pscale;
    bool silent;

    double xs_min;
    double xs_scale;

protected:
    unsigned long long evaluation_count;
};
