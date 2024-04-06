#pragma once

#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>

class Cubic_spline_surface
{
public:
    Cubic_spline_surface(int ncells_x, int ncells_y);

    Eigen::MatrixXd spline_fit(const std::vector<Eigen::Vector3d>& data,
                               double lambda, const Eigen::Vector2d& img_dims,
                               int pruning_threshold = 1);

private:
    inline double B3(double t)
    {
        if (t < 0)
            return 0;
        if (t >= 4)
            return 0;
        if (t < 1)
            return t * t * t / 6.0;
        if (t < 2)
            return (-3 * t * t * t + 12 * t * t - 12 * t + 4) / 6.0;
        if (t < 3)
            return (3 * t * t * t - 24 * t * t + 60 * t - 44) / 6.0;
        return (4 - t) * (4 - t) * (4 - t) / 6.0;
    }

private:
    std::vector<double> knots_x;
    std::vector<double> knots_y;

    int ncells_x;
    int ncells_y;

    double kmax_x;
    double kmax_y;
};
