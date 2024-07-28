#pragma once

#include <Eigen/Core>

class Cubic_spline_surface
{
public:
    Cubic_spline_surface(int ncells_x, int ncells_y);

    Eigen::MatrixXd spline_fit(const std::vector<Eigen::Vector3d>& data,
                               double lambda, const Eigen::Vector2d& img_dims,
                               int pruning_threshold = 1);

private:
    std::vector<double> knots_x;
    std::vector<double> knots_y;

    int ncells_x;
    int ncells_y;

    double kmax_x;
    double kmax_y;
};
