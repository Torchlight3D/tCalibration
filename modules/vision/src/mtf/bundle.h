#pragma once

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>

using Vector2dList = std::vector<Eigen::Vector2d>;
using Vector3dList = std::vector<Eigen::Vector3d>;

class Bundle_adjuster
{
public:
    Bundle_adjuster(Vector2dList& img_points, Vector3dList& world_points,
                    Eigen::Vector3d& t, cv::Mat in_rod_angles,
                    double distortion, double w, double fid_diameter,
                    double img_scale = 1.);

    void solve();

    void unpack(Eigen::Matrix3d& R, Eigen::Vector3d& t, double& distortion,
                double& w);

    double evaluate(const Eigen::VectorXd& v, double penalty = 1.0);

    void seed_simplex(Eigen::VectorXd& v, const Eigen::VectorXd& lambda);

    void simplex_sum(Eigen::VectorXd& psum);

    void nelder_mead(double ftol, int& num_evals);

    double try_solution(Eigen::VectorXd& psum, int ihi, double fac);
    Eigen::VectorXd iterate(double tol);

    bool optimization_failure() const;

public:
    const Vector2dList& img_points;
    const Vector3dList& world_points;
    cv::Mat rot_mat;
    cv::Mat rod_angles;
    Eigen::VectorXd best_sol;
    double fid_diameter;
    double img_scale;

    // variables used by nelder-mead
    std::vector<Eigen::VectorXd> np;
    Eigen::VectorXd ny;
    bool nelder_mead_failed;
    Eigen::VectorXd initial;
    double focal_lower;
    double focal_upper;
    double focal_mode_constraint;
};
