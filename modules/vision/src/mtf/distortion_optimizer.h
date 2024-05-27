#pragma once

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

#include "block.h"

struct Ridge
{
    std::vector<cv::Point2d> ridge;
    cv::Point2d centroid;
    cv::Point2d normal;
    double weight = 0.;
    double residual = 0.;

    Ridge(const std::vector<cv::Point2d>& ridge, const cv::Point2d& centroid,
          const cv::Point2d& normal, double weight = 0.)
        : ridge(ridge), centroid(centroid), normal(normal), weight(weight)
    {
    }
};

class Distortion_optimizer
{
public:
    Distortion_optimizer(const std::vector<Block>& in_blocks,
                         const cv::Point2d& prin);

    cv::Point2d get_max_val() const;

    void solve();

    cv::Point2d inv_warp(const cv::Point2d& p, const Eigen::VectorXd& v);
    double model_not_invertible(const Eigen::VectorXd& v);

    double medcouple(std::vector<float>& x);
    double evaluate(const Eigen::VectorXd& v, double penalty = 1.0);

    void seed_simplex(Eigen::VectorXd& v, const Eigen::VectorXd& lambda);
    void simplex_sum(Eigen::VectorXd& psum);

    // TODO:Maybe duplicated in bundle.h
    void nelder_mead(double ftol, int& num_evals);
    double try_solution(Eigen::VectorXd& psum, int ihi, double fac);
    Eigen::VectorXd iterate(double tol);

    bool optimization_failure() const { return nelder_mead_failed; }

public:
    std::vector<Ridge> ridges;
    cv::Point2d prin;
    double radius_norm;
    Eigen::VectorXd best_sol;
    cv::Point2d max_val;
    double maxrad;

    // variables used by nelder-mead
    std::vector<Eigen::VectorXd> np;
    Eigen::VectorXd ny;
    bool nelder_mead_failed;
    Eigen::VectorXd initial;
    double focal_lower;
    double focal_upper;
    double focal_mode_constraint;
};
