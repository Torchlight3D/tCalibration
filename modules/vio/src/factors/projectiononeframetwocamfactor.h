
#pragma once

#include <ceres/sized_cost_function.h>
#include <Eigen/Core>

namespace tl {

// Given:
//
// Estimate:
//
class ProjectionFactor final
    : public ceres::SizedCostFunction<2, // residual: Rpe
                                      7, // Left camera pose
                                      7, // Right camera pose
                                      7, // Camera to IMU
                                      1> // Inverse depth
{
public:
    ProjectionFactor(const Eigen::Vector3d &point1,
                     const Eigen::Vector3d &point2);

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override;

public:
    static Eigen::Matrix2d sqrt_info;

private:
    const Eigen::Vector3d _point1, _point2;

    Eigen::Matrix<double, 2, 3> tangent_base;
};

// Given:
// + Point in Camera1/Camera2
// + Velocity of Camera1/Camera2
// +
// Estimate:
//
class ProjectionOneFrameTwoCamFactor final
    : public ceres::SizedCostFunction<2, 7, 7, 1, 1>
{
public:
    ProjectionOneFrameTwoCamFactor(const Eigen::Vector3d &point1,
                                   const Eigen::Vector3d &point2,
                                   const Eigen::Vector2d &velocity1,
                                   const Eigen::Vector2d &velocity2, double td1,
                                   double td2);

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override;

public:
    const Eigen::Vector3d _point1, _point2;
    const Eigen::Vector3d _velocity1, _velocity2;
    const double _td1, _td2;

    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
};

// Given:
//
// Estimate:
//
// TODO:
// 1. Make TR and ROW as configurable parameter
class ProjectionTdFactor final
    : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1>
{
public:
    ProjectionTdFactor(const Eigen::Vector3d &point1,
                       const Eigen::Vector3d &point2,
                       const Eigen::Vector2d &velocity1,
                       const Eigen::Vector2d &velocity2, double td1, double td2,
                       double row1, double row2);

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override;

public:
    const Eigen::Vector3d _point1, _point2;
    const Eigen::Vector3d _velocity1, _velocity2;
    const double _td1, _td2;
    const double _row1, _row2;

    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
};

// Given:
// + Point in Camera1/Camera2
// + Velocity of Camera1/Camera2
// +
// Estimate:
//
class ProjectionTwoFrameOneCamFactor final
    : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1>
{
public:
    ProjectionTwoFrameOneCamFactor(const Eigen::Vector3d &point1,
                                   const Eigen::Vector3d &point2,
                                   const Eigen::Vector2d &velocity1,
                                   const Eigen::Vector2d &velocity2, double td1,
                                   double td2);

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override;

public:
    const Eigen::Vector3d _point1, _point2;
    const Eigen::Vector3d _velocity1, _velocity2;
    const double _td1, _td2;

    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
};

// Given:
// + Point in Camera1/Camera2
// + Velocity of Camera1/Camera2
// +
// Estimate:
//
class ProjectionTwoFrameTwoCamFactor final
    : public ceres::SizedCostFunction<2, 7, 7, 7, 7, 1, 1>
{
public:
    ProjectionTwoFrameTwoCamFactor(const Eigen::Vector3d &point1,
                                   const Eigen::Vector3d &point2,
                                   const Eigen::Vector2d &velocity1,
                                   const Eigen::Vector2d &velocity2, double td1,
                                   double td2);

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override;

public:
    const Eigen::Vector3d _point1, _point2;
    const Eigen::Vector3d _velocity1, _velocity2;
    const double _td1, _td2;

    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
};

} // namespace tl
