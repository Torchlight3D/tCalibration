#pragma once

// #include <glog/logging.h>
#include <ceres/rotation.h>
#include <tMath/MathBase>
#include <tMath/RandomGenerator>

namespace tl {

using Eigen::Vector2d;

// Adds noise to the ray i.e. the projection of the point.
inline void AddNoiseToProjection(double factor, RandomNumberGenerator* rng,
                                 Vector2d* ray)
{
    //    CHECK_NOTNULL(rng);
    *ray += Vector2d(rng->RandDouble(-factor, factor),
                     rng->RandDouble(-factor, factor));
}

inline Eigen::Matrix3d RandomRotation(double max_degrees_from_identity,
                                      RandomNumberGenerator* rng)
{
    const Eigen::Vector3d angle_axis =
        math::degToRad(max_degrees_from_identity) *
        rng->RandVector3d().normalized();
    Eigen::Matrix3d rotation;
    ceres::AngleAxisToRotationMatrix(
        angle_axis.data(), ceres::ColumnMajorAdapter3x3(rotation.data()));
    return rotation;
}

} // namespace tl
