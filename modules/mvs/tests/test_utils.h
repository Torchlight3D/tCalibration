#pragma once

#include <ceres/rotation.h>
#include <Eigen/Core>

#include <tCore/Math>
#include <tCore/RandomGenerator>

namespace tl {

using Eigen::Vector2d;

// Adds noise to the ray i.e. the projection of the point.
inline void AddNoiseToProjection(double factor, RandomNumberGenerator* rng,
                                 Vector2d* ray)
{
    //    CHECK_NOTNULL(rng);
    *ray += Vector2d(rng->randFloat(-factor, factor),
                     rng->randFloat(-factor, factor));
}

inline Eigen::Matrix3d RandomRotation(double max_degrees_from_identity)
{
    Eigen::Vector2d::Random();
    const Eigen::Vector3d angle_axis =
        math::degToRad(max_degrees_from_identity) *
        Eigen::Vector3d::Random().normalized();
    Eigen::Matrix3d rotation;
    ceres::AngleAxisToRotationMatrix(
        angle_axis.data(), ceres::ColumnMajorAdapter3x3(rotation.data()));
    return rotation;
}

} // namespace tl
