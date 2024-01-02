#pragma once

#include <Eigen/Core>

namespace thoht {

class DistortionInterface
{
public:
    DistortionInterface() {}
    virtual ~DistortionInterface() {}

    virtual void distort(Eigen::Vector2d& point) const = 0;
    virtual void distort(Eigen::Vector2d& point,
                         Eigen::Matrix2d& jacobian) const = 0;

    virtual void undistort(Eigen::Vector2d& point) const = 0;
    virtual void undistort(Eigen::Vector2d& point,
                           Eigen::Matrix2d& jacobian) const = 0;
};

} // namespace thoht
