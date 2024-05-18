#pragma once

#include <Eigen/Core>

namespace tl {

struct Feature
{
    // 2D point
    Eigen::Vector2d point_ = Eigen::Vector2d::Zero();

    // 2D standard deviation
    Eigen::Matrix2d covariance_ = Eigen::Matrix2d::Identity();

    // Depth prior, some features might have a depth info from an RGB-D image
    double depth_prior_ = 0.0;

    // Depth prior variance
    double depth_prior_variance_ = 0.0;

    Feature() = default;
    Feature(double x, double y) : Feature(Eigen::Vector2d{x, y}) {}
    Feature(double x, double y, double depthPrior)
        : Feature(Eigen::Vector2d{x, y}, depthPrior)
    {
    }
    // No explicit on purpose
    Feature(const Eigen::Vector2d &point) : point_(point) {}
    Feature(const Eigen::Vector2d &point, double depth)
        : point_(point), depth_prior_(depth)
    {
    }
    Feature(const Eigen::Vector2d &point, const Eigen::Matrix2d &covariance)

        : point_(point), covariance_(covariance)
    {
    }
    Feature(const Eigen::Vector2d &point, const Eigen::Matrix2d &covariance,
            double depthPrior, double depthPriorVariance)
        : point_(point),
          covariance_(covariance),
          depth_prior_(depthPrior),
          depth_prior_variance_(depthPriorVariance)
    {
    }

    double x() const { return point_[0]; }
    double y() const { return point_[1]; }

    double depth_prior() const { return depth_prior_; }
    double depth_prior_variance() const { return depth_prior_variance_; }

    bool operator==(const Feature &o) const
    {
        return point_.x() == o.point_.x() && point_.y() == o.point_.y();
    }
    bool operator<(const Feature &o) const
    {
        return point_.x() < o.point_.x() ||
               (point_.x() == o.point_.x() && point_.y() < o.point_.y());
    }
};

} // namespace tl

template <>
struct std::hash<tl::Feature>
{
    size_t operator()(const tl::Feature &k) const
    {
        // Compute individual hash values for two data members and combine them
        // using XOR and bit shifting
        return ((hash<double>()(k.point_.x()) ^
                 (hash<double>()(k.point_.y()) << 1)) >>
                1);
    }
};
