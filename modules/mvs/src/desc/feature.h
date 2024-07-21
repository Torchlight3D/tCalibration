#pragma once

#include <Eigen/Core>

namespace tl {

struct Feature
{
    // 2D pos
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();

    // 2D standard deviation
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();

    // Depth prior, some features might have a depth info from an RGB-D image
    double depthPrior = 0.;

    // Depth prior variance
    double depthPriorVar = 0.;

    Feature() = default;
    Feature(double x, double y) : Feature(Eigen::Vector2d{x, y}) {}
    Feature(double x, double y, double depthPrior)
        : Feature(Eigen::Vector2d{x, y}, depthPrior)
    {
    }
    // No explicit on purpose
    Feature(const Eigen::Vector2d &point) : Feature(point, 0.) {}
    Feature(const Eigen::Vector2d &point, double depth)
        : pos(point), depthPrior(depth)
    {
    }
    Feature(const Eigen::Vector2d &point, const Eigen::Matrix2d &covariance)
        : pos(point), covariance(covariance)
    {
    }

    auto x() const { return pos.x(); }
    auto y() const { return pos.y(); }

    bool operator==(const Feature &o) const { return pos.isApprox(o.pos); }
    bool operator<(const Feature &o) const
    {
        return x() < o.x() || (x() == o.x() && y() < o.y());
    }
};

struct Feature2D2D
{
    Feature feature1;
    Feature feature2;

    Feature2D2D() = default;

    bool operator==(const Feature2D2D &o) const
    {
        return feature1 == o.feature1 && feature2 == o.feature2;
    }
};

struct Feature2D3D
{
    Eigen::Vector2d feature;
    Eigen::Vector3d world_point;

    Feature2D3D() = default;
};

} // namespace tl

template <>
struct std::hash<tl::Feature>
{
    size_t operator()(const tl::Feature &feat) const
    {
        const hash<double> hasher{};
        return ((hasher(feat.x()) ^ (hasher(feat.y()) << 1)) >> 1);
    }
};
