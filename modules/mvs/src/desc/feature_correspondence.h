#pragma once

#include <Eigen/Core>

#include "feature.h"

namespace thoht {

struct FeatureCorrespondence
{
    Feature feature1;
    Feature feature2;

    FeatureCorrespondence() = default;
    FeatureCorrespondence(const Feature& f1, const Feature& f2)
        : feature1(f1), feature2(f2)
    {
    }

    bool operator==(const FeatureCorrespondence& o) const
    {
        return feature1 == o.feature1 && feature2 == o.feature2;
    }
};

struct FeatureCorrespondence2D3D
{
    Eigen::Vector2d feature;
    Eigen::Vector3d world_point;

    FeatureCorrespondence2D3D() = default;
    FeatureCorrespondence2D3D(Eigen::Vector2d feature,
                              Eigen::Vector3d world_point)
        : feature(feature), world_point(world_point)
    {
    }
};

} // namespace thoht
