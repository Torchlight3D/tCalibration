#include "test_utils.h"

#include <ceres/rotation.h>

#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMvs/Poses/AlignRotations>
#include <tMvs/Registration/AlignPointCloud>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

void AddNoiseToVector2(double factor, Eigen::Vector2d* vec)
{
    // [-factor, factor]
    *vec += (Vector2d::Random() * factor);
}

Eigen::Matrix3d RandomRotation(double angle)
{
    const Vector3d rvec =
        math::degToRad(angle) * Vector3d::Random().normalized();

    Matrix3d R;
    ceres::AngleAxisToRotationMatrix(rvec.data(),
                                     ceres::ColumnMajorAdapter3x3(R.data()));
    return R;
}

void AlignOrientations(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_rotations,
    std::unordered_map<ViewId, Eigen::Vector3d>* rotations)
{
    // Collect all rotations into a vector.
    std::vector<Vector3d> gt_rots, rots;
    std::unordered_map<int, ViewId> indexToViewId;
    {
        auto index{0};
        for (const auto& [viewId, gt_rotation] : gt_rotations) {
            gt_rots.push_back(gt_rotation);
            rots.push_back(con::FindOrDie(*rotations, viewId));
            indexToViewId[index] = viewId;
            ++index;
        }
    }

    AlignRotations(gt_rots, &rots);

    for (size_t i{0}; i < rots.size(); ++i) {
        const auto viewId = con::FindOrDie(indexToViewId, i);
        (*rotations)[viewId] = rots[i];
    }
}

void AlignPositions(
    const std::unordered_map<ViewId, Eigen::Vector3d>& gt_positions,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    // Collect all positions into a vector.
    std::vector<Vector3d> gt_pos, pos;
    for (const auto& [viewId, gt_position] : gt_positions) {
        gt_pos.push_back(gt_position);
        const auto& position = con::FindOrDie(*positions, viewId);
        pos.push_back(position);
    }

    Matrix3d rotation;
    Vector3d translation;
    double scale;
    AlignPointCloudsUmeyama(pos, gt_pos, &rotation, &translation, &scale);

    // Apply the similarity transformation.
    for (auto& [_, position] : *positions) {
        position = scale * (rotation * position) + translation;
    }
}

} // namespace tl
