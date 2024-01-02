#pragma once

#include <vector>
#include <Eigen/Core>

#include "bundle_adjustment.h"

namespace thoht {

struct TwoViewBundleAdjustmentOptions
{
    BundleAdjustmentOptions ba_options;
    bool constant_camera1_intrinsics{true};
    bool constant_camera2_intrinsics{true};
};

class Camera;
struct TwoViewInfo;
struct FeatureCorrespondence;

// Performs bundle adjustment on the two views assuming that both views observe
// all of the 3D points. The cameras should be initialized with intrinsics and
// extrinsics appropriately, and the 3D points should be set (e.g., from
// triangulation) before calling this method. The first camera pose is held
// constant during BA, and the optimized pose of the second camera, (optionally)
// intrinsics, and 3D points are returned. The indices of the feature
// correspondences should match the 3D point indices.
BundleAdjustmentSummary BundleAdjustTwoViews(
    const TwoViewBundleAdjustmentOptions& options,
    const std::vector<FeatureCorrespondence>& correspondences, Camera* camera1,
    Camera* camera2, std::vector<Eigen::Vector4d>* points3d);

// Performs bundle adjustment to find the optimal rotation and translation
// describing the two views. This is done without the need for 3D points, as
// described in "Exact Two-Image Structure from Motion" by John Oliensis (PAMI
// 2002).
//
// NOTE: The correspondences must be normalized by the focal length and
// principal point.
BundleAdjustmentSummary BundleAdjustTwoViewsAngular(
    const BundleAdjustmentOptions& options,
    const std::vector<FeatureCorrespondence>& correspondences,
    TwoViewInfo* info);

// Optimizes a fundamentral matrix. It uses the manifold presented in
// Non-Linear Estimation of the Fundamental Matrix With Minimal Parameters,
// PAMI 2004, by Bartoli and Sturm
BundleAdjustmentSummary OptimizeFundamentalMatrix(
    const BundleAdjustmentOptions& options,
    const std::vector<FeatureCorrespondence>& correspondences,
    Eigen::Matrix3d* fundamentalMatrix);

} // namespace thoht
