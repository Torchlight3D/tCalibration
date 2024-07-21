#pragma once

#include <Eigen/Core>

#include <tMvs/Types>

namespace tl {

class Camera;

// First camera is treated as reference camera.
struct ViewPairInfo
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
    double focalLength1 = 0.;
    double focalLength2 = 0.;

    // Number of features that were matched and geometrically verified betwen
    // the images.
    int num_verified_matches = 0;

    // Number of inliers based on homography estimation. This is useful for
    // incremental SfM for choosing an initial view pair for the scene.
    int num_homography_inliers = 0;

    // The visibility score is computed based on the inlier features from 2-view
    // geometry estimation. This score is similar to the number of verified
    // matches, but has a spatial weighting to encourage good coverage of the
    // image by the inliers. The visibility score here is the sum of the
    // visibility scores for each image.
    int visibility_score = 0;

    double scale_estimate = -1.;

    static ViewPairInfo fromCameraPair(const Camera& camera1,
                                       const Camera& camera2,
                                       bool normalize = true);

    void swapView();
};

struct ViewTripletInfo
{
    ViewId ids[3];
    ViewPairInfo info12, info13, info23;
};

} // namespace tl
