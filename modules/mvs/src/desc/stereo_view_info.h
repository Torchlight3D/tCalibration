#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <glog/logging.h>

#include <AxCamera/Camera>

namespace thoht {

// First camera is treated as reference camera.
struct TwoViewInfo
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
    double focal_length1 = 0.;
    double focal_length2 = 0.;

    // Number of features that were matched and geometrically verified betwen
    // the images.
    int num_verified_matches = 0;

    // Number of inliers based on homography estimation. This is useful for
    // incremental SfM for choosing an initial view pair for the reconstruction.
    int num_homography_inliers = 0;

    // The visibility score is computed based on the inlier features from 2-view
    // geometry estimation. This score is similar to the number of verified
    // matches, but has a spatial weighting to encourage good coverage of the
    // image by the inliers. The visibility score here is the sum of the
    // visibility scores for each image.
    int visibility_score = 0;

    void SwapCameras();
};

inline Eigen::Vector3d MultiplyRotations(const Eigen::Vector3d& rvec1,
                                         const Eigen::Vector3d& rvec2)
{
    Eigen::Matrix3d rmat1, rmat2;
    ceres::AngleAxisToRotationMatrix(rvec1.data(), rmat1.data());
    ceres::AngleAxisToRotationMatrix(rvec2.data(), rmat2.data());

    const Eigen::Matrix3d rmat = rmat1 * rmat2;
    Eigen::Vector3d rvec;
    ceres::RotationMatrixToAngleAxis(rmat.data(), rvec.data());
    return rvec;
}

template <bool normalize_position = true>
void TwoViewInfoFromTwoCameras(const Camera& camera1, const Camera& camera2,
                               TwoViewInfo* info)
{
    CHECK_NOTNULL(info);

    const Eigen::Vector3d rotation1 = camera1.orientationAsAngleAxis();
    const Eigen::Vector3d rotation2 = camera2.orientationAsAngleAxis();

    info->focal_length1 = camera1.focalLength();
    info->focal_length2 = camera2.focalLength();

    info->rotation = MultiplyRotations(rotation2, -rotation1);

    // Compute the position of camera 2 in the coordinate system of camera 1
    // using the standard projection equation:
    //    X' = R * (X - c)
    // which yields:
    //    c2' = R1 * (c2 - c1).
    const Eigen::Vector3d offset = camera2.position() - camera1.position();
    ceres::AngleAxisRotatePoint(rotation1.data(), offset.data(),
                                info->position.data());

    // Scale the relative position to be a unit-length vector.
    if constexpr (normalize_position) {
        info->position.normalize();
    }
}

} // namespace thoht
