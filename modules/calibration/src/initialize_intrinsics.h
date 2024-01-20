#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

namespace tl {

struct FeatureCorrespondence2D3D;
struct SacParameters;
struct SacSummary;

// TODO: Better API
bool initializePinholeCamera(
    const std::vector<FeatureCorrespondence2D3D>& correspondences,
    const SacParameters& ransacParams, SacSummary& ransacSummary,
    Eigen::Matrix3d& orientation, Eigen::Vector3d& translation,
    double& focalLength);

bool initializeRadialUndistortionCamera(
    const std::vector<FeatureCorrespondence2D3D>& correspondences,
    const SacParameters& ransacParams, SacSummary& ransacSummary,
    int imageWidth, Eigen::Matrix3d& orientation, Eigen::Vector3d& translation,
    double& focalLength, double& radialDistortion);

// WARNING: Take from basalt(calibraiton_helper.cpp), however at the moment
// working with initializeRadialUndistortionCamera
bool initializeDoubleSphereModel(
    const std::vector<FeatureCorrespondence2D3D>& correspondences,
    const std::vector<int> board_ids, const cv::Size& board_size,
    const SacParameters& ransac_params, const cv::Size& img_size,
    SacSummary& ransacSummary, Eigen::Matrix3d& orientation,
    Eigen::Vector3d& translation, double& focalLength);

} // namespace tl
