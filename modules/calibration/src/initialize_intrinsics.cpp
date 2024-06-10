#include "initialize_intrinsics.h"

#include <iostream>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/calib3d.hpp>

#include <tCamera/Camera>
#include <tCamera/CameraIntrinsics>
#include <tCamera/DivisionUndistortionCameraModel>
#include <tCamera/DoubleSphereCameraModel>
#include <tMath/Eigen/Utils>
#include <tMath/RANSAC/RansacCreator>
#include <tMath/RANSAC/SampleConsensus>
#include <tMvs/PnP/EstimateCalibratedAbsolutePose>
#include <tMvs/PnP/EstimateUncalibratedAbsolutePose>
#include <tMvs/PnP/EstimateRadialDistortionUncalibratedAbsolutePose>
#include <tMvs/Feature>
#include <tVision/EigenCVUtils>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
const size_t kMinCorrespondence{20};
}

bool initializePinholeCamera(const std::vector<Feature2D3D>& corrs,
                             const SacParameters& sacParams,
                             SacSummary& sacSummary, Matrix3d& rotation,
                             Vector3d& position, double& focalLength)
{
    if (corrs.size() <= kMinCorrespondence) {
        LOG(ERROR) << "Not enough feature matching. "
                   << "(" << corrs.size() << "<" << kMinCorrespondence << ")";
        return false;
    }

    // TEST: OpenCV initialization
    //    std::vector<cv::Point2f> point2s;
    //    std::vector<cv::Point3f> point3s;
    //    for (const auto& corr : corrs) {
    //        LOG(INFO) << "2D point: " << corr.feature.transpose()
    //                  << "\n"
    //                     "3D point: "
    //                  << corr.world_point.transpose();
    //        point2s.push_back(eigenToCvPoint2(corr.feature));
    //        point3s.push_back(eigenToCvPoint3(corr.world_point));
    //    }
    //    std::vector<std::vector<cv::Point2f>> imgPoints{point2s};
    //    std::vector<std::vector<cv::Point3f>> objPoints{point3s};

    //    const auto K =
    //        cv::initCameraMatrix2D(objPoints, imgPoints, cv::Size{640, 480});

    //    VLOG(-1) << "OpenCV estimated focus length: " << K.at<float>(0, 0) <<
    //    ", "
    //             << K.at<float>(1, 1)
    //             << "\n"
    //                "OpenCV estimated center: "
    //             << K.at<double>(0, 2) << ", " << K.at<double>(1, 2);

    constexpr RansacType kSacType = RansacType::RANSAC;

    UncalibratedAbsolutePose pose;
    const bool success = EstimateUncalibratedAbsolutePose(
        sacParams, kSacType, corrs, &pose, &sacSummary);

    VLOG(-1) << "Estimated focal length: " << pose.focal_length
             << "\n"
                "Number of RANSAC inliers: "
             << sacSummary.inliers.size();

    if (sacSummary.inliers.size() < kMinCorrespondence) {
        LOG(ERROR) << "Failed initialization: "
                      "Not enough inliner matching. "
                   << "(" << sacSummary.inliers.size() << "<"
                   << kMinCorrespondence << ")";
        return false;
    }

    rotation = pose.rotation;
    position = pose.position;
    focalLength = pose.focal_length;

    return success;
}

bool initializeRadialUndistortionCamera(
    const std::vector<Feature2D3D>& correspondences,
    const SacParameters& sacParams, SacSummary& sacSummary, int imageWidth,
    Matrix3d& orienttion, Vector3d& position, double& focalLength,
    double& radialDistortion)
{
    if (correspondences.size() <= kMinCorrespondence) {
        LOG(ERROR) << "Not enough feature matching.";
        return false;
    }

    constexpr double kCenterRatio = 0.75;
    constexpr double kInterestRange = 0.5;

    RadialDistUncalibratedAbsolutePoseMetaData opts;
    opts.max_focal_length = (kCenterRatio + kInterestRange) * imageWidth;
    opts.min_focal_length = (kCenterRatio - kInterestRange) * imageWidth;

    RadialDistUncalibratedAbsolutePose pose;
    const bool success = EstimateRadialDistUncalibratedAbsolutePose(
        sacParams, RansacType::RANSAC, correspondences, opts, &pose,
        &sacSummary);

    orienttion = pose.rotation;
    position = -pose.rotation.transpose() * pose.translation;
    radialDistortion = pose.radial_distortion;
    focalLength = pose.focal_length;

    Camera cam;
    cam.setCameraIntrinsicsModel(CameraIntrinsicsType::DivisionUndistortion);
    auto intrinsics = cam.cameraIntrinsics();
    intrinsics->setFocalLength(focalLength);
    intrinsics->setPrincipalPoint(0., 0.);
    intrinsics->setParameter(DivisionUndistortionCameraModel::K,
                             radialDistortion);
    cam.setPosition(position);
    cam.setOrientationFromRotationMatrix(orienttion);

    // Calculate reprojection error
    double avgRpe{0.};
    for (const auto& correspondence : correspondences) {
        Vector2d pixel;
        cam.projectPoint(correspondence.world_point.homogeneous(), pixel);
        avgRpe += (pixel - correspondence.feature).norm();
    }
    avgRpe /= correspondences.size();

    VLOG(-1) << "Estimated focal length: " << pose.focal_length
             << "\n"
                "Estimated radial distortion: "
             << pose.radial_distortion
             << "\n"
                "Number of RANSAC inliers: "
             << sacSummary.inliers.size()
             << "\n"
                "Reprojection error: "
             << avgRpe;

    if (sacSummary.inliers.size() < kMinCorrespondence) {
        LOG(ERROR) << "Failed initialization: "
                      "Not enough inliner matching. "
                   << "(" << sacSummary.inliers.size() << "<"
                   << kMinCorrespondence << ")";
        return false;
    }

    constexpr double kMaxRpe{4.};
    if (avgRpe > kMaxRpe) {
        LOG(ERROR) << "Failed initialization: "
                      "Reprojection error is too big. "
                      "("
                   << avgRpe << ">" << kMaxRpe << ")";
        return false;
    }

    return success;
}

bool initializeDoubleSphereModel(const std::vector<Feature2D3D>& corres,
                                 const std::vector<int> board_ids,
                                 const cv::Size& board_size,
                                 const SacParameters& ransac_params,
                                 const cv::Size& img_size,
                                 SacSummary& ransac_summary, Matrix3d& rotation,
                                 Vector3d& position, double& focal_length)
{
    // Initialize the image center at the center of the image.
    eigen_map<int, Vector2d> id_to_corner;
    for (size_t i = 0; i < board_ids.size(); i++) {
        id_to_corner[board_ids[i]] = corres[i].feature;
    }

    // Now we try to find a non-radial line to initialize the focal length
    constexpr double _xi{1.0};
    const int target_cols = board_size.width;
    const int target_rows = board_size.height;

    double gamma0 = 0.0;
    double minRPE = std::numeric_limits<double>::max();
    bool success = false;
    for (size_t r = 0; r < target_rows; ++r) {
        eigen_vector<Vector4d> P;
        for (size_t c = 0; c < target_cols; ++c) {
            int corner_id = (r * target_cols + c);

            if (id_to_corner.find(corner_id) != id_to_corner.end()) {
                const Vector2d imagePoint = id_to_corner[corner_id];
                P.emplace_back(imagePoint[0], imagePoint[1], 0.5,
                               -0.5 * (imagePoint[0] * imagePoint[0] +
                                       imagePoint[1] * imagePoint[1]));
            }
        }

        constexpr int kMinCorners{8};
        if (P.size() > kMinCorners) {
            // Resize P to fit with the count of valid points.
            Eigen::Map<Eigen::Matrix4Xd> P_mat((double*)P.data(), 4, P.size());

            Eigen::MatrixXd P_mat_t = P_mat.transpose();

            Eigen::JacobiSVD<Eigen::MatrixXd> svd{
                P_mat_t, Eigen::ComputeThinU | Eigen::ComputeThinV};

            Vector4d C = svd.matrixV().col(3);

            double t = C(0) * C(0) + C(1) * C(1) + C(2) * C(3);
            if (t < 0) {
                continue;
            }

            // Check that line image is not radial
            double d = sqrt(1.0 / t);
            double nx = C(0) * d;
            double ny = C(1) * d;
            if (hypot(nx, ny) > 0.95) {
                continue;
            }

            double nz = sqrt(1.0 - nx * nx - ny * ny);
            double gamma = fabs(C(2) * d / nz);

            // Undistort points with intrinsic guess
            Camera cam;
            cam.setCameraIntrinsicsModel(CameraIntrinsicsType::DoubleSphere);
            auto intrinsics = cam.cameraIntrinsics();
            intrinsics->setFocalLength(0.5 * gamma);
            intrinsics->setPrincipalPoint(0.0, 0.0);
            intrinsics->setParameter(DoubleSphereCameraModel::Xi, 0.5 * _xi);
            intrinsics->setParameter(DoubleSphereCameraModel::Alpha, 0.0);

            auto new_corrs = corres;
            for (auto& corr : new_corrs) {
                corr.feature = cam.pixelToNormalizedCoordinates(corr.feature)
                                   .hnormalized();
            }

            CalibratedAbsolutePose pose;
            SacParameters params = ransac_params;
            params.error_thresh = 0.5 / img_size.width;
            constexpr PnPType type = PnPType::DLS;
            EstimateCalibratedAbsolutePose(params, RansacType::RANSAC, type,
                                           new_corrs, &pose, &ransac_summary);

            cam.setPosition(pose.position);
            cam.setOrientationFromRotationMatrix(pose.rotation);

            double repro_error = 0.0;
            int in_image = 0;
            for (const auto& corr : corres) {
                Vector2d pixel;
                cam.projectPoint(corr.world_point.homogeneous(), pixel);
                if (pixel(0) >= 0.0 && pixel(1) >= 0.0 &&
                    pixel(0) < img_size.width && pixel(1) < img_size.height) {
                    repro_error += (pixel - corr.feature).norm();
                    in_image++;
                }
            }

            if (in_image > kMinCorners) {
                double avg_reproj_error = repro_error / in_image;

                if (avg_reproj_error < minRPE && avg_reproj_error < 5.0 &&
                    ransac_summary.inliers.size() > 10) {
                    minRPE = avg_reproj_error;
                    gamma0 = gamma;
                    success = true;
                    rotation = pose.rotation;
                    position = pose.position;
                    focal_length = 0.5 * gamma0;

                    VLOG(-1) << "New minimum reprojection error: " << minRPE;

                    return success;
                }
            }
        }
    }

    return success;
}

} // namespace tl
