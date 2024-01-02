#include "calib_observation.h"

namespace thoht {

//// CameraObservation starts from here
// CameraObservation::CameraObservation(CalibBoardBase::Ptr target,
//                                      const cv::Mat &image)
//     : m_calibrated(false)
//{
//     setImage(image);
//     setTarget(target);
// }

// void CameraObservation::setTarget(CalibBoardBase::Ptr target)
//{
//     // TODO: check nullptr
//     m_target = target;
//     clearObservations();
//     m_imgPoints.reserve(m_target->cornerCount());
//     m_observed.reserve(m_target->cornerCount());
// }

// CalibBoardBase::ConstPtr CameraObservation::target() const { return m_target;
// };

// void CameraObservation::setImage(const cv::Mat &image)
//{
//     if (image.empty()) {
//         return;
//     }

//    // TODO: use clone if decide to own
//    m_image = image.clone();
//    clearObservations();
//}

// cv::Mat CameraObservation::image() const { return m_image; };

// void CameraObservation::clearImage()
//{
//     // TODO: use release if decide to own
//     m_image.release();
//     clearObservations();
// }

// void CameraObservation::setImagePoints(
//     const std::vector<cv::Point2d> &imagePoints,
//     const std::vector<bool> &observed)
//{
//     if (imagePoints.size() == observed.size() &&
//         imagePoints.size() == m_target->cornerCount()) {
//         m_imgPoints = imagePoints;
//         m_observed = observed;
//     }
// }

// void CameraObservation::updateImagePoint(int i, const cv::Point2d &pt)
//{
//     // FIXME: check bound
//     m_imgPoints[i] = pt;
//     m_observed[i] = true;
// }

// void CameraObservation::removeImagePoint(int i)
//{
//     // FIXME: check bound
//     m_observed[i] = false;
// }

// bool CameraObservation::observed(int i) const
//{
//     // FIXME: check bound
//     return m_observed[i];
// }

// bool CameraObservation::hasSuccessfulObservation() const
//{
//     return std::any_of(m_observed.begin(), m_observed.end(),
//                        [](const auto &success) { return success; });
// }

// int CameraObservation::observedCornersInTarget(
//     std::vector<cv::Point3d> &obsCorners) const
//{
//     const int cornerCount = m_target->cornerCount();

//    obsCorners.clear();
//    obsCorners.reserve(cornerCount);
//    for (int i{0}; i < cornerCount; ++i) {
//        if (m_observed[i]) {
//            obsCorners.push_back(m_target->boardPoint(i));
//        }
//    }

//    return static_cast<int>(obsCorners.size());
//}

// int CameraObservation::observedCornersInImage(
//     std::vector<cv::Point2d> &obsCorners) const
//{
//     const auto cornerCount = m_target->cornerCount();

//    obsCorners.clear();
//    obsCorners.reserve(cornerCount);
//    for (int i{0}; i < cornerCount; ++i) {
//        if (m_observed[i]) {
//            obsCorners.push_back(m_imgPoints[i]);
//        }
//    }

//    return static_cast<int>(obsCorners.size());
//}

// int CameraObservation::observedCornerIndexes(
//     std::vector<int> &cornerIndexes) const
//{
//     const auto cornerCount = m_target->cornerCount();

//    cornerIndexes.clear();
//    cornerIndexes.reserve(cornerCount);
//    for (int i{0}; i < cornerCount; ++i) {
//        if (m_observed[i]) {
//            cornerIndexes.push_back(i);
//        }
//    }

//    return static_cast<int>(cornerIndexes.size());
//}

// cv::Vec2d CameraObservation::offset(int i) const
//{
//     // FIXME: check bound
//     return m_offsets[i];
// }

// double CameraObservation::reprojectionError(int i) const
//{
//     return cv::norm(offset(i));
// }

// void CameraObservation::setObservePose(const Eigen::Isometry3d &pose)
//{
//     m_pose = pose;
//     m_calibrated = true;
// };

// const Eigen::Isometry3d &CameraObservation::observePose() const
//{
//     return m_pose;
// };

// int CameraObservation::calcCornerReprojection(
//     const CameraModelBase::Ptr camera,
//     std::vector<cv::Point2d> &reprojPoints) const
//{
//     reprojPoints.clear();
//     if (!m_calibrated) {
//         return 0;
//     }

//    std::vector<cv::Point3d> targetPoints;
//    const auto cornerCount = observedCornersInTarget(targetPoints);
//    reprojPoints.reserve(cornerCount);
//    for (int i{0}; i < cornerCount; i++) {
//        Eigen::Vector3d p{targetPoints[i].x, targetPoints[i].y,
//                          targetPoints[i].z};
//        Eigen::VectorXd pReproj = observePose().inverse() * p;

//        Eigen::VectorXd pReprojImg;
//        camera->vsEuclideanToKeypoint(pReproj, pReprojImg);

//        reprojPoints.emplace_back(pReprojImg(0), pReprojImg(1));
//    }

//    return cornerCount;
//}

// void CameraObservation::clearObservations()
//{
//     m_imgPoints.clear();
//     m_observed.clear();
//     m_calibrated = false;
// }

// CalibObservation::CalibObservation(int cameraCount)
//{
//     m_obs.reserve(cameraCount);
// }

// CalibData::CalibData() {}

} // namespace thoht
