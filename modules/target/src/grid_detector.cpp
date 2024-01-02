// #include "GridDetector.h"

// #include <iomanip>

// #include <opencv2/imgproc.hpp>

// #include "../camera/CameraModelBase.h"
// #include "CalibBoardBase.h"
// #include "CalibObservation.h"

// namespace thoht {

// GridDetector::GridDetector(CameraModelBase::Ptr geometry,
//                            CalibBoardBase::Ptr target, const Options
//                            &options)
//     : _geometry(geometry), _target(target), _options(options)
//{
// }

// GridDetector::~GridDetector() = default;

// bool GridDetector::findTarget(const cv::Mat &image,
//                               CalibObservation &outObservation,
//                               bool estimateTransformation) const
//{
//     auto findTargetNoTransformation =
//         [this](const cv::Mat &image, CalibObservation &observation) -> bool {
//         std::vector<cv::Point2d> imagePoints;
//         std::vector<bool> validCorners;
//         const bool success =
//             _target->computeObservation(image, imagePoints, validCorners);

//        observation.setTarget(_target);
//        observation.setImage(image);

//        for (int i = 0; i < imagePoints.size(); i++) {
//            if (validCorners[i]) {
//                observation.updateImagePoint(i, imagePoints[i]);
//            }
//        }

//        return success;
//    };

//    const bool success = findTargetNoTransformation(image, outObservation);

//    sm::kinematics::Transformation trafo;
//    // calculate trafo cam-target
//    if (success) {
//        // also estimate the transformation:
//        success = _geometry->estimateTransformation(outObservation, trafo);
//        if (success) {
//            outObservation.setObservePose(trafo);
//        }
//        else {
//            //      SM_DEBUG_STREAM("estimateTransformation() failed");
//        }
//    }

//    // calculate reprojection errors
//    auto compute_stats = [&](double &mean, double &std,
//                             Eigen::MatrixXd &reprojection_errors_norm,
//                             std::vector<cv::Point2f> &corners_reproj,
//                             std::vector<cv::Point2f> &corners_detected) {
//        corners_reproj.clear();
//        corners_detected.clear();
//        outObservation.calcCornerReprojection(_geometry, corners_reproj);
//        unsigned int numCorners =
//            outObservation.cornersInImageFrame(corners_detected);

//        // calculate error norm
//        reprojection_errors_norm = Eigen::MatrixXd::Zero(numCorners, 1);
//        for (unsigned int i = 0; i < numCorners; i++) {
//            cv::Point2f reprojection_err =
//                corners_detected[i] - corners_reproj[i];

//            reprojection_errors_norm(i, 0) = cv::norm(reprojection_err);
//        }

//        // calculate statistics
//        mean = reprojection_errors_norm.mean();
//        std = 0.0;
//        for (unsigned int i = 0; i < numCorners; i++) {
//            double temp = reprojection_errors_norm(i, 0) - mean;
//            std += temp * temp;
//        }
//        std /= (double)numCorners;
//        std = sqrt(std);
//    };

//    // remove corners with a reprojection error above a threshold
//    //(remove detection outliers)
//    if (_options.filterCornerOutliers && success) {
//        // calculate reprojection errors
//        double mean, std;
//        Eigen::MatrixXd reprojection_errors_norm;
//        std::vector<cv::Point2f> corners_reproj, corners_detected;
//        compute_stats(mean, std, reprojection_errors_norm, corners_reproj,
//                      corners_detected);

//        // disable outlier corners
//        std::vector<int> cornerIdx;
//        outObservation.observedCornerIndexes(cornerIdx);

//        int removeCount{0};
//        for (size_t i{0}; i < corners_detected.size(); i++) {
//            if (reprojection_errors_norm(i, 0) >
//                    mean + _options.filterCornerSigmaThreshold * std &&
//                reprojection_errors_norm(i, 0) >
//                    _options.filterCornerMinReprojError) {
//                outObservation.removeImagePoint(cornerIdx[i]);
//                removeCount++;
//            }
//        }

//        if (removeCount > 0) {
//            //      SM_DEBUG_STREAM("removed " << removeCount << " of " <<
//            //      reprojection_errors_norm.rows() << " calibration target
//            //      corner outliers\n";);
//        }
//    }

//    // show plot of reprojected corners
//    if (_options.plotCornerReprojection) {
//        cv::Mat imageCopy1 = image.clone();
//        cv::cvtColor(imageCopy1, imageCopy1, cv::COLOR_GRAY2RGB);

//        if (success) {
//            // calculate reprojection
//            std::vector<cv::Point2d> reprojs;
//            outObservation.calcCornerReprojection(_geometry, reprojs);

//            for (const auto &point : reprojs)
//                cv::circle(imageCopy1, point, 3, CV_RGB(255, 0, 0), 1);

//            // calculate reprojection errors
//            double mean, std;
//            Eigen::MatrixXd reprojection_errors_norm;
//            std::vector<cv::Point2f> corners_reproj, corners_detected;
//            compute_stats(mean, std, reprojection_errors_norm, corners_reproj,
//                          corners_detected);

//            // show the on the rendered image
//            auto format_str = [](double data) {
//                std::ostringstream oss;
//                oss << std::setprecision(3) << data;
//                return oss.str();
//            };
//            cv::putText(imageCopy1, "reproj err mean: " + format_str(mean),
//                        cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
//                        CV_RGB(0, 255, 0), 3, 8, false);
//            cv::putText(imageCopy1, "reproj err std: " + format_str(std),
//                        cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8,
//                        CV_RGB(0, 255, 0), 3, 8, false);
//        }
//        else {
//            cv::putText(imageCopy1, "Detection failed! (frame not used)",
//                        cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
//                        CV_RGB(255, 0, 0), 3, 8, false);
//        }

//        //    cv::imshow("Corner reprojection", imageCopy1);  // OpenCV call
//        //    if (_options.imageStepping) {
//        //      cv::waitKey(0);
//        //    } else {
//        //      cv::waitKey(1);
//        //    }
//    }

//    return success;
//}

//} // namespace thoht
