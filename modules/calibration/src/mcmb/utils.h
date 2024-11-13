#pragma once

#include <opencv2/core/mat.hpp>

namespace tl::mcmb {

// FIXME: Duplicated
double median(std::vector<double> &v);

// Fit point set to line ax + by + c = 0, res is the residual
double calcLinePara(const std::vector<cv::Point2f> &points, double &a,
                    double &b, double &c);

cv::Mat RT2Proj(cv::InputArray rmat, cv::InputArray tvec);
cv::Mat RVecT2Proj(cv::InputArray rvec, cv::InputArray tvec);
cv::Mat RVecT2ProjInt(cv::InputArray rvec, cv::InputArray tvec,
                      cv::InputArray K);
void Proj2RT(cv::InputArray T, cv::OutputArray rvec, cv::OutputArray tvec);

void invertRvecT(cv::InputArray rvec, cv::InputArray tvec,
                 cv::OutputArray rvec_inv, cv::OutputArray tvec_inv);
void invertRvecT(cv::InputOutputArray rvec, cv::InputOutputArray tvec);

// size 6: rvec, tvec
cv::Mat vectorProj(std::vector<float> ProjV);
std::array<float, 6> ProjToVec(cv::InputArray T);

std::vector<cv::Point3f> transformPoints(const std::vector<cv::Point3f> &points,
                                         cv::Mat rvec, cv::Mat tvec);

void projectPointsWithDistortion(cv::InputArray objectPoints,
                                 cv::InputArray rvec, cv::InputArray tvec,
                                 cv::InputArray cameraMatrix,
                                 cv::InputArray distortion,
                                 cv::OutputArray imagePoints, int type);

cv::Point3f triangulatePointNViews(const std::vector<cv::Point2f> &point2s,
                                   const std::vector<cv::Mat> &rvecs,
                                   const std::vector<cv::Mat> &tvecs,
                                   cv::InputArray cameraMatrix);

cv::Point3f triangulatePointNViewsRansac(
    const std::vector<cv::Point2f> &point2s, const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs, cv::InputArray cameraMatrix,
    cv::InputArray disortion, double threshold, double confidence,
    int iterations);

cv::Mat _solveP3PRansac(const std::vector<cv::Point3f> &objectPoints,
                        const std::vector<cv::Point2f> &imagePoints,
                        cv::InputArray cameraMatrix, cv::InputArray distortion,
                        cv::OutputArray rvec, cv::OutputArray tvec,
                        float threshold, int iterations,
                        double confidence = 0.99, bool refine = true);

cv::Mat solveP3PRansac(const std::vector<cv::Point3f> &objectPoints,
                       const std::vector<cv::Point2f> &imagePoints,
                       cv::InputArray cameraMatrix, cv::InputArray distortion,
                       cv::OutputArray rvec, cv::OutputArray tvec,
                       float threshold, int iterations, int type,
                       double confidence = 0.99, bool refine = true);

cv::Mat solvePnPRansac(cv::InputArray objectPoints, cv::InputArray imagePoints,
                       cv::InputArray cameraMatrix, cv::InputArray distortion,
                       cv::OutputArray rvec, cv::OutputArray tvec,
                       int iteration, float threshold, int type,
                       double confidence = 0.99, bool refine = true);

cv::Mat handeyeCalibration(const std::vector<cv::Mat> &pose_abs_1,
                           const std::vector<cv::Mat> &pose_abs_2);
cv::Mat handeyeBootstratpTranslationCalibration(
    unsigned int nb_cluster, unsigned int nb_it,
    const std::vector<cv::Mat> &pose_abs_1,
    const std::vector<cv::Mat> &pose_abs_2);

cv::Mat convertRotationMatrixToQuaternion(cv::Mat R);
cv::Mat convertQuaternionToRotationMatrix(const std::array<double, 4> &q);

cv::Mat calcAverageRotation(std::vector<double> &r1, std::vector<double> &r2,
                            std::vector<double> &r3,
                            bool useQuaternionAverage = true);

void calcAverageRotation2(cv::InputArrayOfArrays rvecs,
                          cv::OutputArray averageRotation,
                          bool useQuaternionAverage = true);

cv::Mat calcAveragePose(const std::vector<cv::Mat> &poses,
                        bool useQuaternionAverage = true);

} // namespace tl::mcmb
