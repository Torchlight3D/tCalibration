#pragma once

#include <opencv2/core/mat.hpp>

namespace cvtools {
cv::Mat plotScatterXY(const std::vector<cv::Point2d> &P, const cv::Size size,
                      double &minX, double &maxX, double &minY);
cv::Vec3f applyHomTransform(const cv::Matx44f &T, const cv::Vec3f &p);
cv::Vec2f applyHomTransform(const cv::Matx33f &T, const cv::Vec2f &p);
std::vector<cv::Point2f> applyHomTransform(const cv::Matx33f &T,
                                           const std::vector<cv::Point2f> &ps);
std::vector<size_t> findNearestNeighborsAngleSorted(
    const cv::KeyPoint queryPoint, const std::vector<cv::KeyPoint> searchPoints,
    int N);
cv::Mat fitHomography(std::vector<cv::Point2f> q1, std::vector<cv::Point2f> q2,
                      float &ssd);
bool findPartialCirclesGrid(const cv::Mat &im, std::vector<cv::Point2f> &q,
                            std::vector<cv::Point3f> &Q, float circleSpacing);
void phaseCorrelate(const cv::Mat &im1, const cv::Mat &im2, float &scale,
                    float &angle, cv::Point2f &shift);
cv::Mat logPolar(const cv::Mat &image, float scale);
void initDistortMap(const cv::Matx33f cameraMatrix,
                    const cv::Vec<float, 5> distCoeffs, const cv::Size size,
                    cv::Mat &map1, cv::Mat &map2);

// Downsample a texture which was created in virtual column/row space for a
// diamond pixel array projector
void diamondDownsample(cv::InputArray src, cv::OutputArray dst);

cv::Mat histimage(cv::Mat histogram);

} // namespace cvtools
