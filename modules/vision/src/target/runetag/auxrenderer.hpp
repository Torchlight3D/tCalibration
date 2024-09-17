#pragma once

#include <opencv2/core/mat.hpp>

#include "markerpose.hpp"

namespace tl {
namespace runetag {

class EllipsePoint;
class MarkerDetected;
class Slot;

template <typename T>
std::string toStr(T val)
{
    std::stringstream ss;
    ss << val;
    return ss.str();
}

void drawEllipsePoint(cv::Mat frame, const EllipsePoint& e,
                      const cv::Mat& intrinsics, cv::Scalar color);
void drawEllipse(cv::Mat& frame, const cv::RotatedRect& el,
                 const cv::Mat& intrinsics);
void drawEllipse(cv::Mat frame, const cv::Mat& e, const cv::Mat& intrinsics,
                 cv::Scalar color);
void fillEllipse(cv::Mat frame, const cv::Mat& e, const cv::Mat& intrinsics,
                 cv::Scalar color);
void drawPoint(cv::Mat& frame, const cv::Point2f p, const cv::Mat& intrinsics,
               cv::Scalar color);
void drawPoint(cv::Mat& frame, const cv::Point2f p, const cv::Mat& intrinsics);
void drawVector(cv::Mat& frame, const cv::Point2f center,
                const cv::Point2f vector, cv::Scalar color,
                const cv::Mat& intrinsics);
void drawLine(cv::Mat& frame, const cv::Point2f p1, const cv::Point2f p2,
              cv::Scalar color, const cv::Mat& intrinsics);

void fillSlot(cv::Mat& frame, const Slot& slot, cv::Scalar color,
              const cv::Mat& intrinsics);

void drawDetectedMarker(cv::Mat& frame, const MarkerDetected& m,
                        const cv::Mat& intrinsics);

void drawDetectedMarker3D(cv::Mat& frame, const MarkerDetected& m,
                          const Pose& p, const cv::Mat& intrinsics,
                          const cv::Mat& distortion,
                          cv::Scalar color = cv::Scalar(255, 0, 0));

void drawDetectedMarker3DCylinder(cv::Mat& frame, const MarkerDetected& m,
                                  const Pose& p, const cv::Mat& intrinsics,
                                  const cv::Mat& distortion, cv::Scalar color);

void drawDetectedMarker3DCylinder2(cv::Mat& frame, const MarkerDetected& m,
                                   const Pose& p, const cv::Mat& intrinsics,
                                   const cv::Mat& distortion);

void drawDetectedMarker3DSlots(cv::Mat& frame, const MarkerDetected& m,
                               const Pose& p, const cv::Mat& intrinsics,
                               const cv::Mat& distortion,
                               cv::Scalar color = cv::Scalar(255, 0, 0));
void drawDetectedMarker3Dfits(cv::Mat& frame, const MarkerDetected& m,
                              unsigned int slot1, unsigned int slot2,
                              unsigned int real_slot1, unsigned int real_slot2,
                              const Pose& p, const cv::Mat& intrinsics,
                              const cv::Mat& distortion);

} // namespace runetag
} // namespace tl
