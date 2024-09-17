#include "Marker.h"

#include <opencv2/imgproc.hpp>

namespace tl {
namespace stag {

Marker::Marker(const Quad &q, int inId)
{
    corners = q.corners;
    lineInf = q.lineInf;
    projectiveDistortion = q.projectiveDistortion;
    H = q.H.clone();
    center = q.center;

    id = inId;
    C = cv::Mat(1, 1, CV_64FC1);
}

Marker::Marker(const Marker &m)
{
    corners = m.corners;
    lineInf = m.lineInf;
    projectiveDistortion = m.projectiveDistortion;
    H = m.H.clone();
    center = m.center;

    id = m.id;
    C = m.C;
}

void Marker::shiftCorners2(int shift)
{
    if (shift == 1) {
        auto temp = corners[0];
        corners[0] = corners[1];
        corners[1] = corners[2];
        corners[2] = corners[3];
        corners[3] = temp;
    }
    else if (shift == 2) {
        auto temp1 = corners[0];
        auto temp2 = corners[1];
        corners[0] = corners[2];
        corners[1] = corners[3];
        corners[2] = temp1;
        corners[3] = temp2;
    }
    else if (shift == 3) {
        auto temp = corners[0];
        corners[0] = corners[3];
        corners[3] = corners[2];
        corners[2] = corners[1];
        corners[1] = temp;
    }
    else {
        return;
    }

    // have to recalculate homography after shift
    estimateHomography();
}

bool Marker::isSimilarIn(const std::vector<Marker> &markers) const
{
    for (const auto &marker : markers) {
        double dist = cv::norm(center - marker.center);

        // convert corners from double to float
        std::vector<cv::Point2f> fcorners1(corners.size());
        std::vector<cv::Point2f> fcorners2(marker.corners.size());
        cv::Mat(corners).copyTo(fcorners1);
        cv::Mat(marker.corners).copyTo(fcorners2);

        // calculate difference between areas
        double area1 = cv::contourArea(fcorners1);
        double area2 = cv::contourArea(fcorners2);
        double areaMean = (area1 + area2) / 2;
        double distArea = std::abs(area1 - area2);

        // Markers are similar if
        // 1. Ids are identical
        // 2. Distance between centers is less than ??? percent of mean area
        // 3. Difference of areas is less than ??? of mean area
        constexpr auto kMaxCenterDistanceRatio{2e-3};
        constexpr auto kMaxAreaDiffRatio{0.25};
        if (id == marker.id && dist < areaMean * kMaxCenterDistanceRatio &&
            distArea < areaMean * kMaxAreaDiffRatio) {
            return true;
        }
    }
    return false;
}

} // namespace stag
} // namespace tl
