#include "Marker.h"

#include <opencv2/imgproc.hpp>

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
        cv::Point2d t = corners[0];
        corners[0] = corners[1];
        corners[1] = corners[2];
        corners[2] = corners[3];
        corners[3] = t;
    }
    else if (shift == 2) {
        cv::Point2d t1 = corners[0];
        cv::Point2d t2 = corners[1];
        corners[0] = corners[2];
        corners[1] = corners[3];
        corners[2] = t1;
        corners[3] = t2;
    }
    else if (shift == 3) {
        cv::Point2d t = corners[0];
        corners[0] = corners[3];
        corners[3] = corners[2];
        corners[2] = corners[1];
        corners[1] = t;
    }
    else
        return;

    // have to recalculate homography after shift
    estimateHomography();
}

bool Marker::isSimilarIn(std::vector<Marker> collection)
{
    for (auto &marker : collection) {
        // calculate distance between centers
        double dist = norm(center - marker.center);

        // convert corners from double to float
        std::vector<cv::Point2f> fcorners1(corners.size());
        std::vector<cv::Point2f> fcorners2(marker.corners.size());
        cv::Mat(corners).copyTo(fcorners1);
        cv::Mat(marker.corners).copyTo(fcorners2);

        // calculate difference between areas
        double area1 = cv::contourArea(fcorners1);
        double area2 = cv::contourArea(fcorners2);
        double areaMean = (area1 + area2) / 2;
        double distArea = fabs(area1 - area2);

        // markers are similar if
        // - ids are identical
        // - distance between centers is less than 0.2% of mean area
        // - difference of areas is less than 25% of mean area
        if (id == marker.id && dist < areaMean * 0.002 &&
            distArea < areaMean * 0.25) {
            return true;
        }
    }
    return false;
}
