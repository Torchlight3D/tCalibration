#include "Quad.h"

namespace tl {
namespace stag {

Quad::Quad(const std::array<cv::Point2d, 4> &_corners) : corners(_corners)
{
    calcLineAtInfinity();
    calculateProjectiveDistortion();
}

Quad::Quad(const Quad &o)
{
    corners = o.corners;
    lineInf = o.lineInf;
    projectiveDistortion = o.projectiveDistortion;
    H = o.H.clone();
    center = o.center;
}

Quad::~Quad() = default;

void Quad::estimateHomography()
{
    // apply affine rectification to the corners
    std::array<cv::Point2d, 4> affineCorners;
    for (int i = 0; i < 4; i++) {
        affineCorners[i] =
            cv::Point2d(corners[i].x / (lineInf.x * corners[i].x +
                                        lineInf.y * corners[i].y + lineInf.z),
                        corners[i].y / (lineInf.x * corners[i].x +
                                        lineInf.y * corners[i].y + lineInf.z));
    }

    cv::Mat HarInv = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat Haffsim = cv::Mat::eye(3, 3, CV_64FC1);

    // inverse of affine rectification
    HarInv.at<double>(2, 0) = -lineInf.x / lineInf.z;
    HarInv.at<double>(2, 1) = -lineInf.y / lineInf.z;
    HarInv.at<double>(2, 2) = 1 / lineInf.z;

    // find the affine transformation from square to affine rectified quad
    Haffsim.at<double>(0, 0) = affineCorners[1].x - affineCorners[0].x;
    Haffsim.at<double>(0, 1) = affineCorners[3].x - affineCorners[0].x;
    Haffsim.at<double>(0, 2) = affineCorners[0].x;
    Haffsim.at<double>(1, 0) = affineCorners[1].y - affineCorners[0].y;
    Haffsim.at<double>(1, 1) = affineCorners[3].y - affineCorners[0].y;
    Haffsim.at<double>(1, 2) = affineCorners[0].y;

    // product of these transformations is the homography
    H = HarInv * Haffsim;

    // locate the projection of the center of the marker
    cv::Mat origCenter(3, 1, CV_64FC1);
    origCenter.at<double>(0) = 0.5;
    origCenter.at<double>(1) = 0.5;
    origCenter.at<double>(2) = 1;

    origCenter = H * origCenter;
    center.x = origCenter.at<double>(0) / origCenter.at<double>(2);
    center.y = origCenter.at<double>(1) / origCenter.at<double>(2);
}

void Quad::calcLineAtInfinity()
{
    // vectors going from one corner to another
    const auto vec23 = corners[1] - corners[2];
    const auto vec14 = corners[0] - corners[3];
    const auto vec34 = corners[2] - corners[3];
    const auto vec12 = corners[0] - corners[1];

    const auto pair1 = vec14.cross(vec23);
    const auto pair2 = vec12.cross(vec34);
    const auto parallel1 = pair1 == 0;
    const auto parallel2 = pair2 == 0;

    // If both edge pairs are parallel
    if (parallel1 && parallel2) {
        lineInf = {0., 0., 1.};
        return;
    }

    // cross products of corners (i.e. lines representing the edges)
    double cross14 = corners[0].cross(corners[3]);
    double cross23 = corners[1].cross(corners[2]);
    double cross12 = corners[0].cross(corners[1]);
    double cross34 = corners[2].cross(corners[3]);
    // intersection points at the vanishing line
    cv::Point2d inters1, inters2;
    // if one edge pair is parallel
    if (parallel1) {
        inters2 = (cross12 * vec34 - cross34 * vec12) / pair2;

        // this intersection is not real. doing this to find the equation of the
        // line with only one point.
        inters1 = inters2 + vec14;
    }
    // if the other edge pair is parallel
    else if (parallel2) {
        inters1 = (cross14 * vec23 - cross23 * vec14) / pair1;
        inters1.y = (cross14 * vec23.y - vec14.y * cross23) / pair1;

        // this intersection is not real. doing this to find the equation of the
        // line with only one point.
        inters2 = inters1 + vec12;
    }
    // if neither pairs are parallel
    else {
        inters1 = (cross14 * vec23 - cross23 * vec14) / pair1;
        inters2 = (cross12 * vec34 - cross34 * vec12) / pair2;
    }

    // find the vanishing line in homogeneous coordinates
    // l = P1 x P2
    double l1 = inters1.y - inters2.y;
    double l2 = inters2.x - inters1.x;
    double l3 = inters1.x * inters2.y - inters2.x * inters1.y;

    // normalize using
    // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html
    // (13)
    double normalizer = std::sqrt(l1 * l1 + l2 * l2);
    l1 /= normalizer;
    l2 /= normalizer;
    l3 /= normalizer;

    lineInf = cv::Point3d(l1, l2, l3);
}

void Quad::calculateProjectiveDistortion()
{
    // find the minimum and maximum distance from corners to the vanishing line
    // projective distortion = maxDist / minDist
    double curDist = std::abs(lineInf.x * corners[0].x +
                              lineInf.y * corners[0].y + lineInf.z);

    double minDist = curDist;
    double maxDist = curDist;
    for (int i = 1; i < 4; i++) {
        curDist = std::abs(lineInf.x * corners[i].x + lineInf.y * corners[i].y +
                           lineInf.z);

        minDist = std::min(minDist, curDist);
        maxDist = std::max(maxDist, curDist);
    }
    projectiveDistortion = maxDist / minDist;
}

} // namespace stag
} // namespace tl
