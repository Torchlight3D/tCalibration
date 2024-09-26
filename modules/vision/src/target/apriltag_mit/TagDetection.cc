#include "TagDetection.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#ifdef PLATFORM_APERIOS
// missing/broken isnan
namespace std {
static bool isnan(float x)
{
    const int EXP = 0x7f800000;
    const int FRAC = 0x007fffff;
    const int y = *((int*)(&x));
    return ((y & EXP) == EXP && (y & FRAC) != 0);
}
} // namespace std
#endif

namespace AprilTags {

using Eigen::Matrix3d;
using Eigen::Matrix4d;

TagDetection::TagDetection()
    : good(false),
      obsCode(),
      code(),
      id(),
      hammingDistance(),
      rotation(),
      p(),
      cxy(),
      observedPerimeter(),
      homography(),
      hxy()
{
    homography.setZero();
}

TagDetection::TagDetection(int _id)
    : good(false),
      obsCode(),
      code(),
      id(_id),
      hammingDistance(),
      rotation(),
      p(),
      cxy(),
      observedPerimeter(),
      homography(),
      hxy()
{
    homography.setZero();
}

float TagDetection::getXYOrientation() const
{
    // Because the order of segments in a quad is arbitrary, so is the
    // homography's rotation, so we can't determine orientation directly
    // from the homography.  Instead, use the homography to find two
    // bottom corners of a properly oriented tag in pixel coordinates,
    // and then compute orientation from that.
    const auto p0 = interpolate(-1, -1); // lower left corner of tag
    const auto p1 = interpolate(1, -1);  // lower right corner of tag
    float orient = std::atan2(p1.y - p0.y, p1.x - p0.x);
    return !std::isnan(float(orient)) ? orient : 0.;
}

cv::Point2f TagDetection::interpolate(float x, float y) const
{
    float z = homography(2, 0) * x + homography(2, 1) * y + homography(2, 2);
    if (z == 0) {
        return {};
    }

    float newx =
        (homography(0, 0) * x + homography(0, 1) * y + homography(0, 2)) / z +
        hxy.x;
    float newy =
        (homography(1, 0) * x + homography(1, 1) * y + homography(1, 2)) / z +
        hxy.y;
    return {newx, newy};
}

bool TagDetection::overlapsTooMuch(const TagDetection& other) const
{
    // Compute a sort of "radius" of the two targets. We'll do this by
    // computing the average length of the edges of the quads (in
    // pixels).
    float radius =
        (cv::norm(p[0] - p[1]) + cv::norm(p[1] - p[2]) + cv::norm(p[2] - p[3]) +
         cv::norm(p[3] - p[0]) + cv::norm(other.p[0] - other.p[1]) +
         cv::norm(other.p[1] - other.p[2]) + cv::norm(other.p[2] - other.p[3]) +
         cv::norm(other.p[3] - other.p[0])) /
        16.f;

    // distance (in pixels) between two tag centers
    float dist = cv::norm(cxy - other.cxy);

    // reject pairs where the distance between centroids is smaller than
    // the "radius" of one of the tags.
    return (dist < radius);
}

Eigen::Matrix4d TagDetection::getRelativeTransform(double tagSize, double fx,
                                                   double fy, double px,
                                                   double py) const
{
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;
    double s = tagSize / 2.;
    objPts.push_back(cv::Point3f(-s, -s, 0));
    objPts.push_back(cv::Point3f(s, -s, 0));
    objPts.push_back(cv::Point3f(s, s, 0));
    objPts.push_back(cv::Point3f(-s, s, 0));

    imgPts.push_back(p[0]);
    imgPts.push_back(p[1]);
    imgPts.push_back(p[2]);
    imgPts.push_back(p[3]);

    const cv::Matx33f cameraMatrix(fx, 0, px, 0, fy, py, 0, 0, 1);
    const cv::Vec4f distParam(0, 0, 0, 0);

    cv::Mat rvec, tvec;
    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);

    cv::Matx33d r;
    cv::Rodrigues(rvec, r);
    Matrix3d wRo;
    cv::cv2eigen(r, wRo);

    Matrix4d T;
    T.topLeftCorner(3, 3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1),
        tvec.at<double>(2);
    T.row(3) << 0, 0, 0, 1;

    return T;
}

void TagDetection::getRelativeTranslationRotation(double tag_size, double fx,
                                                  double fy, double px,
                                                  double py,
                                                  Eigen::Vector3d& trans,
                                                  Eigen::Matrix3d& rot) const
{
    Eigen::Matrix4d T = getRelativeTransform(tag_size, fx, fy, px, py);

    // converting from camera frame (z forward, x right, y down) to
    // object frame (x forward, y left, z up)
    // clang-format off
    Matrix4d M;
    M << 0.,  0., 1., 0.,
        -1.,  0., 0., 0.,
         0., -1., 0., 0.,
         0.,  0., 0., 1.;
    // clang-format on

    Matrix4d MT = M * T;
    // translation vector from camera to the April tag
    trans = MT.col(3).head(3);
    // orientation of April tag with respect to camera: the camera
    // convention makes more sense here, because yaw,pitch,roll then
    // naturally agree with the orientation of the object
    rot = T.block(0, 0, 3, 3);
}

// draw one April tag detection on actual image
void TagDetection::draw(cv::Mat& image) const
{
    // use corner points detected by line intersection

    // plot outline
    cv::line(image, p[0], p[1], cv::Scalar(255, 0, 0, 0));
    cv::line(image, p[1], p[2], cv::Scalar(0, 255, 0, 0));
    cv::line(image, p[2], p[3], cv::Scalar(0, 0, 255, 0));
    cv::line(image, p[3], p[0], cv::Scalar(255, 0, 255, 0));

    // mark center
    cv::circle(image, cxy, 8, cv::Scalar(0, 0, 255, 0), 2);

    // print ID
    cv::putText(image, std::format("#{}", id), cxy + cv::Point2f{10.f, 10.f},
                cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
}

} // namespace AprilTags
