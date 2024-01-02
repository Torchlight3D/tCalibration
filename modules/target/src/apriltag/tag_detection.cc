#include "tag_detection.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
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

namespace apriltags {

TagDetection::TagDetection() : TagDetection(0) {}

TagDetection::TagDetection(int _id)
    : homography(),
      p(),
      cxy(),
      hxy(),
      observedPerimeter(),
      obsCode(),
      code(),
      id(_id),
      hammingDistance(),
      rotation(),
      good(false)
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
    float orient = atan2(p1.y - p0.y, p1.x - p0.x);
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

Eigen::Matrix4d TagDetection::getRelativeTransform(double tag_size, double fx,
                                                   double fy, double px,
                                                   double py) const
{
    const double half_tag_size = tag_size / 2.;

    std::vector<cv::Point3d> obj_pts;
    obj_pts.reserve(4);
    obj_pts.emplace_back(-half_tag_size, -half_tag_size, 0.);
    obj_pts.emplace_back(half_tag_size, -half_tag_size, 0.);
    obj_pts.emplace_back(half_tag_size, half_tag_size, 0.);
    obj_pts.emplace_back(-half_tag_size, half_tag_size, 0.);

    std::vector<cv::Point2d> img_pts;
    img_pts.reserve(4);
    img_pts.push_back(cv::Point2d(p[0]));
    img_pts.push_back(cv::Point2d(p[1]));
    img_pts.push_back(cv::Point2d(p[2]));
    img_pts.push_back(cv::Point2d(p[3]));

    cv::Matx33d K{fx, 0., px, 0., fy, py, 0., 0., 1.};
    cv::Vec4d distortion; // no distortion?
    cv::Mat rvec, tvec;
    cv::solvePnP(obj_pts, img_pts, K, distortion, rvec, tvec);

    cv::Matx33d rmat;
    cv::Rodrigues(rvec, rmat);

    Eigen::Matrix3d wRo;
    cv::cv2eigen(rmat, wRo);

    Eigen::Matrix4d T;
    T.topLeftCorner(3, 3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1),
        tvec.at<double>(2);
    T.row(3) << 0, 0, 0, 1;

    return T;
}

void TagDetection::getRelativeTranslationRotation(double tag_size, double fx,
                                                  double fy, double px,
                                                  double py,
                                                  Eigen::Vector3d& tvec,
                                                  Eigen::Matrix3d& rmat) const
{
    Eigen::Matrix4d T = getRelativeTransform(tag_size, fx, fy, px, py);

    // camera frame -> object frame
    // (z forward, x right, y down) -> (x forward, y left, z up)
    Eigen::Matrix4d M;
    // clang-format off
    M << 0.,  0., 1., 0.,
        -1.,  0., 0., 0.,
         0., -1., 0., 0.,
         0.,  0., 0., 1.;
    // clang-format on

    Eigen::Matrix4d MT = M * T;
    // translation vector from camera to the April tag
    tvec = MT.col(3).head(3);
    // orientation of April tag with respect to camera: the camera
    // convention makes more sense here, because yaw,pitch,roll then
    // naturally agree with the orientation of the object
    rmat = T.block(0, 0, 3, 3);
}

void TagDetection::draw(cv::Mat& image) const
{
    const auto& p1 = p[0];
    const auto& p2 = p[1];
    const auto& p3 = p[2];
    const auto& p4 = p[3];

    // Plot outline
    cv::line(image, cv::Point(p1), cv::Point(p2), CV_RGB(255, 0, 0));
    cv::line(image, cv::Point(p2), cv::Point(p3), CV_RGB(0, 255, 0));
    cv::line(image, cv::Point(p3), cv::Point(p4), CV_RGB(0, 0, 255));
    cv::line(image, cv::Point(p4), cv::Point(p1), CV_RGB(255, 0, 255));

    // Plot center
    constexpr int kRadius{5};
    constexpr int kThickness{2};
    cv::circle(image, cv::Point(cxy), kRadius, CV_RGB(0, 0, 255), kThickness);

    // Print Id
    constexpr double kFontScale{0.8};
    std::ostringstream oss;
    oss << "#" << id;
    cv::putText(image, oss.str(), cv::Point(cxy + cv::Point2f{10.f, 10.f}),
                cv::FONT_HERSHEY_PLAIN, kFontScale, CV_RGB(0, 0, 255));
}

} // namespace apriltags
