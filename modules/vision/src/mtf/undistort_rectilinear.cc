#include "undistort_rectilinear.h"
#include "laguerre_roots.h"

namespace {

double laguerre_smallest_positive_root(double ru, double k1, double k2)
{
    std::vector<cplex> a = {ru, -1.0, ru * k1, 0.0, ru * k2};
    std::vector<cplex> roots(4);
    lroots(a, roots);

    double minRoot = std::numeric_limits<double>::max();
    for (const auto& root : roots) {
        if (root.imag() == 0 && root.real() >= 0 && root.real() < minRoot) {
            minRoot = root.real();
        }
    }

    // try to refine the root
    cplex root(minRoot, 0.0);
    int its;
    laguerre(a, root, its);

    return root.real();
}

bool lagsolve(double ru, double k1, double k2, double& root)
{
    // we are looking for the roots of
    // P(r) = ru*k2*root^4 + ru*k1*root^2 - root + ru = 0
    // we would prefer the smallest positive root

    if (std::abs(k1) < 1e-8 && std::abs(k2) < 1e-8) {
        root = ru;
    }
    else {
        if (std::abs(k2) < 1e-8) { // we only have a quadratic
            // a == 1
            double b = -1.0 / (k1 * ru);
            double c = 1.0 / k1;
            double q = -0.5 * (b + std::copysign(sqrt(b * b - 4 * c), b));
            // force negative roots to become very large
            double r1 = q;
            double r2 = c / q;
            if (r1 < 0) {
                root = r2;
            }
            else {
                if (r2 < 0) {
                    root = r1;
                }
                else {
                    root = std::min(r1, r2);
                }
            }
        }
        else {
            root = laguerre_smallest_positive_root(ru, k1, k2);
        }
    }

    return true;
}

} // namespace

Undistort_rectilinear::Undistort_rectilinear(
    const cv::Rect& r, const std::vector<double>& in_coeffs)
    : Undistort(r)
{
    for (size_t i = 0; i < 2; i++) {
        coeffs_[i] = in_coeffs[i];
    }

    radius_norm_ = cv::norm(centre);
}

cv::Point2d Undistort_rectilinear::slow_transform_point(
    const cv::Point2d& point) const
{
    const auto pt = (point + offset - centre);
    double ru = cv::norm(pt) / radius_norm_;

    if (ru == 0.) {
        return point;
    }

    auto rd{0.};
    // not the fastest method, but it should be robust
    if (bool s = lagsolve(ru, coeffs_[0], coeffs_[1], rd); !s) {
        return point;
    }

    return pt * (rd / ru) + centre - offset;
}

cv::Point2d Undistort_rectilinear::inverse_transform_point(
    const cv::Point2d& point) const
{
    const auto pt = (point + offset - centre);
    double rd = cv::norm(pt) / radius_norm_;

    if (rd == 0.) {
        return centre - offset;
    }

    double r2 = rd * rd;
    double ru = 1. + (coeffs_[0] + coeffs_[1] * r2) * r2;

    return pt / ru + centre - offset;
}

// note: second parameter is the raw Bayer image, which must also be padded out
cv::Mat Undistort_rectilinear::unmap(const cv::Mat& in_src, cv::Mat& rawimg)
{
    constexpr double kBufferRatio = 0.025;

    const auto maxDim = std::max(in_src.cols, in_src.rows);

    auto pi = inverse_transform_point(
        centre - cv::Point2d{max_val.x + kBufferRatio * maxDim, 0.});
    int pad_left = pi.x < 0. ? std::ceil(-pi.x) : 0;
    pi = inverse_transform_point(
        centre - cv::Point2d{0., max_val.y + kBufferRatio * maxDim});
    int pad_top = pi.y < 0. ? std::ceil(-pi.y) : 0;

    return unmap_base(in_src, rawimg, pad_left, pad_top);
}
