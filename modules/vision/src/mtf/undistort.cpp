#include "undistort.h"

#include <format>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

Undistort::Undistort(const cv::Rect &r)
    : centre(r.width / 2, r.height / 2),
      offset(r.x, r.y),
      radmap(0),
      max_val(1, 1),
      last_padding(0, 0)
{
}

cv::Point Undistort::transform_pixel(int col, int row)
{
    cv::Point2d tp = transform_point(double(col), double(row));
    return cv::Point(lrint(tp.x), lrint(tp.y));
}

// find closest point in radmap lookup table, then apply
// quadratic interpolation to refine the value
cv::Point2d Undistort::transform_point(double c, double r)
{
    double px = (c + offset.x - centre.x);
    double py = (r + offset.y - centre.y);

    double rad = 2 * sqrt(px * px + py * py); // convert to half-pixel spacing
                                              // to match how radmap was built
    int rad_f = (int)rad;

    if (rad_f > (int)radmap.size() - 3) {
        // this should only happen if we call transform_point before
        // unmap, which should not happen
        if (radmap.empty()) {
            LOG(ERROR) << "Warning: radmap not initialized in transform_point. "
                          "Calling build_radmap, but fix the code!";
            build_radmap();
        }
        else {
            LOG(ERROR) << std::format(
                "Warning: radmap range exceeded in transform_point. "
                "Clamping (rad-{}, rad_f={})",
                rad, rad_f);
            rad_f = (int)radmap.size() - 3;
        }
    }

    double w = rad - rad_f;
    // quadratic interpolation through half-pixel spaced points
    double fc = radmap[rad_f];
    double fb =
        -1.5 * radmap[rad_f] + 2 * radmap[rad_f + 1] - 0.5 * radmap[rad_f + 2];
    double fa =
        0.5 * radmap[rad_f] - radmap[rad_f + 1] + 0.5 * radmap[rad_f + 2];
    double rad_d = 2 * (fa * w * w + fb * w + fc);

    // avoid divide-by-zero
    if (rad < 1e-8) {
        return centre;
    }

    return cv::Point2d(px, py) * (rad_d / rad) + centre - offset;
}

void Undistort::build_radmap()
{
    radmap.clear();
    double maxrad =
        1.1 *
        sqrt(centre.x * centre.x +
             centre.y *
                 centre.y); // TODO: add better support for non-centered CoD
    for (int rad = 0; rad <= 2 * (maxrad + 2); rad++) { // half-pixel spacing
        cv::Point2d tp = slow_transform_point(centre.x + 0.5 * rad, centre.y);
        double rd = norm(tp - cv::Point2d(centre.x, centre.y));
        radmap.push_back(rd);
    }
}

cv::Mat Undistort::unmap_base(const cv::Mat &in_src, cv::Mat &rawimg,
                              int pad_left, int pad_top)
{
    // to preserve Bayer CFA alignment
    pad_left += pad_left % 2;
    pad_top += pad_top % 2;

    cv::Mat src;
    cv::copyMakeBorder(in_src, src, pad_top, pad_top, pad_left, pad_left,
                       cv::BORDER_CONSTANT, cv::Scalar::all(0));
    centre.x = src.cols / 2;
    centre.y = src.rows / 2;

    LOG(INFO) << std::format(
        "Padding with {} left/right pixels, {} top/bottom pixels", pad_left,
        pad_top);

    if (pad_left > 0 || pad_top > 0) {
        LOG(INFO) << "Padding distorted/Bayer image.";
        cv::Mat rcopy = rawimg.clone();
        cv::copyMakeBorder(rcopy, rawimg, pad_top, pad_top, pad_left, pad_left,
                           cv::BORDER_CONSTANT, cv::Scalar::all(0));
        last_padding = cv::Point(pad_left, pad_top);
    }

    build_radmap();

    cv::Mat map_x(src.rows, src.cols, CV_32FC1);
    cv::Mat map_y(src.rows, src.cols, CV_32FC1);

    cv::Mat blurred;
    cv::blur(src, blurred, cv::Size(3, 3));

    cv::Point2d prev;
    for (int r = 0; r < src.rows; r++) {
        prev = transform_point(-1, r);
        for (int c = 0; c < src.cols; c++) {
            cv::Point2d tp = transform_point(c, r);
            map_x.at<float>(r, c) = tp.x;
            map_y.at<float>(r, c) = tp.y;

            // add some blurring to suppress noise before magnification
            // this avoids the edges becoming so rough that the rectangle
            // test on the undistorted image fails
            double stretch = norm(prev - tp);
            if (stretch > 0.9) {
                blurred.at<uint16_t>(r, c) = src.at<uint16_t>(r, c);
            }
            else {
                // 0.9 -> src
                // 0.4 -> blurred
                double w = stretch < 0.4 ? 0 : 2 * (stretch - 0.4);
                blurred.at<uint16_t>(r, c) =
                    w * src.at<uint16_t>(r, c) +
                    (1 - w) * blurred.at<uint16_t>(r, c);
            }
            prev = tp;
        }
    }
    cv::Mat timg;
    cv::remap(blurred, timg, map_x, map_y, cv::INTER_LINEAR,
              cv::BORDER_CONSTANT, cv::Scalar::all(0));

    return timg;
}

void Undistort::estimate_padding(const cv::Mat &src, int &pad_left,
                                 int &pad_top)
{
    // this method scans along the longest edge of the image to see if an edge
    // of reasonable length, say 80 pixels will map to a marginal-length edge
    // (say 20 pixels) this becomes the cut-off on the longest edge, which also
    // defines the short edge cut-off

    const double src_w = 80;  // pixels
    const double dest_w = 20; // do not try to expand beyond this

    cv::Point2d extreme = inverse_transform_point(1, 1);

    if (!allow_crop) {
        pad_left = extreme.x < 0 ? -ceil(extreme.x) : 0;
        pad_top = extreme.y < 0 ? -ceil(extreme.y) : 0;
        LOG(INFO) << std::format("extreme: {}, {}. pad_left={}, pad_top={}",
                                 extreme.x, extreme.y, pad_left, pad_top);
        return;
    }

    cv::Point2d sdir(1, 0);
    if (src.rows > src.cols) {
        sdir = cv::Point2d(0, 1);
    }

    const double rad_min = centre.ddot(sdir);
    double rad_max = centre.ddot(sdir) - extreme.ddot(sdir);
    cv::Point2d dir = centre * (1.0 / norm(centre));
    double rad = rad_min;
    // rather sweep along the top and left edges (but still measure radially?)
    for (; rad <= rad_max; rad += 10.0) {
        cv::Point2d p;
        if (sdir.x == 1.0) {
            p = cv::Point2d(centre.x - rad, 0);
        }
        else {
            p = cv::Point2d(0, centre.y - rad);
        }
        cv::Point2d p2 = p - src_w * dir;

        cv::Point2d tp = slow_transform_point(p.x, p.y);
        cv::Point2d tp2 = slow_transform_point(p2.x, p2.y);
        double dist = norm(tp2 - tp);
        if (dist < dest_w)
            break;
    }
    if (sdir.x == 1.0) {
        pad_left = std::max(-(centre.x - rad - src_w), 0.0);
        cv::Point2d fwd = slow_transform_point(-pad_left, 0);
        pad_top = fwd.y + src_w;
    }
    else {
        pad_top = std::max(-(centre.y - rad - src_w), 0.0);
        cv::Point2d fwd = slow_transform_point(0, -pad_top);
        pad_left = fwd.x + src_w;
    }
}

void Undistort::apply_padding(std::vector<cv::Mat> &images)
{
    if (last_padding.x > 0 || last_padding.y > 0) {
        for (const auto &image : images) {
            cv::Mat rcopy = image.clone();
            cv::copyMakeBorder(rcopy, image, last_padding.y, last_padding.y,
                               last_padding.x, last_padding.x,
                               cv::BORDER_CONSTANT, cv::Scalar::all(0));
        }
    }
}

bool Undistort::rectilinear_equivalent() const { return rectilinear; }

void Undistort::set_rectilinear_equivalent(bool b) { rectilinear = b; }

void Undistort::set_max_val(const cv::Point2d &maxv) { max_val = maxv; }

void Undistort::set_allow_crop(bool crop) { allow_crop = crop; }
