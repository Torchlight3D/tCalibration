#include "autocrop.h"

#include <format>

#include <glog/logging.h>

namespace {
constexpr int kBorder = 150;
}

AutoCropper::AutoCropper(const cv::Mat& img) : Cropper(img)
{
    // Init row and col bound
    for (int r = 4; r < img.rows - 4; r += 2) {
        for (int c = 4; c < img.cols - 4; c += 2) {
            int_rs[r] += img.at<uint16_t>(r, c) / 65536.0;
            int_cs[c] += img.at<uint16_t>(r, c) / 65536.0;

            int_rs[r + 1] = int_rs[r];
            int_cs[c + 1] = int_cs[c];
        }
    }

    const auto bounds_r = calcOtsuBound(int_rs, kBorder);
    rstart_ = bounds_r.x;
    height_ = bounds_r.y;

    const auto bounds_c = calcOtsuBound(int_cs, kBorder);
    cstart_ = bounds_c.x;
    width_ = bounds_c.y;

    LOG(INFO) << std::format("start r/c: [{} {}], h/w: [{} {}]", rstart_,
                             cstart_, height_, width_);
}

cv::Mat AutoCropper::subset(const cv::Mat& X, cv::Rect* dimensions) const
{
    if (dimensions) {
        // what to add to cropped image coordinates to obtain original
        // coordinates
        dimensions->x = cstart_;
        dimensions->y = rstart_;
        // original width and height
        dimensions->width = X.cols;
        dimensions->height = X.rows;
    }

    cv::Mat Y;
    X(cv::Rect(cstart_, rstart_, width_, height_)).copyTo(Y);
    return Y;
}

cv::Point AutoCropper::calcOtsuBound(const std::vector<double>& data,
                                     int border) const
{
    double otsu = calcOtsuThreshold(data);

    // if the left or right edge is above the threshold, then we probably have a
    // bright edge so we have to re-estimate the brightness threshold after
    // compensating for that
    int dead_left = 0;
    while (data[dead_left] < otsu && dead_left < int(data.size() / 4)) {
        dead_left++;
    }
    if (dead_left < 0.1 * data.size()) {
        while (data[dead_left] > otsu && dead_left < int(data.size() / 4))
            dead_left++;
    }
    else {
        dead_left = 0;
    }

    int dead_right = data.size() - 1;
    while (data[dead_right] < otsu && dead_right > int(3 * data.size() / 4))
        dead_right--;
    if (dead_right > 0.9 * data.size()) {
        while (data[dead_right] > otsu && dead_right > int(3 * data.size() / 4))
            dead_right--;
    }
    else {
        dead_right = data.size() - 1;
    }

    if (dead_left > 0 || dead_right < (int)data.size() - 1) {
        std::vector<double> dcopy(data);
        std::sort(dcopy.begin(), dcopy.end());
        double median = dcopy[dcopy.size() / 2];
        // replace values above threshold with median?
        dcopy = data;
        for (auto& d : dcopy) {
            if (d > otsu) {
                d = median;
            }
        }
        otsu = calcOtsuThreshold(dcopy);
        LOG(INFO) << "Autocrop had to use a deadzone because the image borders "
                     "were bright";
    }

    int upper = dead_left;
    int lower = dead_right;

    for (int i = dead_left; i < (int)data.size(); i++) {
        if (data[i] > otsu) {
            upper = i;
        }
    }
    for (int i = dead_right; i > 0; i--) {
        if (data[i] > otsu) {
            lower = i;
        }
    }

    int even_start = lower - border;
    even_start += even_start % 2;
    even_start = std::max(0, even_start);
    int even_end = std::min(int(data.size() - 1), upper + border);
    even_end -= even_end % 2;
    int ewidth = (even_end - even_start);

    return {even_start, ewidth};
}
