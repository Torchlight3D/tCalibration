#include "cropperbase.h"

#include <map>

namespace {
constexpr double kBinScale = 20.0;
}

Cropper::Cropper(const cv::Mat &img)
{
    int_rs = std::vector<double>(img.rows, 0);
    int_cs = std::vector<double>(img.cols, 0);
}

double Cropper::calcOtsuThreshold(const std::vector<double> &data) const
{
    // Statistic
    std::map<int, int> histogram;
    double total = 0;
    for (const auto &v : data) {
        int val = lrint(v * kBinScale);

        if (const auto hi = histogram.find(val); hi == histogram.end()) {
            histogram[val] = 0;
        }

        histogram[val]++;
        total++;
    }

    double sum = 0;
    for (const auto &[val, count] : histogram) {
        sum += val * count;
    }

    // Compute Otsu threshold
    double sum_b = 0;
    double w_b = 0;
    double max = 0;
    double thresh1 = 0;
    double thresh2 = 0;
    for (const auto &[val, count] : histogram) {
        w_b += count;
        double w_f = total - w_b;
        if (w_f == 0)
            break;

        sum_b += val * count;
        double m_b = sum_b / w_b;
        double m_f = (sum - sum_b) / w_f;
        double between = w_b * w_f * (m_b - m_f) * (m_b - m_f);
        if (between >= max) {
            thresh1 = val;
            if (between > max) {
                thresh2 = val;
            }
            max = between;
        }
    }

    return ((thresh1 + thresh2) * 0.5) / kBinScale;
}
