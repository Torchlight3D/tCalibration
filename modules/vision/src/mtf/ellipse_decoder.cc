#include "ellipse_decoder.h"
#include "logger.h"

Ellipse_decoder::Ellipse_decoder(const Ellipse_detector &e, const cv::Mat &img)
    : code(-1), valid(false), ratio(e.minor_axis / e.major_axis)
{
    extract_code(e, img);
}

void Ellipse_decoder::extract_code(const Ellipse_detector &e,
                                   const cv::Mat &img)
{
    // collect histogram stats inside ellipse
    std::map<int, int> histo;
    double total = 0;
    for (auto it = e.scanset.begin(); it != e.scanset.end(); it++) {
        int y = it->first;
        for (int x = it->second.start; x <= it->second.end; x++) {
            int val = img.at<uint16_t>(y, x);

            auto hi = histo.find(val);
            if (hi == histo.end()) {
                histo[val] = 0;
            }
            histo[val]++;
            total++;
        }
    }
    double sum = 0;
    // compute Otsu threshold
    for (auto it = histo.begin(); it != histo.end(); it++) {
        sum += it->first * it->second;
    }
    double sum_b = 0;
    double w_b = 0;
    double max = 0;
    double thresh1 = 0;
    double thresh2 = 0;
    for (auto it = histo.begin(); it != histo.end(); it++) {
        w_b += it->second;
        double w_f = total - w_b;
        if (w_f == 0)
            break;

        sum_b += it->first * it->second;
        double m_b = sum_b / w_b;
        double m_f = (sum - sum_b) / w_f;
        double between = w_b * w_f * (m_b - m_f) * (m_b - m_f);
        if (between >= max) {
            thresh1 = it->first;
            if (between > max) {
                thresh2 = it->first;
            }
            max = between;
        }
    }
    int otsu = lrint((thresh1 + thresh2) * 0.5);

    double cs = cos(e.angle);
    double ss = sin(e.angle);

    // take 30 samples along inner track
    int steps = 30;
    int ones = 0;
    for (int i = 0; i < steps; i++) {
        double theta = i * 2.0 * M_PI / double(steps);

        double synth_x = 0.3 * e.major_axis * cos(theta);
        double synth_y = 0.3 * e.minor_axis * sin(theta);
        double px = cs * synth_x - ss * synth_y + e.centroid_x;
        double py = ss * synth_x + cs * synth_y + e.centroid_y;

        int bit = img.at<uint16_t>(lrint(py), lrint(px)) > otsu ? 1 : 0;
        ones += bit;
    }
    double inner_ratio = ones / double(steps);

    steps = 50;
    ones = 0;
    for (int i = 0; i < steps; i++) {
        double theta = i * 2.0 * M_PI / double(steps);
        double synth_x = 0.5 * e.major_axis * cos(theta);
        double synth_y = 0.5 * e.minor_axis * sin(theta);
        double px = cs * synth_x - ss * synth_y + e.centroid_x;
        double py = ss * synth_x + cs * synth_y + e.centroid_y;

        int bit = img.at<uint16_t>(lrint(py), lrint(px)) > otsu ? 1 : 0;
        ones += bit;
    }
    double outer_ratio = ones / double(steps);

    steps = 10;
    ones = 0;
    for (int i = 0; i < steps && !ones; i++) {
        double theta = i * 2.0 * M_PI / double(steps);

        double synth_x = 0.07 * e.major_axis * cos(theta);
        double synth_y = 0.07 * e.minor_axis * sin(theta);
        double px = cs * synth_x - ss * synth_y + e.centroid_x;
        double py = ss * synth_x + cs * synth_y + e.centroid_y;

        int bit = img.at<uint16_t>(lrint(py), lrint(px)) > otsu ? 1 : 0;
        ones += bit;
    }

    int ix = lrint(e.centroid_x);
    int iy = lrint(e.centroid_y);
    if (!ones && ix > 0 && iy > 0 && ix < img.cols - 1 && iy < img.rows - 1 &&
        img.at<uint16_t>(iy, ix) < otsu) {
        code = -1;
        valid = false;
        return;
    }

    int inner_code = lrint(inner_ratio * 3);
    int outer_code = lrint(outer_ratio * 5);
    int final_code = outer_code * 4 + inner_code;
    code = final_code;
    valid = true;

    if (e.fg_fraction > 0.9999) {
        logger.debug("%s\n", "ellipse too solid, cannot be a valid code");
        valid = false;
    }
}
