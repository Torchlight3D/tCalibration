#include "cacore.h"

#include <opencv2/imgcodecs.hpp>

#include <tCore/Math>

#include "edgeinfo.h"

namespace tl {

namespace {
std::vector<double> binned_lsf(std::vector<Ordered_point>& ordered)
{
    std::vector<double> lsf(512, 0.0);

    thread_local std::vector<double> weights(512, 0.0);
    thread_local std::vector<double> mean(512, 0.0);

    std::fill(weights.begin(), weights.end(), 0);
    std::fill(mean.begin(), mean.end(), 0);
    for (int i = 0; i < int(ordered.size()); i++) {
        int cbin = int(ordered[i].first * 8 + 256);
        int left = std::max(0, cbin - 5);
        int right = std::min((int)lsf.size() - 1, cbin + 5);

        for (int b = left; b <= right; b++) {
            double mid = (b - 256) * 0.125;
            double w = 1 - abs((ordered[i].first - mid) * 1.75) > 0
                           ? 1 - abs((ordered[i].first - mid) * 1.75)
                           : 0;
            mean[b] += ordered[i].second * w;
            weights[b] += w;
        }
    }

    int left_non_missing = 0;
    int right_non_missing = 0;

    constexpr double missing = 1e9;

    // some housekeeping to take care of missing values
    for (size_t idx = 0; idx < lsf.size(); idx++) {
        if (weights[idx] > 0) {
            lsf[idx] = mean[idx] / weights[idx];
            if (!left_non_missing) {
                left_non_missing = idx; // first non-missing value from left
            }
            right_non_missing = idx; // last non-missing value
        }
        else {
            lsf[idx] = missing;
        }
    }

    // estimate a reasonable value for the non-missing samples
    constexpr int nm_target = 8 * 3;
    int nm_count = 1;
    double l_nm_mean = lsf[left_non_missing];
    for (int idx = left_non_missing + 1;
         idx < (int)lsf.size() / 2 && nm_count < nm_target; idx++) {
        if (lsf[idx] != missing) {
            l_nm_mean += lsf[idx];
            nm_count++;
        }
    }
    l_nm_mean /= nm_count;

    nm_count = 1;
    double r_nm_mean = lsf[right_non_missing];
    for (int idx = right_non_missing - 1;
         idx > (int)lsf.size() / 2 && nm_count < nm_target; idx--) {
        if (lsf[idx] != missing) {
            r_nm_mean += lsf[idx];
            nm_count++;
        }
    }
    r_nm_mean /= nm_count;

    // now just pad out the ends of the sequences with the last non-missing
    // values
    for (int idx = left_non_missing - 1; idx >= 0; idx--) {
        lsf[idx] = l_nm_mean;
    }
    for (int idx = right_non_missing + 1; idx < (int)lsf.size(); idx++) {
        lsf[idx] = r_nm_mean;
    }

    // apply Masaoka's interpolation method to take care of zero-count bins
    for (int idx = 0; idx < (int)lsf.size(); idx++) {
        if (std::abs(lsf[idx] - missing) < 1e-6) {
            int prev_present = idx - 1;
            int next_present = idx;
            while (std::abs(lsf[next_present] - missing) < 1e-6) {
                next_present++;
            }
            int j;
            for (j = prev_present + 1; j < next_present - 1; j++) {
                lsf[j] = lsf[prev_present];
            }
            lsf[j] = 0.5 * (lsf[prev_present] + lsf[next_present]);
        }
    }

    double old = lsf[0];
    for (int idx = 0; idx < (int)lsf.size() - 1; idx++) {
        double temp = lsf[idx];
        lsf[idx] = (lsf[idx + 1] - old);
        old = temp;
    }
    lsf[lsf.size() - 1] = lsf[lsf.size() - 2];

    return lsf;
}

double estimate_centroid(const std::vector<double>& a)
{
    constexpr int pad = 48;

    double sum = 0;
    double wsum = 0;
    double sqsum = 0;
    // initial centroid estimate using Hann weight function
    // this estimate can be quite far off if there is a large shift
    for (int i = pad; i <= (int)a.size() - pad; i++) {
        double theta = two_pi * (i - pad) / double(512 - 2 * pad);
        double w = 0.5 * (1.0 - cos(theta));
        double x = i * 0.125 - 32.0;

        sum += x * a[i] * w;
        wsum += a[i] * w;
    }

    double centroid_x = sum / wsum;

    // now estimate the variance to determine the width of
    // the Gaussian weighting function used below
    wsum = 0;
    for (int i = pad; i <= (int)a.size() - pad; i++) {
        double theta = two_pi * (i - pad) / double(512 - 2 * pad);
        double w = 0.5 * (1.0 - cos(theta));
        double x = (i * 0.125 - 32.0) - centroid_x;

        sqsum += x * x * a[i] * w;
        wsum += a[i] * w;
    }
    double var = std::min(256.0, std::max(1.0, std::abs(sqsum / wsum)));

    double delta_centroid = 50;
    size_t iters = 0;
    do {
        sum = wsum = 0;
        // estimate centroid with a Gaussian weight function
        for (int i = pad; i <= (int)a.size() - pad; i++) {
            double x = (i * 0.125 - 32.0);
            double wx = x - centroid_x;
            double w = exp(-wx * wx / (2.2 * var));

            sum += x * a[i] * w;
            wsum += a[i] * w;
        }
        delta_centroid = centroid_x - sum / wsum;
        centroid_x = sum / wsum;
    } while (std::abs(delta_centroid) > 1e-3 && iters++ < 10);

    return centroid_x;
}
} // namespace

void Ca_core::set_rgb_channels(std::vector<cv::Mat> in_channels)
{
    channels = in_channels;
}

void Ca_core::set_allow_all_edges() { allow_all_edges = true; }

void Ca_core::calculate_ca(Block& block)
{
    thread_local std::vector<std::vector<double>> red_lsf(4);
    thread_local std::vector<std::vector<double>> green_lsf(4);
    thread_local std::vector<std::vector<double>> blue_lsf(4);

    const double angle_threshold = std::cos(math::degToRad(45.));

    if (channels.empty()) {
        extract_rgb_lsf_bayer(block, mtf_core.img, mtf_core.bayer_img, red_lsf,
                              green_lsf, blue_lsf);
    }
    else {
        extract_rgb_lsf(block, mtf_core.img, channels, red_lsf, green_lsf,
                        blue_lsf);
    }

    cv::Point2d img_centre(mtf_core.img.cols / 2, mtf_core.img.rows / 2);

    for (size_t k = 0; k < 4; k++) {
        // skip edges that are likely to be truncated, or have really bad MTF
        // values for some other reason
        if (!block.get_edge_valid(k) || block.get_mtf50_value(k) >= 1.0) {
            block.set_ca(k, cv::Point2d(Edge_info::nodata, Edge_info::nodata));
            continue;
        }

        cv::Point2d dir = block.get_edge_centroid(k) - img_centre;
        dir = dir * (1.0 / norm(dir));
        double delta = dir.dot(block.get_normal(k));
        // only process CA on tangential edges, unless we
        // override this behaviour explicitly
        if (std::abs(delta) > angle_threshold || mtf_core.is_single_roi() ||
            allow_all_edges) {
            double green_centroid = estimate_centroid(green_lsf[k]);
            double red_ca = estimate_centroid(red_lsf[k]) - green_centroid;
            double blue_ca = estimate_centroid(blue_lsf[k]) - green_centroid;

            // choose the correct sign for CA shift, depending on edge
            // orientation
            if (std::abs(delta) > angle_threshold && delta < 0) {
                red_ca *= -1;
                blue_ca *= -1;
            }

            block.set_ca(k, cv::Point2d(red_ca, blue_ca));
        }
    }
}

void Ca_core::extract_rgb_lsf_bayer(Block& block, const cv::Mat& img,
                                    const cv::Mat& bayer_img,
                                    std::vector<std::vector<double>>& red_lsf,
                                    std::vector<std::vector<double>>& green_lsf,
                                    std::vector<std::vector<double>>& blue_lsf)
{
    std::vector<Ordered_point> ordered;
    double edge_length = 0;

    const double angle_threshold = std::cos(math::degToRad(45.));
    cv::Point2d img_centre(mtf_core.img.cols / 2, mtf_core.img.rows / 2);

    for (size_t k = 0; k < 4; k++) {
        cv::Point2d dir = block.get_edge_centroid(k) - img_centre;
        dir = dir * (1.0 / norm(dir));
        double delta = dir.dot(block.get_normal(k));

        // only process CA on tangential edges, unless we override this
        // behaviour explicitly
        if (!block.get_edge_valid(k) ||
            (std::abs(delta) <= angle_threshold &&
             !(mtf_core.is_single_roi() || allow_all_edges))) {
            continue;
        }

        ordered.clear();
        mtf_core.get_esf_sampler()->sample(
            block.get_edge_model(k), ordered, block.get_scanset(k), edge_length,
            img, bayer_img,
            Bayer::to_cfa_mask(Bayer::GREEN, mtf_core.get_cfa_pattern()));
        green_lsf[k] = binned_lsf(ordered);

        ordered.clear();
        mtf_core.get_esf_sampler()->sample(
            block.get_edge_model(k), ordered, block.get_scanset(k), edge_length,
            img, bayer_img,
            Bayer::to_cfa_mask(Bayer::RED, mtf_core.get_cfa_pattern()));
        red_lsf[k] = binned_lsf(ordered);

        ordered.clear();
        mtf_core.get_esf_sampler()->sample(
            block.get_edge_model(k), ordered, block.get_scanset(k), edge_length,
            img, bayer_img,
            Bayer::to_cfa_mask(Bayer::BLUE, mtf_core.get_cfa_pattern()));
        blue_lsf[k] = binned_lsf(ordered);
    }
}

void Ca_core::extract_rgb_lsf(Block& block, const cv::Mat& img,
                              const std::vector<cv::Mat>& channels,
                              std::vector<std::vector<double>>& red_lsf,
                              std::vector<std::vector<double>>& green_lsf,
                              std::vector<std::vector<double>>& blue_lsf)
{
    std::vector<Ordered_point> ordered;
    double edge_length = 0;

    const double angle_threshold = std::cos(math::degToRad(45.));
    cv::Point2d img_centre(mtf_core.img.cols / 2, mtf_core.img.rows / 2);

    for (size_t k = 0; k < 4; k++) {
        cv::Point2d dir = block.get_edge_centroid(k) - img_centre;
        dir = dir * (1.0 / norm(dir));
        double delta = dir.dot(block.get_normal(k));

        // only process CA on tangential edges, unless we override this
        // behaviour explicitly
        if (!block.get_edge_valid(k) ||
            (std::abs(delta) <= angle_threshold &&
             !(mtf_core.is_single_roi() || allow_all_edges))) {
            continue;
        }

        ordered.clear();
        mtf_core.get_esf_sampler()->sample(block.get_edge_model(k), ordered,
                                           block.get_scanset(k), edge_length,
                                           img, channels[1], Bayer::ALL);
        green_lsf[k] = binned_lsf(ordered);

        ordered.clear();
        mtf_core.get_esf_sampler()->sample(block.get_edge_model(k), ordered,
                                           block.get_scanset(k), edge_length,
                                           img, channels[2], Bayer::ALL);
        red_lsf[k] = binned_lsf(ordered);

        ordered.clear();
        mtf_core.get_esf_sampler()->sample(block.get_edge_model(k), ordered,
                                           block.get_scanset(k), edge_length,
                                           img, channels[0], Bayer::ALL);
        blue_lsf[k] = binned_lsf(ordered);
    }
}

} // namespace tl
