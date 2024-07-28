#include "esfmodel.h"

#include <array>
#include <cmath>

#include <glog/logging.h>

#include "samplingrate.h"

const std::array<std::string, 2> EsfModel::esf_model_names = {
    {"kernel", "loess"}};

EsfModel::EsfModel(double alpha) : alpha(alpha), w(NYQUIST_FREQ * 4, 0.0) {}

EsfModel::~EsfModel() {}

void EsfModel::moving_average_smoother(std::vector<double>& smoothed,
                                       double* sampled, int fft_size,
                                       int fft_left, int fft_right,
                                       int left_trans, int right_trans,
                                       int width)
{
    if (width < 1) {
        return;
    }

    width = std::min(width, 32);

    smoothed[0] = sampled[0];
    for (int idx = 1; idx < fft_size; idx++) {
        smoothed[idx] = smoothed[idx - 1] + sampled[idx];
    }

    const int tpad = width * 2;
    const int bhw = width;
    const int bhw_min = 1;
    for (int idx = std::max(fft_left + bhw, left_trans - tpad);
         idx < left_trans; idx++) {
        int lbhw = (left_trans - idx) * bhw / tpad + bhw_min;
        sampled[idx] = (smoothed[idx + lbhw] - smoothed[idx - lbhw - 1]) /
                       double(2 * lbhw + 1);
    }
    for (int idx = std::min(right_trans + tpad - 1, fft_right - bhw - 1);
         idx > right_trans; idx--) {
        int lbhw = (idx - right_trans) * bhw / tpad + bhw_min;
        sampled[idx] = (smoothed[idx + lbhw] - smoothed[idx - lbhw - 1]) /
                       double(2 * lbhw + 1);
    }
    for (int idx = bhw + 1; idx < left_trans - tpad; idx++) {
        sampled[idx] = (smoothed[idx + bhw] - smoothed[idx - bhw - 1]) /
                       double(2 * bhw + 1);
    }
    for (int idx = std::min(right_trans + tpad, fft_right - bhw - 1);
         idx < fft_size - bhw - 1; idx++) {
        sampled[idx] = (smoothed[idx + bhw] - smoothed[idx - bhw - 1]) /
                       double(2 * bhw + 1);
    }
}

int EsfModel::estimate_esf_clipping(std::vector<Ordered_point>& ordered,
                                    double* sampled, const int fft_size,
                                    bool allow_peak_shift, int effective_maxdot,
                                    std::vector<double>& mean,
                                    std::vector<double>& weights, int& fft_left,
                                    int& fft_right, int& twidth, Snr& snr)
{
    thread_local std::vector<double> slopes(fft_size, 0);
    constexpr size_t sample_histo_size = 64;
    int sample_histo[sample_histo_size];

    constexpr double shift_tolerance = 4;

    int rval = 0;

    const int fft_size2 = fft_size / 2;
    double rightsum = 0;
    int rightcount = 0;
    double leftsum = 0;
    int leftcount = 0;
    int left_non_missing = 0;
    int right_non_missing = 0;

    int clipped_count = 0;

    fft_left = fft_size2 - 8 * effective_maxdot;
    fft_right = fft_size2 + 8 * effective_maxdot;

retry:

    std::fill(weights.begin(), weights.end(), 0);
    std::fill(mean.begin(), mean.end(), 0);
    std::fill(sample_histo, sample_histo + sample_histo_size, 0);
    for (int i = 0; i < int(ordered.size()); i++) {
        int cbin = int(ordered[i].first * 8 + fft_size2);
        int left = std::max(fft_left, cbin - 5);
        int right = std::min(fft_right - 1, cbin + 5);

        for (int b = left; b <= right; b++) {
            double mid = (b - fft_size2) * 0.125;
            double w = 1 - std::abs((ordered[i].first - mid) * 1.75) > 0
                           ? 1 - std::abs((ordered[i].first - mid) * 1.75)
                           : 0;
            mean[b] += ordered[i].second * w;
            weights[b] += w;

            if (std::fabs(mid) <= double(sample_histo_size) / 16.0) {
                int hist_idx =
                    (ordered[i].first + double(sample_histo_size) / 16.0) * 8;
                if (hist_idx >= 0 && hist_idx < (int)sample_histo_size) {
                    sample_histo[hist_idx]++;
                }
            }
        }
    }
    const int leftsum_limit =
        std::max(fft_size2 - fft_size / 8, fft_left + 2 * 8);
    const int rightsum_limit =
        std::min(fft_size2 + fft_size / 8, fft_right - 2 * 8);
    // some housekeeping to take care of missing values
    for (int idx = fft_left - 1; idx <= fft_right + 1; idx++) {
        if (weights[idx] > 0) {
            sampled[idx] = mean[idx] / weights[idx];
            if (idx < leftsum_limit) {
                leftsum += sampled[idx];
                leftcount++;
            }
            if (idx > rightsum_limit) {
                rightsum += sampled[idx];
                rightcount++;
            }
            if (!left_non_missing) {
                left_non_missing = idx; // first non-missing value from left
            }
            right_non_missing = idx; // last non-missing value
        }
        else {
            sampled[idx] = missing;
        }
    }

    // now just pad out the ends of the sequences with the last non-missing
    // values
    for (int idx = left_non_missing - 1; idx >= 0; idx--) {
        sampled[idx] = sampled[left_non_missing];
    }
    for (int idx = right_non_missing + 1; idx < fft_size; idx++) {
        sampled[idx] = sampled[right_non_missing];
    }

    fill(slopes.begin(), slopes.end(), 0);
    int peak_slope_idx = 0;
    constexpr int pw = 16;
    for (int idx = pw + 1; idx < fft_size - 1 - pw; idx++) {
        double sx2 = 0;
        double sxy = 0;
        // sx == 0 because the window is symmetric
        for (int j = -pw; j <= pw; j++) {
            sxy += sampled[idx + j] * j;
            sx2 += j * j;
        }
        slopes[idx] = sxy / (sx2);
        if (fabs(slopes[idx]) > fabs(slopes[peak_slope_idx]) &&
            idx > fft_left + pw && idx < fft_right - pw - 1) {
            peak_slope_idx = idx;
        }
    }

    if (!allow_peak_shift) {
        if (abs(peak_slope_idx - fft_size / 2) >
                2 * 8 && // peak is more than 2 pixels from centre
            abs(peak_slope_idx - fft_size / 2) <
                12 * 8) { // but not at the edge?
            LOG(INFO) << "edge rejected because of shifted peak slope: "
                      << abs(peak_slope_idx - fft_size / 2) / 8.;
            return -1;
        }
    }

    // compute central peak magnitude and sign
    double central_peak = 0;
    for (int w = -16; w <= 16; w++) {
        if (fabs(slopes[fft_size2 + w]) > fabs(central_peak)) {
            central_peak = slopes[fft_size2 + w];
        }
    }

    double peak_threshold =
        fabs(central_peak * 0.001); // must be negative by at least a little bit
    // scan for significant slope sign change
    bool clipped = false;
    for (int idx = fft_size2 - 16; idx >= fft_left + 4; idx--) {
        if (slopes[idx] * central_peak < 0 &&
            fabs(slopes[idx]) > peak_threshold) {
            // check if a fair number of the remaining slopes are also negative
            // (relative to peak)
            int below = 0;
            double maxdev = 0;
            int scount = 0;
            for (int j = idx; j >= fft_left; j--) {
                if (slopes[j] * central_peak < 0) {
                    below++;
                    maxdev = std::max(maxdev, fabs(slopes[j]));
                }
                scount++;
            }
            if ((below > scount * 0.4 && maxdev / fabs(central_peak) > 0.25) ||
                (below > 0.9 * scount && scount > 16)) {
                fft_left = std::min(idx, fft_size2 - 2 * 8);
                clipped = true;
                break;
            }
        }
    }
    for (int idx = fft_size2 + 16; idx < fft_right - 4; idx++) {
        if (slopes[idx] * central_peak < 0 &&
            fabs(slopes[idx]) > peak_threshold) {
            // check if a fair number of the remaining slopes are also negative
            // (relative to peak)
            int below = 0;
            double maxdev = 0;
            int scount = 0;
            for (int j = idx; j < fft_right; j++) {
                if (slopes[j] * central_peak < 0) {
                    below++;
                    maxdev = std::max(maxdev, fabs(slopes[j]));
                }
                scount++;
            }
            if ((below > scount * 0.4 && maxdev / fabs(central_peak) > 0.25) ||
                (below > 0.9 * scount && scount > 16)) {
                fft_right = std::max(idx, fft_size2 + 2 * 8);
                clipped = true;
                break;
            }
        }
    }

    if (clipped) {
        if (fft_size2 - fft_left < shift_tolerance * 8 ||
            fft_right - fft_size2 < shift_tolerance * 8) {
            LOG(INFO) << "probably contamination. tagging edge as dodgy";
            rval = 1;
        }
    }

    if (clipped && clipped_count < 2) {
        for (size_t idx = 0; idx < weights.size(); idx++) {
            sampled[idx] = 0;
            weights[idx] = 0;
        }
        leftsum = 0;
        rightsum = 0;
        rightcount = 0;
        leftcount = 0;
        left_non_missing = 0;
        right_non_missing = 0;
        clipped_count++;
        goto retry;
    }

    leftsum /= double(leftcount);
    rightsum /= double(rightcount);

    // now find 10% / 90% thresholds
    double bright = std::max(leftsum, rightsum);
    double dark = std::min(leftsum, rightsum);
    int p10idx = fft_left - 1;
    int p90idx = fft_left - 1;
    double p10err = 1e50;
    double p90err = 1e50;
    for (int idx = fft_left; idx <= fft_right; idx++) {
        double smoothed = (sampled[idx - 2] + sampled[idx - 1] + sampled[idx] +
                           sampled[idx + 1] + sampled[idx + 2]) /
                          5.0;
        if (fabs((smoothed - dark) - 0.1 * (bright - dark)) < p10err) {
            p10idx = idx;
            p10err = fabs((smoothed - dark) - 0.1 * (bright - dark));
        }
        if (fabs((smoothed - dark) - 0.9 * (bright - dark)) < p90err) {
            p90idx = idx;
            p90err = fabs((smoothed - dark) - 0.9 * (bright - dark));
        }
    }
    // we know that mtf50 ~ 1/(p90idx - p10idx) * (1/samples_per_pixel)
    double rise_dist =
        std::max(double(4), fabs(double(p10idx - p90idx)) * 0.125);
    if (p10idx < p90idx) {
        std::swap(p10idx, p90idx);
    }
    p10idx += 4 + 2 * lrint(rise_dist); // advance at least one more full pixel
    p90idx -= 4 + 2 * lrint(rise_dist);
    twidth = std::max(fabs(double(p10idx - fft_size2)),
                      fabs(double(p90idx - fft_size2)));

    // Contrast-to-Noise-Ratio (CNR), effectively SNR for the S-E method
    // Only the esf_model_loess currently uses this, but to good effect
    std::vector<double> smooth_esf(fft_size, 0);
    constexpr double bwidth = 1.85;
    int left_trans = std::max(fft_size / 2 - bwidth * twidth, fft_left + 2.0);
    int right_trans = std::min(fft_size / 2 + bwidth * twidth, fft_right - 3.0);

    moving_average_smoother(smooth_esf, sampled, fft_size, fft_left, fft_right,
                            left_trans, right_trans);

    constexpr int tail_len = 2 * 8;
    double left_tail = 0;
    for (int idx = fft_left; idx < fft_left + tail_len; idx++) {
        left_tail += sampled[idx];
    }
    left_tail /= tail_len;
    double right_tail = 0;
    for (int idx = fft_right - tail_len; idx < fft_right; idx++) {
        right_tail += sampled[idx];
    }
    right_tail /= tail_len;

    const double fft_left_pix = std::max(
        0.125 * (double(fft_left) - fft_size / 2), ordered.front().first);
    const double fft_right_pix = std::min(
        0.125 * (double(fft_right) - fft_size / 2), ordered.back().first);
    // ensure that the range over which we will compute the CNR is at least two
    // pixels wide
    const double cnr_left_pix = std::max(fft_left_pix + 2, -2 * twidth * 0.125);
    const double cnr_right_pix =
        std::min(fft_right_pix - 2, 2 * twidth * 0.125);

    auto left_it =
        fft_left_pix < ordered.front().first
            ? ordered.begin()
            : std::lower_bound(ordered.begin(), ordered.end(), fft_left_pix);
    auto right_it = std::lower_bound(left_it, ordered.end(), cnr_left_pix);

    double left_sse = 0;
    size_t left_sse_count = 0;
    for (auto it = left_it; it != right_it; it++) {
        int cbin = std::max(fft_left, int(it->first * 8 + 0.5 + fft_size2));

        double e = sampled[cbin] - it->second;
        left_sse += e * e;
    }
    left_sse_count = right_it - left_it;

    left_it = lower_bound(right_it, ordered.end(), cnr_right_pix);
    right_it = fft_right_pix >= ordered.back().first
                   ? ordered.end()
                   : lower_bound(left_it, ordered.end(), fft_right_pix);

    double right_sse = 0;
    size_t right_sse_count = 0;
    for (auto it = left_it; it != right_it; it++) {
        int cbin = std::min(fft_right, int(it->first * 8 + 0.5 + fft_size2));

        double e = sampled[cbin] - it->second;
        right_sse += e * e;
    }
    right_sse_count += right_it - left_it;

    left_sse /= left_sse_count;
    right_sse /= right_sse_count;

    double bright_mean = left_tail;
    double dark_mean = right_tail;
    double bright_sd = sqrt(left_sse);
    double dark_sd = sqrt(right_sse);
    if (dark_mean > bright_mean) {
        std::swap(bright_mean, dark_mean);
        std::swap(bright_sd, dark_sd);
    }
    snr = Snr(dark_mean, dark_sd, bright_mean, bright_sd);

    // analyze the distribution of samples to calculate effective oversampling
    // factors
    double mean_oversampling = 0;
    for (size_t i = 0; i < sample_histo_size / 8; i++) {
        int oversamples = 8;
        for (size_t j = 0; j < 8; j++) {
            oversamples -= sample_histo[i * 8 + j] == 0 ? 1 : 0;
        }
        mean_oversampling += oversamples;
    }
    mean_oversampling /= (sample_histo_size / 8);
    snr.set_oversampling(mean_oversampling);

    return rval;
}

void EsfModel::set_alpha(double a)
{
    alpha = a;
    compute_mtf_corrections();
}

// Note: this is the basic correction for the central difference
// derivative approximation
void EsfModel::compute_mtf_corrections()
{
    w[0] = 1.0;
    for (int i = 1; i < NYQUIST_FREQ * 4; i++) {
        double dc_x = 2 * M_PI * i / double(NYQUIST_FREQ * 2);
        w[i] = sinc(dc_x / 8.0);
    }
}

const std::vector<double>& EsfModel::get_correction() const { return w; }

void EsfModel::set_monotonic_filter(bool b) { apply_monotonic_filter = b; }

double EsfModel::get_alpha() const { return alpha; }

// TODO: Put into math module
double EsfModel::sinc(double x) { return x == 0 ? 1 : sin(x) / x; }
