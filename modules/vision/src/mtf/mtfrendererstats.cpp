#include "mtfrendererstats.h"

#include <format>

#include <glog/logging.h>

namespace {
inline double quantile(const std::vector<double>& d, double q)
{
    size_t idx = (int)floor(d.size() * q);
    return d[idx];
}

} // namespace

Mtf_renderer_stats::Mtf_renderer_stats(bool lpmm_mode, double pixel_size)
    : pixel_size(lpmm_mode ? pixel_size : 1)
{
}

void Mtf_renderer_stats::render(const std::vector<Block>& blocks)
{
    if (blocks.empty()) {
        return;
    }

    std::vector<double> unfiltered;
    std::vector<double> filtered;
    for (const auto& block : blocks) {
        if (!block.valid) {
            continue;
        }

        for (size_t k = 0; k < 4; k++) {
            double val = block.get_mtf50_value(k);
            if (val == 1.0) {
                continue;
            }

            unfiltered.push_back(val);

            if (block.get_quality(k) >= 0.5) {
                filtered.push_back(val);
            }
        }
    }

    if (unfiltered.size() < 1) {
        return;
    }

    print_stats(unfiltered, filtered);
}

void Mtf_renderer_stats::render(const std::vector<Mtf_profile_sample>& samples)
{
    if (samples.empty()) {
        return;
    }

    std::vector<double> unfiltered;
    std::vector<double> filtered;
    for (const auto& sample : samples) {
        const double val = sample.mtf;
        if (val == 1.0) {
            continue;
        }

        unfiltered.push_back(val);

        if (sample.quality >= 0.5) {
            filtered.push_back(val);
        }
    }

    if (unfiltered.size() < 2) {
        return;
    }

    print_stats(unfiltered, filtered);
}

void Mtf_renderer_stats::print_stats(std::vector<double>& unfiltered,
                                     std::vector<double>& filtered)
{
    sort(filtered.begin(), filtered.end());
    sort(unfiltered.begin(), unfiltered.end());

    LOG(INFO) << std::format(
        "    Quantiles ->                   %5d%% %5d%% %5d%% "
        "%5d%% %5d%%",
        5, 25, 50, 75, 95);
    LOG(INFO) << std::format(
        "Statistics on all edges:           %5.4lf %5.4lf %5.4lf "
        "%5.4lf %5.4lf  (total=%u)",
        quantile(unfiltered, 0.05) * pixel_size,
        quantile(unfiltered, 0.25) * pixel_size,
        quantile(unfiltered, 0.5) * pixel_size,
        quantile(unfiltered, 0.75) * pixel_size,
        quantile(unfiltered, 0.95) * pixel_size,
        (unsigned int)unfiltered.size());

    if (filtered.size() < 2) {
        return;
    }

    LOG(INFO) << std::format(
        "Statistics on all filtered edges : %5.4lf %5.4lf %5.4lf "
        "%5.4lf %5.4lf  (total=%u)",
        quantile(filtered, 0.05) * pixel_size,
        quantile(filtered, 0.25) * pixel_size,
        quantile(filtered, 0.5) * pixel_size,
        quantile(filtered, 0.75) * pixel_size,
        quantile(filtered, 0.95) * pixel_size, (unsigned int)filtered.size());
}
