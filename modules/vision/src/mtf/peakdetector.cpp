#include "peakdetector.h"

#include <algorithm>

#include "utils.h"

namespace {

} // namespace

Peak_detector::Peak_detector(const std::vector<double>& in_data, size_t in_bins,
                             double in_min_data, double in_max_data)
    : nbins(in_bins),
      data(in_data),
      counts(in_bins, 0),
      min_data(in_min_data),
      max_data(in_max_data)
{
    range = max_data - min_data;

    // bias the counts so that ties during NMS are impossible
    for (size_t i = 0; i < in_bins; i++) {
        counts[i] = i;
    }

    // build histogram
    DBG(FILE* rawpts = fopen("raw_points.txt", "wt");)
    for (const auto& val : data) {
        DBG(fprintf(rawpts, "%lf\n", val);)
        size_t idx = lrint((nbins * (val - min_data) / range));
        if (idx > counts.size() - 1) {
            idx = counts.size() - 1;
        }
        counts[idx] += in_bins + 1;
    }
    DBG(fclose(rawpts);)

    // perform non-maximum suppression
    std::vector<size_t> nm(nbins, 0);
    const int delta = 3;
    for (size_t i = 0; i < nbins; i++) {
        int start = int(i) - delta;
        int end = int(i) + delta;

        size_t max_count = 0;
        for (int j = start; j < end; j++) {
            int eidx = j;
            if (eidx < 0)
                eidx = (eidx + nbins) % nbins;
            eidx = eidx % nbins;

            if (counts[eidx] > max_count) {
                max_count = counts[eidx];
            }
        }
        if (counts[i] < max_count) {
            nm[i] = 0;
        }
        else {
            nm[i] = counts[i];
        }
    }

    // now extract the peaks, noting both their value, and their counts
    DBG(FILE* fout = fopen("peaks.txt", "wt");)
    for (size_t i = 0; i < nbins; i++) {
        DBG(fprintf(fout, "%Zd %lf %Zd %Zd\n", i, i * range / double(nbins),
                    nm[i], counts[i]);)
        if (nm[i] > 0) {
            pq.push(std::pair<size_t, double>(
                counts[i], min_data + double(range * i) / double(nbins)));
        }
    }
    DBG(fclose(fout);)
}

void Peak_detector::select_best_n(std::vector<double>& best, size_t in_n,
                                  double il_thresh)
{
    // il_thresh is a fraction of "range"
    best.clear();
    for (size_t k = 0; k < in_n && !pq.empty(); k++) {
        best.push_back(pq.top().second);
        pq.pop();
    }

    // now go back to the data, and refine the values of the best ones
    std::vector<std::vector<double>> members(4);
    for (const auto& val : data) {
        for (size_t k = 0; k < best.size(); k++) {
            if (tl::angular_diff(val, best[k]) < il_thresh * range) {
                // close to minimum
                if (tl::angular_diff(best[k], min_data) < il_thresh * range &&
                    val < 0) {
                    // shift the origin if the peak is anywhere near -pi
                    members[k].push_back(val + 2 * M_PI);
                }
                else {
                    members[k].push_back(val);
                }
            }
        }
    }

    // compute trimmed mean, discarding <5 and >95 percentiles
    for (size_t k = 0; k < best.size(); k++) {
        std::ranges::sort(members[k]);
        double best_sum = 0.0;
        double best_count = 0.0;
        size_t lower = members[k].size() / 20;
        size_t upper = (members[k].size() * 19) / 20;
        for (size_t j = lower; j < upper; j++) {
            double weight = 1.0;
            best_sum += members[k][j] * weight;
            best_count += weight;
        }
        best[k] = best_sum / best_count;
    }
}
