#include "mtfrendererprofile.h"

#include <format>

#include <glog/logging.h>

#include "common_types.h"
#include "loessfit.h"
#include "mtfcore.h"
#include "utils.h"

namespace {

double IQR(const std::vector<double> &counts, int start, int end)
{
    std::vector<double> sorted;
    start = std::max(0, start);
    end = std::min(counts.size() - 1, size_t(end));
    for (size_t i = start; i < size_t(end); i++) {
        sorted.push_back(counts[i]);
    }
    std::ranges::sort(sorted);
    return sorted[lrint(sorted.size() * 0.75)] -
           sorted[lrint(sorted.size() * 0.25)];
}

double median(const std::vector<double> &counts, int start, int end)
{
    std::vector<double> sorted;
    start = std::max(0, start);
    end = std::min(counts.size() - 1, size_t(end));
    for (size_t i = start; i < size_t(end); i++) {
        sorted.push_back(counts[i]);
    }
    std::ranges::sort(sorted);
    return sorted[lrint(sorted.size() * 0.5)];
}

void extract_row_maxima(std::map<int, double> &row_max,
                        const std::vector<Block> &blocks, bool transpose)
{
    row_max.clear();
    for (size_t i = 0; i < blocks.size(); i++) {
        for (size_t k = 0; k < 4; k++) {
            double val = blocks[i].get_mtf50_value(k);
            double angle = blocks[i].get_edge_angle(k);
            double mindiff = 0;

            if (val == 1.0)
                continue; // skip N/A blocks

            if (transpose) {
                mindiff = std::min(tl::angular_diff(angle, 0) / M_PI * 180,
                                   tl::angular_diff(angle, M_PI) / M_PI * 180);
            }
            else {
                mindiff =
                    std::min(tl::angular_diff(angle, M_PI / 2) / M_PI * 180,
                             tl::angular_diff(angle, -M_PI / 2) / M_PI * 180);
            }

            if (val > 0 && blocks[i].get_quality(k) >= 0.5 && mindiff < 30) {
                cv::Point2d cent = blocks[i].get_edge_centroid(k);

                int y = 0;

                if (transpose) {
                    y = lrint(cent.x);
                }
                else {
                    y = lrint(cent.y);
                }

                auto it = row_max.find(y);
                if (it == row_max.end()) {
                    row_max[y] = val;
                }
                else {
                    if (val >= row_max[y]) {
                        row_max[y] = val;
                    }
                }
            }
        }
    }
}

bool test_for_bimodal_distribution(const std::map<int, double> &data)
{
    int minval = data.begin()->first;
    int maxval = data.rbegin()->first;
    int span = maxval - minval;

    if (span == 0) { // all the samples are in a perfectly straight line, so
        // we have to take the transpose
        return true;
    }

    int nbins = 60;

    // bin coarser
    std::vector<double> counts(nbins, 0);
    for (auto it = data.begin(); it != data.end(); ++it) {
        int idx = (it->first - minval) * nbins / (span);
        idx = std::max(0, idx);
        idx = std::min(nbins - 1, idx);
        counts[idx] = std::max(it->second, counts[idx]);
    }

    // find two breakpoints in density; we are looking for a -_- shape,
    // which indicates that the chart has been rotated
    double min_cost = 1e50;
    int min_start = 0;
    int min_end = 0;
    for (int b_start = 10; b_start < 2 * nbins / 3; b_start += 2) {
        for (int b_end = std::max(nbins / 2, b_start + 4); b_end < nbins - 10;
             b_end += 2) {
            double cost = IQR(counts, 0, b_start) +
                          10 * IQR(counts, b_start, b_end) +
                          IQR(counts, b_end, nbins);
            if (cost < min_cost) {
                min_cost = cost;
                min_start = b_start;
                min_end = b_end;
            }
        }
    }
    double first_segment = median(counts, 0, min_start);
    double middle_segment = median(counts, min_start, min_end);
    double last_segment = median(counts, min_end, nbins);

    return middle_segment <= first_segment && middle_segment <= last_segment;
}

} // namespace

Mtf_renderer_profile::Mtf_renderer_profile(const std::string &img_filename,
                                           const std::string &wdir,
                                           const std::string &prof_fname,
                                           const std::string &gnuplot_binary,
                                           const cv::Mat &img,
                                           int gnuplot_width, bool lpmm_mode,
                                           double pixel_size, int mtf_contrast)
    : wdir(wdir),
      prname(prof_fname),
      gnuplot_binary(gnuplot_binary),
      img(img),
      lpmm_mode(lpmm_mode),
      pixel_size(pixel_size),
      gnuplot_failure(false),
      gnuplot_warning(true),
      gnuplot_width(gnuplot_width),
      img_filename(img_filename),
      mtf_contrast(mtf_contrast)
{
}

void Mtf_renderer_profile::render(const std::vector<Block> &blocks)
{
    size_t largest_block = 0;
    cv::Point2d centroid(0, 0);

    size_t valid_blocks = 0;
    for (const auto &block : blocks) {
        for (size_t k = 0; k < 4; k++) {
            valid_blocks += block.get_mtf50_value(k) < 1;
        }
    }

    if (blocks.size() < 10 ||
        valid_blocks < 40) { // probably not a valid image for profiles
        return;
    }

    std::map<int, double> row_max;
    extract_row_maxima(row_max, blocks, false);

    if (row_max.empty()) {
        return;
    }

    // check distribution: a bimodal distribution probably means the image
    // is rotated
    bool transpose = test_for_bimodal_distribution(row_max);
    if (transpose) {
        LOG(INFO) << "bimodal distribution found, taking transpose";
        extract_row_maxima(row_max, blocks, true);
    }

    // compute robust maximum
    std::vector<double> mtf50_values;
    for (auto it = row_max.begin(); it != row_max.end(); ++it) {
        mtf50_values.push_back(it->second);
    }
    std::ranges::sort(mtf50_values);
    // pick 95% percentile
    double effective_max = mtf50_values[0.98 * mtf50_values.size()];
    // adjust it upwards a little
    effective_max /= 0.9;

    for (size_t i = 0; i < blocks.size(); i++) {
        if (blocks[i].get_area() > blocks[largest_block].get_area()) {
            largest_block = i;
        }
        centroid.x += blocks[i].get_centroid().x;
        centroid.y += blocks[i].get_centroid().y;
    }

    if (row_max.empty()) {
        LOG(INFO) << std::format(
            "Warning: All MTF{} values are zero, cannot generate a profile.",
            mtf_contrast);
        return;
    }

    // apply some median filtering to remove obvious outliers
    // this unfortunately chops the peaks of the sharp double-exponential
    // curves seen on synthetic images. ah well ...
    const size_t w = 2;
    std::vector<double> med_filt_mtf(row_max.size(), 0);
    size_t i = 0;
    std::vector<Ordered_point> ordered;
    for (auto it = row_max.begin(); it != row_max.end(); ++it) {
        std::vector<double> medwin;
        auto start = it;
        auto end = it;
        for (size_t j = 0; j < w && start != row_max.begin(); j++, --start);
        for (size_t j = 0; j <= w && end != row_max.end(); j++, ++end);
        for (; start != end; ++start) {
            medwin.push_back(start->second);
        }
        std::ranges::sort(medwin);
        med_filt_mtf[i++] = medwin[medwin.size() / 2];
        double ex = (it->first - img.cols / 2) / double(img.cols) * max_dot;
        ordered.push_back(Ordered_point(ex, medwin[medwin.size() / 2]));
    }

    // apply additional smoothing
    // try various LOESS filter sizes until the sign changes in the slope
    // drops below 5% (which seems to provide relatively oscillation-free
    // curves)
    if (ordered.size() > 5) {
        for (size_t w2 = 5; w2 < std::max(ordered.size() / 10, size_t(6));
             w2 += 3) {
            for (size_t i = 0; i < ordered.size() - 1; i++) {
                cv::Point2d sol;

                size_t start = std::max(0, int(i) - int(w2));
                size_t end = std::min(ordered.size() - 1, i + w2);
                loess_core(ordered, start, end, ordered[i].first, sol);

                med_filt_mtf[i] = ordered[i].first * sol.y + sol.x;
            }
            double frac_sign_change = 0;
            for (size_t i = 3; i < ordered.size() - 2; i++) {
                if ((med_filt_mtf[i] - med_filt_mtf[i - 2]) *
                        (med_filt_mtf[i + 2] - med_filt_mtf[i]) <
                    0) {
                    frac_sign_change += 1.0;
                }
            }
            frac_sign_change /= ordered.size();
            if (frac_sign_change < 0.05) { // less than 5% of sequential
                // slopes experience sign changes
                break;
            }
        }
    }

    FILE *prfile = fopen((wdir + prname).c_str(), "wt");
    i = 0;
    double max_med_filt = 0;
    int max_med_filt_coord = 0;
    for (auto it = row_max.begin(); it != row_max.end(); ++it) {
        if (med_filt_mtf[i] >= max_med_filt) {
            max_med_filt = med_filt_mtf[i];
            max_med_filt_coord = it->first;
        }
        fprintf(prfile, "%lf %lf %lf\n", it->first / pixel_size,
                it->second * pixel_size, med_filt_mtf[i++] * pixel_size);
    }
    fprintf(prfile, "\n\n");

    centroid.x -= blocks[largest_block].get_centroid().x;
    centroid.y -= blocks[largest_block].get_centroid().y;

    centroid.x /= blocks.size() - 1;
    centroid.y /= blocks.size() - 1;

    // profile peak
    size_t peak_idx = 0;
    double min_dist = 1e50;
    for (size_t k = 0; k < 4; k++) {
        double dist = sqrt(
            SQR(centroid.x - blocks[largest_block].get_edge_centroid(k).x) +
            SQR(centroid.y - blocks[largest_block].get_edge_centroid(k).y));

        if (dist < min_dist) {
            min_dist = dist;
            peak_idx = k;
        }
    }

    double peak_shift = 0;
    double ref_edge_position = 0;
    if (transpose) {
        ref_edge_position = blocks[largest_block].get_edge_centroid(peak_idx).x;
    }
    else {
        ref_edge_position = blocks[largest_block].get_edge_centroid(peak_idx).y;
    }
    peak_shift = ref_edge_position - max_med_filt_coord;

    // The LOESS-filtered curve is already a good approximation of the mean
    // MTF50 at any given position along the curve. We can thus estimate
    // mean MTF50 at the position of the reference edge by directly looking
    // up the value in the smoothed curve.
    double mean_mtf50_at_ref_edge = 0;
    min_dist = 1e50;
    i = 0;
    for (const auto &[pos, _] : row_max) {
        double dist = std::abs(pos - ref_edge_position);
        if (dist < min_dist) {
            min_dist = dist;
            mean_mtf50_at_ref_edge = med_filt_mtf[i++];
        }
    }

    LOG(INFO) << std::format("peak_shift={}, mtf_at_peak={} {}", peak_shift,
                             mean_mtf50_at_ref_edge * pixel_size,
                             lpmm_mode ? "lp/mm" : "c/p");

    double peak_mtf50 = blocks[largest_block].get_mtf50_value(peak_idx);
    bool peak_quality_good = true;
    if (!blocks[largest_block].get_quality(peak_idx)) {
        peak_mtf50 = max_med_filt;
        peak_quality_good = false;
    }
    fprintf(
        prfile, "%lf %lf %lf\n",
        transpose
            ? blocks[largest_block].get_edge_centroid(peak_idx).x / pixel_size
            : blocks[largest_block].get_edge_centroid(peak_idx).y / pixel_size,
        peak_mtf50 * pixel_size, peak_mtf50 * 3 * pixel_size);
    fclose(prfile);

    FILE *gpf = fopen((wdir + std::string("profile.gnuplot")).c_str(), "wt");
    fprintf(gpf, "set xlab \"column (%s)\"\n", lpmm_mode ? "mm" : "pixels");
    fprintf(gpf, "set ylab \"MTF%2d (%s)\"\n", mtf_contrast,
            lpmm_mode ? "line pairs per mm" : "cycles/pixel");
    double ar = 768.0 / 1024.0;
    int fontsize = lrint(12.0 * gnuplot_width / 1024.0);
    int title_fontsize = lrint(14.0 * gnuplot_width / 1024.0);
    int linewidth = lrint(3 * gnuplot_width / 1024.0);
    double pointsize = 0.5 * gnuplot_width / 1024.0;
    fprintf(gpf,
            "set term pngcairo dashed transparent enhanced size %d, %d "
            "font '%s,%d'  background rgb \"white\"\n",
            gnuplot_width, int(gnuplot_width * ar),
#ifdef _WIN32
            "Verdana",
#else
            "Arial",
#endif
            fontsize);
    if (img_filename.length() > 0) {
        fprintf(gpf, "set title \"%s\" font \",%d\"\n", img_filename.c_str(),
                title_fontsize);
    }
    fprintf(gpf, "set output \"%sprofile_image.png\"\n", wdir.c_str());
    fprintf(gpf,
            "plot [][0:%lf] \"%s\" i 0 u 1:3 t \"MTF%2d (%s) smoothed\" w "
            "l lw %d lc rgb \"#0020af20\", \"%s\" i 0 u 1:2 t \"MTF%2d "
            "(%s) raw\" w p ps %lf lc rgb \"#70ff2020\",  \"%s\" i 1 u 1:2 "
            "t \"Expected focus point\" w i lc %d lw %d\n",
            effective_max * pixel_size, (wdir + prname).c_str(), mtf_contrast,
            lpmm_mode ? "lp/mm" : "c/p", linewidth, (wdir + prname).c_str(),
            mtf_contrast, lpmm_mode ? "lp/mm" : "c/p", pointsize,
            (wdir + prname).c_str(), peak_quality_good ? 3 : 1, linewidth);

    fclose(gpf);

    char *buffer = new char[1024];
#ifdef _WIN32
    sprintf(buffer, "\"\"%s\" \"%sprofile.gnuplot\"\"", gnuplot_binary.c_str(),
            wdir.c_str());
#else
    sprintf(buffer, "\"%s\" \"%sprofile.gnuplot\"", gnuplot_binary.c_str(),
            wdir.c_str());
#endif
    int rval = system(buffer);
    if (rval != 0) {
        LOG(ERROR) << std::format("Failed to execute gnuplot (error code {})",
                                  rval);
        LOG(INFO) << std::format(
            "You can try to execute [{}] to render the plots manually", buffer);
        gnuplot_failure = true;
    }
    else {
        LOG(INFO) << "Gnuplot plot completed successfully. Look for "
                     "profile_image.png";
    }

    delete[] buffer;
}

void Mtf_renderer_profile::set_gnuplot_warning(bool gnuplot)
{
    gnuplot_warning = gnuplot;
}

bool Mtf_renderer_profile::gnuplot_failed() const { return gnuplot_failure; }
