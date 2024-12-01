#include "mtfcore.h"

#include <array>
#include <format>
#include <memory>
#include <mutex>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include <magic_enum/magic_enum.hpp>

#include "edgemodel.h"
#include "ellipsedecoder.h"
#include "esfsamplerdeferred.h"
#include "esfsamplerline.h"
#include "esfsamplerpiecewisequad.h"
#include "esfsamplerquad.h"
#include "mtf50edgequalityrating.h"
#include "peakdetector.h"
#include "pointhelpers.h"

#include "savitzkygolaytables.h"

// global lock to prevent race conditions on detected_blocks
static std::mutex global_mutex;

class Rect_roi
{
public:
    Rect_roi(const cv::Point2d &a, const cv::Point2d &b, double half_width)
        : half_width(half_width), base(a)
    {
        dir = b - a;
        len = norm(dir);
        if (len > 0) {
            dir *= 1.0 / len;
        }
        n = cv::Point2d(-dir.y, dir.x);
    }

    void activate() { active = true; }

    inline bool inside(const cv::Point2d &p) const
    {
        if (!active)
            return true;

        cv::Point2d d(p - base);
        return d.dot(dir) >= 0 && d.dot(dir) <= len &&
               std::abs(d.dot(n)) <= half_width;
    }

    inline bool inside(int x, int y) const { return inside(cv::Point2d(x, y)); }

    cv::Rect bounds(const cv::Mat &img) const
    {
        std::vector<cv::Point2d> corners = {base + len * dir - half_width * n,
                                            base + len * dir + half_width * n,
                                            base - half_width * n,
                                            base + half_width * n};

        cv::Point2d min_v(corners[0]);
        cv::Point2d max_v(corners[0]);

        for (auto &c : corners) {
            min_v.x = std::min(min_v.x, c.x);
            min_v.y = std::min(min_v.y, c.y);
            max_v.x = std::max(max_v.x, c.x);
            max_v.y = std::max(max_v.y, c.y);
        }

        min_v.x = std::max(min_v.x, double(0));
        min_v.y = std::max(min_v.y, double(0));
        max_v.x = std::min(max_v.x, double(img.cols));
        max_v.y = std::min(max_v.y, double(img.rows));

        return cv::Rect(lrint(min_v.x), lrint(min_v.y), lrint(max_v.x),
                        lrint(max_v.y));
    }

private:
    double half_width;
    cv::Point2d base;
    cv::Point2d dir;
    cv::Point2d n;
    double len;

    bool active = false;
};

Mtf_core::Mtf_core(const Component_labeller &in_cl, const Gradient &in_g,
                   const cv::Mat &in_img, const cv::Mat &in_bayer_img,
                   std::string bayer_subset, std::string cfa_pattern_name,
                   std::string esf_sampler_name, Undistort *undistort,
                   int border_width)

    : cl(in_cl),
      g(in_g),
      img(in_img),
      bayer_img(in_bayer_img),
      absolute_sfr(false),
      snap_to(false),
      snap_to_angle(0),
      sfr_smoothing(true),
      sliding(false),
      samples_per_edge(0),
      find_fiducials(false),
      undistort(undistort),
      ridges_only(false)
{
    // TODO: Handle failure
    bayer = magic_enum::enum_cast<Bayer::bayer_t>(bayer_subset).value();
    cfa_pattern =
        magic_enum::enum_cast<Bayer::cfa_pattern_t>(cfa_pattern_name).value();
    LOG(INFO) << std::format("Bayer subset is {}, CFA pattern type is {}",
                             bayer_subset, cfa_pattern_name);

    switch (EsfSampler::from_string(esf_sampler_name)) {
        case EsfSampler::LINE:
            esf_sampler = new EsfLineSampler(
                max_dot, Bayer::to_cfa_mask(bayer, cfa_pattern), border_width);
            break;
        case EsfSampler::QUAD:
            esf_sampler = new EsfQuadSampler(
                max_dot, Bayer::to_cfa_mask(bayer, cfa_pattern), border_width);
            break;
        case EsfSampler::PIECEWISE_QUAD:
            esf_sampler = new EsfPiecewiseQuadSampler(
                max_dot, Bayer::to_cfa_mask(bayer, cfa_pattern), border_width);
            break;
        case EsfSampler::DEFERRED:
            if (!undistort) { // hopefully we stop it before this point ...
                LOG(FATAL) << "No undistortion model provided to "
                              "Esf_sampler_deferred. Aborting";
            }
            esf_sampler = new EsfDeferredSampler(
                undistort, max_dot, Bayer::to_cfa_mask(bayer, cfa_pattern),
                border_width);
            break;
    };

    for (const auto &[index, _] : cl.get_boundaries()) {
        valid_obj.push_back(index);
    }

    cv::Mat temp;
    in_img.convertTo(temp, CV_8U, 256.0 / 16384.0);
    cv::cvtColor(temp, od_img, cv::COLOR_GRAY2RGB);
}

Mtf_core::~Mtf_core()
{
    // cv::imwrite(string("detections.png"), od_img);
}

void Mtf_core::search_borders(const cv::Point2d &cent, int label)
{
    Mrectangle rrect;
    bool valid = extract_rectangle(cent, label, rrect);

    if (!valid && find_fiducials && cl.largest_hole(label) > 0) {
        // this may be an ellipse. check it ...
        auto it = cl.get_boundaries().find(label);
        Ellipse_detector e;
        int valid = e.fit(cl, g, it->second, 0, 0, 2);
        if (valid) {
            Ellipse_decoder ed(e, img);

            // search small region near centre for mismatched labels
            // this is almost a duplicate of the foreground fraction check
            // in the Ellipse_detector class, but adds the constraint that
            // the hole must be near the centre ....
            // TODO: consolidate fiducial validation in one spot
            bool hole_found = false;
            for (int dy = -3; dy <= 3 && !hole_found; dy++) {
                for (int dx = -3; dx <= 3 && !hole_found; dx++) {
                    if (cl(lrint(e.centroid_x + dx),
                           lrint(e.centroid_y + dy)) != label) {
                        hole_found = true;
                    }
                }
            }

            if (ed.valid && ed.code >= 0 && hole_found) {
                {
                    std::lock_guard<std::mutex> lock(global_mutex);
                    e.valid = ed.valid;
                    e.set_code(ed.code);
                    ellipses.push_back(e);
                    LOG(INFO) << std::format(
                        "Fiducial with code {} extracted at ({} {})", e.code,
                        e.centroid_x, e.centroid_y);
                }

                for (double theta = 0; theta < 2 * M_PI;
                     theta += M_PI / 720.0) {
                    double synth_x = e.major_axis * cos(theta);
                    double synth_y = e.minor_axis * sin(theta);
                    double rot_x = cos(e.angle) * synth_x -
                                   sin(e.angle) * synth_y + e.centroid_x;
                    double rot_y = sin(e.angle) * synth_x +
                                   cos(e.angle) * synth_y + e.centroid_y;

                    // clip to image size, just in case
                    rot_x = std::max(rot_x, 0.);
                    rot_x = std::min(rot_x, od_img.cols - 1.);
                    rot_y = std::max(rot_y, 0.);
                    rot_y = std::min(rot_y, od_img.rows - 1.);

                    auto &color =
                        od_img.at<cv::Vec3b>(lrint(rot_y), lrint(rot_x));
                    color[0] = 255;
                    color[1] = 255;
                    color[2] = 0;
                }

                const auto buffer = std::to_string(e.code);
                int baseline = 0;
                cv::Size ts = cv::getTextSize(buffer, cv::FONT_HERSHEY_SIMPLEX,
                                              0.5, 1, &baseline);
                cv::Point to(-ts.width / 2, ts.height / 2);
                to.x += e.centroid_x;
                to.y += e.centroid_y;

                cv::putText(od_img, buffer, to, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            CV_RGB(20, 20, 20), 2, cv::LINE_AA);
                cv::putText(od_img, buffer, to, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            CV_RGB(0, 255, 255), 1, cv::LINE_AA);
            }
        }
        return;
    }

    Block block(rrect);

    if (block.get_area() <= 225) {
        return;
    }

    if (!rrect.corners_ok() || !rrect.valid) {
        LOG(INFO) << "discarding broken square (early)";
        return;
    }

    if (sliding) {
        process_with_sliding_window(rrect);
        return;
    }

    if (!ridges_only) {
        if (!homogenous(cent, label, rrect)) {
            LOG(INFO) << "discarding inhomogenous object";
            return;
        }
    }

    for (size_t k = 0; k < 4; k++) {
        if (rrect.corners[k].x >= 0 && rrect.corners[k].x < g.width() &&
            rrect.corners[k].y >= 0 && rrect.corners[k].y < g.height()) {
            cv::Vec3b &color =
                od_img.at<cv::Vec3b>(rrect.corners[k].y, rrect.corners[k].x);
            color[0] = 255;
            color[1] = 255;
            color[2] = 0;
        }
    }

    std::vector<Edge_record> edge_record(4);
    std::vector<std::map<int, scanline>> scansets(4);
    std::vector<std::shared_ptr<Edge_model>> edge_model(4);
    bool reduce_success = true;
    bool small_target = false;
    for (size_t k = 0; k < 4; k++) {
        // now construct buffer around centroid, aligned with direction, of
        // width max_dot
        Mrectangle nr(rrect, k, (undistort ? 4 : 1) * max_dot + 0.5);

        double max_edge_len = 0;
        double min_perp_dist = 9;
        double par_dist_bias = 0;
        for (int kk = 0; kk < 4; kk++) {
            double el = cv::norm(nr.corners[kk] - nr.corners[(kk + 1) % 4]);
            max_edge_len = std::max(max_edge_len, el);
            min_perp_dist = std::min(min_perp_dist, el);
            double par_dist =
                std::abs(
                    (nr.corners[kk] - rrect.centroids[k]).dot(rrect.edges[k])) /
                cv::norm(rrect.edges[k]);
            par_dist_bias = std::max(par_dist_bias, par_dist);
        }

        for (int kk = 0; kk < 4; kk++) {
            cv::line(od_img, nr.corners[kk], nr.corners[(kk + 1) % 4],
                     cv::Scalar(0, 255, 128));
        }

        edge_model[k] = std::shared_ptr<Edge_model>(new Edge_model(
            rrect.centroids[k], rrect.edges[k], rrect.quad_coeffs[k]));
        Edge_model *emk = edge_model[k].get();

        double perp_threshold = nr.length >= 45 ? 12.0 : 8.0;
        small_target |= nr.length < 45;

        emk->hint_point_set_size(
            (int)ceil(par_dist_bias), (int)ceil(max_edge_len + 2),
            (int)ceil(2 * std::max(perp_threshold, min_perp_dist) + 4));

        for (double y = nr.tl.y; y < nr.br.y; y += 1.0) {
            for (double x = nr.tl.x; x < nr.br.x; x += 1.0) {
                cv::Point2d p(x, y);
                if (!nr.is_inside(p)) {
                    continue;
                }

                cv::Point2d d = p - rrect.centroids[k];

                int iy = lrint(y);
                int ix = lrint(x);
                if (iy >= 0 && iy < img.rows && ix >= 0 && ix < img.cols &&
                    std::abs(d.ddot(rrect.normals[k])) < perp_threshold) {
                    edge_record[k].add_point(x, y, fabs(g.grad_x(ix, iy)),
                                             fabs(g.grad_y(ix, iy)));
                }

                if (iy >= 0 && iy < img.rows && ix >= 0 && ix < img.cols) {
                    emk->add_point(x, y, g.grad_magnitude(ix, iy),
                                   perp_threshold);
                }

                auto it = scansets[k].find(iy);
                if (it == scansets[k].end()) {
                    scanline sl(ix, ix);
                    scansets[k].insert(std::make_pair(iy, sl));
                }
                if (ix < scansets[k][iy].start) {
                    scansets[k][iy].start = ix;
                }
                if (ix > scansets[k][iy].end) {
                    scansets[k][iy].end = ix;
                }
            }
        }

        edge_model[k]->estimate_ridge();
        reduce_success &= edge_record[k].reduce();
    }

    double max_shift = 0;
    if (!ridges_only && !small_target && reduce_success) {
        // re-calculate the ROI after we have refined the edge centroid above
        Mrectangle newrect(rrect, edge_record);
        if (!newrect.corners_ok()) {
            LOG(INFO) << "discarding broken square (after updates)";
            return;
        }

        scansets = std::vector<std::map<int, scanline>>(4); // re-initialise
        for (size_t k = 0; k < 4; k++) {
            // now construct buffer around centroid, aligned with direction, of
            // width max_dot
            Mrectangle nr(newrect, k, (undistort ? 4 : 1) * max_dot + 0.5);
            edge_record[k].clear();

            double perp_threshold = nr.length >= 45 ? 12.0 : 6.0;

            for (double y = nr.tl.y; y < nr.br.y; y += 1.0) {
                for (double x = nr.tl.x; x < nr.br.x; x += 1.0) {
                    cv::Point2d p(x, y);
                    if (!nr.is_inside(p)) {
                        continue;
                    }

                    int iy = lrint(y);
                    int ix = lrint(x);
                    if (iy < 0 || iy >= img.rows || ix < 0 || ix >= img.cols) {
                        continue;
                    }

                    cv::Point2d d = p - newrect.centroids[k];
                    if (std::abs(d.ddot(newrect.normals[k])) < perp_threshold) {
                        edge_record[k].add_point(x, y, fabs(g.grad_x(ix, iy)),
                                                 fabs(g.grad_y(ix, iy)));
                    }

                    auto it = scansets[k].find(iy);
                    if (it == scansets[k].end()) {
                        scanline sl(ix, ix);
                        scansets[k].insert(std::make_pair(iy, sl));
                    }
                    if (ix < scansets[k][iy].start) {
                        scansets[k][iy].start = ix;
                    }
                    if (ix > scansets[k][iy].end) {
                        scansets[k][iy].end = ix;
                    }
                }
            }

            cv::Point2d ocx = edge_record[k].centroid;
            reduce_success &= edge_record[k].reduce();
            cv::Point2d ncx = edge_record[k].centroid;
            double shift = sqrt(SQR(ocx.x - ncx.x) + SQR(ocx.y - ncx.y));
            max_shift = std::max(max_shift, shift);
        }
        rrect = newrect;
    }

    if (!ridges_only && !small_target && reduce_success && max_shift > 1) {
        // re-calculate the ROI after we have refined the edge centroid above
        Mrectangle newrect(rrect, edge_record);
        if (!newrect.corners_ok()) {
            LOG(INFO) << "discarding broken square (after updates)";
            return;
        }

        scansets = std::vector<std::map<int, scanline>>(4); // re-initialise
        for (size_t k = 0; k < 4; k++) {
            // now construct buffer around centroid, aligned with direction, of
            // width max_dot
            Mrectangle nr(newrect, k, (undistort ? 4 : 1) * max_dot + 0.5);
            edge_record[k].clear();
            scansets[k].clear();

            double perp_threshold = nr.length >= 45 ? 12.0 : 6.0;

            for (double y = nr.tl.y; y < nr.br.y; y += 1.0) {
                for (double x = nr.tl.x; x < nr.br.x; x += 1.0) {
                    cv::Point2d p(x, y);
                    if (!nr.is_inside(p)) {
                        continue;
                    }

                    int iy = lrint(y);
                    int ix = lrint(x);
                    if (iy < 0 || iy >= img.rows || ix < 0 || ix >= img.cols) {
                        continue;
                    }

                    cv::Point2d d = p - newrect.centroids[k];
                    if (std::abs(d.ddot(newrect.normals[k])) < perp_threshold) {
                        edge_record[k].add_point(x, y,
                                                 std::abs(g.grad_x(ix, iy)),
                                                 std::abs(g.grad_y(ix, iy)));
                    }

                    auto it = scansets[k].find(iy);
                    if (it == scansets[k].end()) {
                        scanline sl(ix, ix);
                        scansets[k].insert(std::make_pair(iy, sl));
                    }
                    if (ix < scansets[k][iy].start) {
                        scansets[k][iy].start = ix;
                    }
                    if (ix > scansets[k][iy].end) {
                        scansets[k][iy].end = ix;
                    }
                }
            }
            reduce_success &= edge_record[k].reduce();
        }
    }

    if (!reduce_success) {
        LOG(INFO) << "reduce failed, probably not a rectangle/quadrangle";
        return;
    }

#if 1
    std::vector<std::pair<double, std::pair<int, int>>> pairs;
    for (size_t k = 0; k < 3; k++) {
        for (size_t l = k + 1; l < 4; l++) {
            double rel_sim =
                edge_record[k].relative_orientation(edge_record[l]);
            if (rel_sim > 0.80902) { // = cos(M_PI/5.0) ~ 36 degrees
                pairs.push_back(std::make_pair(1 - rel_sim,
                                               std::make_pair(int(k), int(l))));
            }
        }
    }

    std::sort(pairs.begin(), pairs.end());
    for (const auto &pair : pairs) {
        int e1 = pair.second.first;
        int e2 = pair.second.second;
        if (edge_record[e1].compatible(edge_record[e2])) {
            if (!edge_record[e1].is_pooled() && !edge_record[e2].is_pooled()) {
                Edge_record::pool_edges(edge_record[e1], edge_record[e2]);
            }
        }
    }
#endif

    bool allzero = true;
    for (size_t k = 0; k < 4; k++) {
        double quality = 0;
        std::vector<double> sfr(mtf_width, 0);
        std::vector<double> esf(FFT_SIZE, 0);
        Snr snr;

        double ea = edge_record[k].angle;
        if (snap_to) {
            double max_dot_angle = snap_to_angle;
            double max_dot = 0;

            double angles[8] = {snap_to_angle,
                                -snap_to_angle,
                                M_PI / 2 - snap_to_angle,
                                M_PI / 2 + snap_to_angle,
                                -M_PI / 2 + snap_to_angle,
                                -M_PI / 2 - snap_to_angle,
                                M_PI + snap_to_angle,
                                M_PI - snap_to_angle};

            for (int l = 0; l < 8; l++) {
                double sa = angles[l];

                double dot = cos(edge_record[k].angle) * cos(sa) +
                             sin(edge_record[k].angle) * sin(sa);
                if (dot > max_dot) {
                    max_dot = dot;
                    max_dot_angle = sa;
                }
            }

            ea = max_dot_angle;
            edge_record[k].angle = ea;
        }

        edge_model[k]->update_location(edge_record[k].centroid,
                                       cv::Point2d(-sin(ea), cos(ea)));

        double mtf50 = 0.01;
        double edge_length = 0;
        if (!ridges_only) {
            mtf50 = compute_mtf(*edge_model[k], scansets[k], quality,
                                edge_length, sfr, esf, snr);
        }

        allzero &= fabs(mtf50) < 1e-6;

        if (mtf50 <= 1.2) { // reject mtf values above 1.2, since these are
                            // impossible, and likely to be erroneous
            std::lock_guard<std::mutex> lock(global_mutex);
            if (shared_blocks_map.find(label) == shared_blocks_map.end()) {
                shared_blocks_map[label] = block;
            }
            shared_blocks_map[label].set_mtf50_value(k, mtf50, quality);
            shared_blocks_map[label].set_normal(
                k, cv::Point2d(cos(edge_record[k].angle),
                               sin(edge_record[k].angle)));
            shared_blocks_map[label].set_sfr(k, sfr);
            shared_blocks_map[label].set_esf(k, esf);
            shared_blocks_map[label].set_snr(k, snr);
            shared_blocks_map[label].rect.centroids[k] =
                edge_record[k].centroid;
            shared_blocks_map[label].set_scanset(k, scansets[k]);
            shared_blocks_map[label].set_edge_model(k, edge_model[k]);
            shared_blocks_map[label].set_edge_valid(k);
            shared_blocks_map[label].set_edge_length(k, edge_length);
        }
    }
    if (allzero) {
        std::lock_guard<std::mutex> lock(global_mutex);
        auto it = shared_blocks_map.find(label);
        if (it != shared_blocks_map.end()) {
            shared_blocks_map.erase(it);
        }
    }
}

std::vector<Block> &Mtf_core::get_blocks()
{
    if (detected_blocks.empty()) {
        return detected_blocks;
    }

    // make a copy into an STL container if necessary
    detected_blocks.reserve(shared_blocks_map.size());
    for (const auto &[_, block] : shared_blocks_map) {
        bool allzero = true;
        for (int k = 0; k < 4 && allzero; k++) {
            if (fabs(block.get_mtf50_value(k)) > 1e-6) {
                allzero = false;
            }
        }

        if (block.valid && !allzero) {
            detected_blocks.push_back(block);
        }
    }

    return detected_blocks;
}

bool Mtf_core::extract_rectangle(const cv::Point2d &cent, int label,
                                 Mrectangle &rect)
{
    int ix = lrint(cent.x);
    int iy = lrint(cent.y);

    // skip non-convex objects with centroids outside of the image
    if (ix < 0 || ix > cl.get_width() || iy < 0 || iy > cl.get_height()) {
        return false;
    }

    Pointlist points = cl.get_boundaries().find(label)->second;

    std::vector<double> thetas(points.size(), 0);
    for (size_t i = 0; i < points.size(); i++) {
        cv::Point2d dir =
            average_dir(g, lrint(points[i].x), lrint(points[i].y));
        thetas[i] = atan2(-dir.x, dir.y); // funny ordering and signs because of
                                          // average_dir conventions
    }
    std::vector<double> main_thetas(4, 0.0);

    Peak_detector pd(thetas, 360 / 5.0);
    pd.select_best_n(main_thetas, 4);
    std::sort(main_thetas.begin(), main_thetas.end());

    rect = Mrectangle(main_thetas, thetas, points, g, label, 5.0 / 180.0 * M_PI,
                      allow_partial);

    return rect.valid;
}

bool Mtf_core::homogenous(const cv::Point2d & /*cent*/, int label,
                          const Mrectangle &rrect) const
{
    bool fail = false;
    for (size_t k = 0; k < 4 && !fail; k++) {
        const cv::Point2d &ec = rrect.centroids[k];
        const cv::Point2d &en = rrect.normals[k];

        int lcount = 0;
        for (double d = 1; d <= 20; d += 1.0) {
            cv::Point2d pos = ec - d * en;
            int x = lrint(pos.x);
            int y = lrint(pos.y);
            if (cl(x, y) == label) {
                lcount++;
            }
        }
        fail = lcount < 15;
    }
    return !fail;
}

static double angle_reduce(double x)
{
    double quad1 = fabs(fmod(x, M_PI / 2.0));
    if (quad1 > M_PI / 4.0) {
        quad1 = M_PI / 2.0 - quad1;
    }
    quad1 = quad1 / M_PI * 180;
    return quad1;
}

double Mtf_core::compute_mtf(Edge_model &edge_model,
                             const std::map<int, scanline> &scanset,
                             double &quality, double &edge_length,
                             std::vector<double> &sfr, std::vector<double> &esf,
                             Snr &snr, bool allow_peak_shift)
{
    quality = 1.0; // assume this is a good edge

    std::vector<Ordered_point> ordered;
    edge_length = 0;

    thread_local std::vector<double> fft_out_buffer(FFT_SIZE * 2);
    thread_local std::vector<double> magnitude(NYQUIST_FREQ * 4);
    thread_local std::vector<double> smoothed(NYQUIST_FREQ * 4);

    fill(fft_out_buffer.begin(), fft_out_buffer.end(), 0);

    esf_sampler->sample(edge_model, ordered, scanset, edge_length, img,
                        bayer_img);

#ifdef MDEBUG
    // The noise is added sequentially, row-by-row, which should be most similar
    // to the way noise is added in mtf_generate_rectangle
    if (fabs(noise_sd) > 1e-8) {
        std::mt19937 mt(noise_seed);
        std::normal_distribution<double> d(0.0, 65536.0 * noise_sd);
        for (auto it = ordered.begin(); it != ordered.end(); it++) {
            it->second = lrint(it->second + d(mt));
        }
    }
#endif

    sort(ordered.begin(), ordered.end());

    if (ordered.size() < 10) {
        quality = 0; // this edge is not usable in any way
        return 0;
    }

    int success = esf_model->build_esf(
        ordered, fft_out_buffer.data(), FFT_SIZE, max_dot, esf, snr,
        allow_peak_shift); // bin_fit computes the ESF derivative as part of the
                           // fitting procedure
    if (success < 0) {
        quality = poor_quality;
        LOG(INFO) << std::format("failed edge at ({}, {})",
                                 edge_model.get_centroid().x,
                                 edge_model.get_centroid().y);
        return 1.0;
    }
    afft.realfft(fft_out_buffer.data());

    double quad = angle_reduce(
        atan2(edge_model.get_direction().y, edge_model.get_direction().x));

    double n0 = fabs(fft_out_buffer[0]);
    magnitude[0] = 1.0;
    for (int i = 1; i < NYQUIST_FREQ * 4; i++) {
        magnitude[i] =
            sqrt(SQR(fft_out_buffer[i]) + SQR(fft_out_buffer[FFT_SIZE - i])) /
            n0;
    }

    if (sfr_smoothing) {
        // first, convert a "bounce" to a sign change
        double uw_phase[3] = {
            0, 0, 0}; // store the most recent three unwrapped phase values
        double m1 = -1.0;
        double current_sign = 1.0;
        int last_change = -1;
        for (int idx = 1; idx < NYQUIST_FREQ * 4 - 1; idx++) {
            if (magnitude[idx] > 1e-4) {
                uw_phase[2] = atan2(fft_out_buffer[FFT_SIZE - idx] * m1,
                                    fft_out_buffer[idx] * m1);
                int min_k = 0;
                // apply some basic phase unwrapping
                for (int k = -5; k <= 5; k++) {
                    if (abs((uw_phase[2] + k * 2 * M_PI) - uw_phase[1]) <
                        abs((uw_phase[2] + min_k * 2 * M_PI) - uw_phase[1])) {
                        min_k = k;
                    }
                }
                uw_phase[2] += min_k * 2 * M_PI;
            }
            else {
                uw_phase[2] = 0;
            }

            if (idx > 3 && fabs(uw_phase[2] - uw_phase[0]) > M_PI / 2.0 &&
                last_change < idx - 1 && magnitude[idx] < 0.5) {
                current_sign *= -1;
                last_change = idx;
            }
            magnitude[idx] *= current_sign;
            m1 *= -1;

            // shift
            uw_phase[0] = uw_phase[1];
            uw_phase[1] = uw_phase[2];
        }

        // apply narrow SG filter to lower frequencies
        fill(smoothed.begin(), smoothed.end(), 0);
        const double lf_sgw[5] = {-0.086, 0.343, 0.486, 0.343, -0.086};
        for (int idx = 0; idx < NYQUIST_FREQ * 4 - 3; idx++) {
            for (int x = -2; x <= 2; x++) {
                smoothed[idx] += magnitude[abs(idx + x)] * lf_sgw[x + 2];
            }
        }
        for (int idx = 0; idx < NYQUIST_FREQ * 4 - 3; idx++) {
            magnitude[idx] = smoothed[idx];
        }

        // perform Savitsky-Golay filtering of MTF curve
        // use filters of increasing width
        // narrow filters reduce bias in lower frequencies
        // wide filter perform the requisite strong filtering at high
        // frequencies
        const int sgh = 7;

        for (int rep = 0; rep < 3; rep++) {
            const double *sgw = 0;
            for (int idx = 0; idx < NYQUIST_FREQ * 4 - sgh; idx++) {
                if (idx < sgh) {
                    smoothed[idx] = magnitude[idx];
                }
                else {
                    smoothed[idx] = 0;
                    const int stride = 3;
                    int filter_order = std::min(5, (idx - 5) / stride);
                    sgw = savitsky_golay[filter_order];
                    for (int x = -sgh; x <= sgh; x++) {
                        // since magnitude has extra samples at the end, we can
                        // safely go past the end
                        smoothed[idx] += magnitude[idx + x] * sgw[x + sgh];
                    }
                }
            }
            for (int idx = NYQUIST_FREQ * 4 - sgh - 2; idx < NYQUIST_FREQ * 4;
                 idx++) {
                smoothed[idx] = magnitude[idx];
            }

            assert(fabs(magnitude[0] - 1.0) < 1e-6);
            assert(fabs(smoothed[0] - 1.0) < 1e-6);
            for (int idx = 0; idx < NYQUIST_FREQ * 4; idx++) {
                magnitude[idx] = smoothed[idx] / smoothed[0];
            }
        }

        // undo and sign changes we introduced above to remove a "bounce"
        for (int idx = 0; idx < NYQUIST_FREQ * 4; idx++) {
            magnitude[idx] = std::abs(magnitude[idx]);
        }
    }

    const std::vector<double> &base_mtf = esf_model->get_correction();

    double prev_freq = 0;
    double prev_val = n0;

    double mtf50 = 0; // we'll call it MTF50, but it could be anything between
                      // MTF10 and MTF90

    // first estimate mtf50 using simple linear interpolation
    bool done = false;
    int cross_idx = 0;
    for (int i = 0; i < NYQUIST_FREQ * 2 && !done; i++) {
        double mag = magnitude[i];
        mag /= base_mtf[i];
        if (prev_val > mtf_contrast && mag <= mtf_contrast) {
            // interpolate
            double m = -(mag - prev_val) * (FFT_SIZE);
            mtf50 = -(mtf_contrast - prev_val - m * prev_freq) / m;
            cross_idx = i;
            done = true;
        }
        prev_val = mag;
        prev_freq = i / double(FFT_SIZE);
    }

    if (sfr_smoothing) {
        // perform least-squares quadratic fit to compute MTF50
        if (done && cross_idx >= 5 && cross_idx < NYQUIST_FREQ * 2 - 9 - 1) {
            int hs = std::min(std::max(2, cross_idx - 9), 9);
            int npts = 2 * hs + 1;

            Eigen::MatrixXd design(npts, 3);
            Eigen::VectorXd b(npts);

            for (int r = 0; r < npts; r++) {
                double y = magnitude[abs(cross_idx - hs + r)] /
                           base_mtf[abs(cross_idx - hs + r)];
                double x = -hs + r;

                design(r, 0) = 1;
                design(r, 1) = y;
                design(r, 2) = y * y;
                b[r] = x;
            }

            Eigen::VectorXd sol = design.colPivHouseholderQr().solve(b);

            double mid = sol[0] + mtf_contrast * sol[1] +
                         mtf_contrast * mtf_contrast * sol[2];
            mid = (mid + double(cross_idx)) / double(FFT_SIZE);

            // at lower MTF50s, slowly blend in the smoothed value
            if (cross_idx > 9) {
                mtf50 = mid;
            }
            else {
                double d = (cross_idx - 5.0) / double(FFT_SIZE);
                double w = d / 8.0;
                mtf50 = (1 - w) * mtf50 + w * mid;
            }
        }
    }

    if (!done || fabs(quad) < 0.1) {
        mtf50 = 0.125;
    }
    mtf50 *= 8;

    if (absolute_sfr) {
        for (size_t i = 0; i < sfr.size(); i++) {
            sfr[i] = (n0 * magnitude[i] / base_mtf[i]) / (65536 * 2);
        }
    }
    else {
        for (size_t i = 0; i < sfr.size(); i++) {
            sfr[i] = magnitude[i] / base_mtf[i];
        }
    }

    // derate the quality of the known poor angles
    if (quad <= 1) {
        quality = poor_quality;
    }

    if (fabs(quad - 26.565051) < 1) {
        quality = medium_quality;
    }

    if (quad >= 44) {
        quality = poor_quality;
    }

    if (edge_length < 25) { // derate short edges
        quality *= poor_quality;
    }

    if (success > 0) { // possibly contaminated edge
        quality = very_poor_quality;
    }

    return mtf50;
}

void Mtf_core::process_with_sliding_window(Mrectangle &rrect)
{
    double winlen = 40; // desired length of ROI along edge direction

    const std::vector<std::vector<int>> &corner_map = rrect.corner_map;
    const std::vector<cv::Point2d> &corners = rrect.corners;

    std::vector<Mtf_profile_sample> local_samples;

    std::vector<std::pair<double, int>> dims;
    for (int k = 0; k < 4; k++) {
        dims.push_back(std::make_pair(
            norm(corners[corner_map[k][0]] - corners[corner_map[k][1]]), k));
    }
    sort(dims.begin(), dims.end());

    int v1 = dims[3].second;
    int v2 = dims[2].second;

    std::vector<cv::Point2d> b(2);
    std::vector<cv::Point2d> d(2);

    b[0] = corners[corner_map[v1][0]];
    d[0] = corners[corner_map[v1][1]] - corners[corner_map[v1][0]];

    b[1] = corners[corner_map[v2][0]];
    d[1] = corners[corner_map[v2][1]] - corners[corner_map[v2][0]];

    if (dims[2].first < winlen) {
        LOG(INFO)
            << "Rectangle not really long enough for sliding mode. Skipping.";
        return;
    }

    // first, refine edge orientation using (most) of the edge
    for (int side = 0; side < 1; side++) {
        cv::Point2d dir(d[side]);
        double edge_len = norm(dir);
        cv::Point2d start(b[side]);
        dir *= 1.0 / edge_len;

        cv::Point2d n(-d[side].y, d[side].x);
        n *= 1.0 / norm(n);
        double cross = dir.x * n.y - dir.y * n.x;
        if (cross < 0) {
            n = -n;
        }

        cv::Point2d end(b[side] + edge_len * dir);

        cv::Point2d tl(start + 16 * n);
        cv::Point2d br(end - 16 * n);
        if (tl.x > br.x) {
            std::swap(tl.x, br.x);
        }
        if (tl.y > br.y) {
            std::swap(tl.y, br.y);
        }

        cv::Point2d tl2(start - 16 * n);
        cv::Point2d br2(end + 16 * n);
        if (tl2.x > br2.x) {
            std::swap(tl2.x, br2.x);
        }
        if (tl2.y > br2.y) {
            std::swap(tl2.y, br2.y);
        }
        tl.x = std::min(tl.x, tl2.x);
        tl.y = std::min(tl.y, tl2.y);
        br.x = std::max(br.x, br2.x);
        br.y = std::max(br.y, br2.y);

        Edge_record edge_record;

        // clamp ROi to image
        tl.x = std::max(1.0, tl.x);
        tl.y = std::max(1.0, tl.y);
        br.x = std::min(img.cols - 1.0, br.x);
        br.y = std::min(img.rows - 1.0, br.y);

        for (double y = tl.y; y < br.y; y += 1.0) {
            for (double x = tl.x; x < br.x; x += 1.0) {
                cv::Point2d p(x, y);
                cv::Point2d gd = p - b[side];
                double pdot = gd.ddot(dir);

                // TODO: making the window narrow here helps a bit ...
                if (std::abs(gd.ddot(n)) < 12 && pdot > 5 &&
                    pdot < (edge_len - 5)) {
                    int iy = lrint(y);
                    int ix = lrint(x);
                    edge_record.add_point(x, y, fabs(g.grad_x(ix, iy)),
                                          fabs(g.grad_y(ix, iy)));
                }
            }
        }

        edge_record.reduce(); // we can now move the ROI if we have to ...

        cv::Point2d nd(sin(edge_record.angle), -cos(edge_record.angle));
        if (nd.ddot(dir) < 0) {
            nd = -nd;
        }

        // use angle to set d[], keep orientation
        d[side] = nd * edge_len;
        // TODO: should we update b[] as well?
    }

    // now process all the windows
    for (int side = 0; side < 2; side++) {
        const double buffer = 5; // pixels to ignore near edge of block?
        double steplen = 4;
        cv::Point2d dir(d[side]);
        double edge_len = cv::norm(dir);

        int steps = std::floor((edge_len - winlen) / steplen) + 1;

        if (samples_per_edge != 0) {
            steps = std::max(2, std::min(steps, samples_per_edge));
            steplen = (edge_len - winlen) / double(steps - 1);
        }

        cv::Point2d start(b[side]);
        dir *= 1.0 / edge_len;

        for (int step = 0; step < steps; step++) {
            cv::Point2d n(-d[side].y, d[side].x);
            n *= 1.0 / norm(n);
            double cross = dir.x * n.y - dir.y * n.x;
            if (cross < 0) {
                n = -n;
            }

            cv::Point2d end(b[side] + (step * steplen + winlen) * dir);

            cv::Point2d tl(start + 16 * n);
            cv::Point2d br(end - 16 * n);
            if (tl.x > br.x) {
                std::swap(tl.x, br.x);
            }
            if (tl.y > br.y) {
                std::swap(tl.y, br.y);
            }

            cv::Point2d tl2(start - 16 * n);
            cv::Point2d br2(end + 16 * n);
            if (tl2.x > br2.x) {
                std::swap(tl2.x, br2.x);
            }
            if (tl2.y > br2.y) {
                std::swap(tl2.y, br2.y);
            }
            tl.x = std::min(tl.x, tl2.x);
            tl.y = std::min(tl.y, tl2.y);
            br.x = std::max(br.x, br2.x);
            br.y = std::max(br.y, br2.y);

            tl.x = std::max(1.0, tl.x);
            tl.y = std::max(1.0, tl.y);
            br.x = std::min(img.cols - 1.0, br.x);
            br.y = std::min(img.rows - 1.0, br.y);

            std::map<int, scanline> scanset;
            Edge_record edge_record;

            double min_p = 1e50;
            double max_p = -1e50;

            for (double y = tl.y; y < br.y; y += 1.0) {
                for (double x = tl.x; x < br.x; x += 1.0) {
                    cv::Point2d p(x, y);
                    cv::Point2d gd = p - b[side];
                    double pdot = gd.ddot(dir);
                    cv::Point2d ld = p - start;
                    double ldot = (ld.ddot(dir)) / winlen;

                    if (pdot > buffer && pdot < (edge_len - buffer) &&
                        ldot > 0 && ldot < 1) {
                        int iy = lrint(y);
                        int ix = lrint(x);
                        if (std::abs(gd.ddot(n)) < 14) {
                            edge_record.add_point(x, y, fabs(g.grad_x(ix, iy)),
                                                  fabs(g.grad_y(ix, iy)));
                        }

                        min_p = std::min(min_p, pdot);
                        max_p = std::max(max_p, pdot);
                    }
                }
            }

            edge_record.reduce(); // we can now move the ROI if we have to ...
                                  // (usually we iterate a bit here...)

#if 1
            cv::Point2d newcent(edge_record.centroid.x, edge_record.centroid.y);
            edge_record.clear();

            for (double y = tl.y; y < br.y; y += 1.0) {
                for (double x = tl.x; x < br.x; x += 1.0) {
                    cv::Point2d p(x, y);
                    cv::Point2d wd = p - newcent;
                    double dot = wd.ddot(n);

                    cv::Point2d ld = p - start;
                    double ldot = (ld.ddot(dir)) / winlen;

                    cv::Point2d gd = p - b[side];
                    double pdot = gd.ddot(dir);

                    if (pdot > buffer && pdot < (edge_len - buffer) &&
                        ldot > 0 && ldot < 1) {
                        int iy = lrint(y);
                        int ix = lrint(x);

                        if (std::abs(dot) < 12) {
                            edge_record.add_point(x, y,
                                                  std::abs(g.grad_x(ix, iy)),
                                                  std::abs(g.grad_y(ix, iy)));
                        }

                        if (std::abs(dot) < (max_dot + 1)) {
                            auto it = scanset.find(iy);
                            if (it == scanset.end()) {
                                scanline sl(ix, ix);
                                scanset.insert(std::make_pair(iy, sl));
                            }
                            if (ix < scanset[iy].start) {
                                scanset[iy].start = ix;
                            }
                            if (ix > scanset[iy].end) {
                                scanset[iy].end = ix;
                            }
                        }
                    }
                }
            }

            edge_record.reduce();

#endif

            cv::Vec3b &color = od_img.at<cv::Vec3b>(
                lrint(edge_record.centroid.y), lrint(edge_record.centroid.x));
            color[0] = 255;
            color[1] = 255;
            color[2] = 0;

            double quality = 0;
            double edge_length = 0;
            std::vector<double> sfr(mtf_width, 0);
            std::vector<double> esf(FFT_SIZE, 0);
            Snr snr;
            Edge_model edge_model(
                edge_record.centroid,
                cv::Point2d(-sin(edge_record.angle), cos(edge_record.angle)));
            double mtf50 = compute_mtf(edge_model, scanset, quality,
                                       edge_length, sfr, esf, snr);
            // TODO: there is no way to store the snr object of a non-block data
            // point

            if (mtf50 < 1.0 && quality > very_poor_quality) {
                local_samples.push_back(Mtf_profile_sample(
                    edge_record.centroid, mtf50, edge_record.angle, quality));
            }

            start = b[side] + (step + 1) * steplen * dir;
        }
    }

    if (local_samples.size() > 0) {
        std::lock_guard<std::mutex> lock(global_mutex);
        samples.insert(samples.end(), local_samples.begin(),
                       local_samples.end());
        sliding_edges.push_back(std::make_pair(b[0], d[0]));
        sliding_edges.push_back(std::make_pair(b[1], d[1]));
    }
}

// NB: 'bounds' is interpreted as (xmin, ymin, xmax, ymax)
void Mtf_core::process_image_as_roi(const cv::Rect &bounds,
                                    cv::Point2d handle_a, cv::Point2d handle_b)
{
    single_roi_mode = true; // TODO: harmless potential race condition

    Rect_roi roi(handle_a, handle_b, max_dot);
    if (handle_a.x > -1e10 && handle_b.x > -1e10 && handle_a.y > -1e10 &&
        handle_b.y > -1e10) {
        roi.activate();
    }

    std::map<int, scanline> scanset;
    Edge_record er;
    for (int row = bounds.y; row < bounds.height; row++) {
        for (int col = bounds.x; col < bounds.width; col++) {
            if (!roi.inside(col, row))
                continue;

            er.add_point(col, row, fabs(g.grad_x(col, row)),
                         fabs(g.grad_y(col, row)));

            if (scanset.find(row) == scanset.end()) {
                scanset[row] = scanline(col, col);
            }
            scanset[row].start = std::min(col, scanset[row].start);
            scanset[row].end = std::max(col, scanset[row].end);
        }
    }

    // ROI is the whole image.
    er.reduce();
    cv::Point2d cent(er.centroid);
    LOG(INFO) << "ER reduce grad estimate: " << er.angle / M_PI * 180;

    // scan the ROI to identify outliers, i.e., pixels far from the
    // edge with large gradient values (indicative of contamination of the ROI)
    cv::Point2d mean_grad(cos(er.angle), sin(er.angle));
    cv::Point2d edge_direction(-sin(er.angle), cos(er.angle));
    std::vector<std::vector<double>> binned_gradient(max_dot * 4 + 1);
    for (auto it = scanset.begin(); it != scanset.end(); ++it) {
        int y = it->first;
        for (int x = it->second.start; x <= it->second.end; ++x) {
            cv::Point2d d((x)-cent.x, (y)-cent.y);
            int idot = lrint(d.ddot(mean_grad) * 2) + max_dot;
            if (idot >= 0 && idot < (int)binned_gradient.size()) {
                binned_gradient[idot].push_back(g.grad_magnitude(x, y));
            }
        }
    }

    for (auto &b : binned_gradient) {
        std::ranges::sort(b);
    }

    std::vector<int> skiplist;
    for (const auto &[y, scanline] : scanset) {
        for (int x = scanline.start; x <= scanline.end; ++x) {
            cv::Point2d d((x)-cent.x, (y)-cent.y);
            double dot = d.ddot(mean_grad);
            if (std::abs(dot) <= 2) {
                continue;
            }

            int idot = lrint(dot * 2) + max_dot;
            if (idot < 0 || idot >= (int)binned_gradient.size() ||
                binned_gradient[idot].size() <= 3) {
                continue;
            }

            double upper = binned_gradient[idot][(int)std::floor(
                binned_gradient[idot].size() * 0.95)];
            if (binned_gradient[idot].back() -
                    binned_gradient[idot][binned_gradient[idot].size() - 2] <
                0.001) {
                continue;
            }

            if (g.grad_magnitude(x, y) >= upper) {
                skiplist.push_back(y * img.cols + x);
            }
        }
    }

    std::ranges::sort(skiplist);

    // iterate the edge record to both centre the edge, as well as refine the
    // edge orientation
    cent = er.centroid;
    cv::Point2d normal(cos(er.angle), sin(er.angle));
    scanset.clear();
    er.clear();

    auto em =
        std::make_shared<Edge_model>(cent, cv::Point2d(-normal.y, normal.x));

    double diag_len =
        sqrt(bounds.height * bounds.height + bounds.width * bounds.width);
    em->hint_point_set_size((int)ceil(diag_len + 2),
                            (int)ceil(2 * diag_len + 4),
                            (int)ceil(2 * max_dot + 4));

    for (int row = bounds.y; row < bounds.height; row++) {
        for (int col = bounds.x; col < bounds.width; col++) {
            if (!roi.inside(col, row)) {
                continue;
            }

            cv::Point2d p(col, row);
            cv::Point2d d = p - cent;
            if (std::abs(d.ddot(normal)) < 12) {
                int idx = row * img.cols + col;
                if (!binary_search(skiplist.begin(), skiplist.end(), idx)) {
                    er.add_point(col, row, fabs(g.grad_x(col, row)),
                                 fabs(g.grad_y(col, row)));
                    em->add_point(col, row, g.grad_magnitude(col, row), 12);
                }
            }

            if (scanset.find(row) == scanset.end()) {
                scanset[row] = scanline(col, col);
            }
            scanset[row].start = std::min(col, scanset[row].start);
            scanset[row].end = std::max(col, scanset[row].end);
        }
    }
    er.reduce();
    em->estimate_ridge();

    if (snap_to) {
        double max_dot_angle = snap_to_angle;
        double max_dot = 0;

        double angles[8] = {snap_to_angle,
                            -snap_to_angle,
                            M_PI / 2 - snap_to_angle,
                            M_PI / 2 + snap_to_angle,
                            -M_PI / 2 + snap_to_angle,
                            -M_PI / 2 - snap_to_angle,
                            M_PI + snap_to_angle,
                            M_PI - snap_to_angle};

        for (int l = 0; l < 8; l++) {
            double sa = angles[l];

            double dot = cos(er.angle) * cos(sa) + sin(er.angle) * sin(sa);
            if (dot > max_dot) {
                max_dot = dot;
                max_dot_angle = sa;
            }
        }

        er.angle = max_dot_angle;
    }

    LOG(INFO) << std::format(
        "updated: ER reduce grad estimate: {}, centroid ({}, {}) -> ({}, {})",
        er.angle / M_PI * 180, cent.x, cent.y, er.centroid.x, er.centroid.y);

    em->update_location(er.centroid,
                        cv::Point2d(-sin(er.angle), cos(er.angle)));

    double quality;
    double edge_length = 0;
    std::vector<double> sfr(mtf_width, 0);
    std::vector<double> esf(FFT_SIZE, 0);
    Snr snr;
    double mtf50 =
        compute_mtf(*em, scanset, quality, edge_length, sfr, esf, snr, true);

    // add a block with the correct properties ....
    if (mtf50 <= 1.2) {
        Mrectangle rect;
        rect.centroids[0] = er.centroid;
        rect.thetas[0] = er.angle;
        Block block(rect);
        block.centroid = er.centroid; // just in case
        block.set_mtf50_value(0, mtf50, quality);
        block.set_normal(0, cv::Point2d(cos(er.angle), sin(er.angle)));

        block.set_sfr(0, sfr);
        block.set_esf(0, esf);
        block.set_snr(0, snr);
        block.set_line_deviation(0, em->line_deviation());
        block.set_scanset(0, scanset);
        block.set_edge_model(0, em);
        block.set_edge_valid(0);
        block.set_edge_length(0, edge_length);

        for (int k = 1; k < 4; k++) {
            block.set_mtf50_value(k, 1.0, 0.0);
            block.set_sfr(k, std::vector<double>(NYQUIST_FREQ * 2, 0));
            block.set_esf(k, std::vector<double>(FFT_SIZE / 2, 0));
        }

        shared_blocks_map[1] =
            block; // TODO: major concurrency issue here, must fix!
        detected_blocks.push_back(block);
    }
}

void Mtf_core::process_manual_rois(const std::string &roi_fname)
{
    FILE *fin = fopen(roi_fname.c_str(), "rt");

    if (!fin) {
        LOG(ERROR) << "Could not open --roi-file " << roi_fname;
        return;
    }

    // TODO: we could process all these ROIs in parallel, after loading them
    while (!feof(fin)) {
        cv::Point2d handle_a;
        cv::Point2d handle_b;
        size_t nread = fscanf(fin, "%lf %lf %lf %lf", &handle_a.x, &handle_a.y,
                              &handle_b.x, &handle_b.y);

        if (nread == 4) {
            printf("going to process %.1lf %.1lf -> %.1lf %.1lf\n", handle_a.x,
                   handle_a.y, handle_b.x, handle_b.y);

            Rect_roi roi(handle_a, handle_b, max_dot);
            process_image_as_roi(roi.bounds(img), handle_a, handle_b);
        }
    }

    fclose(fin);
}

std::vector<Mtf_profile_sample> &Mtf_core::get_samples() { return samples; }

void Mtf_core::set_absolute_sfr(bool val) { absolute_sfr = val; }

void Mtf_core::set_sfr_smoothing(bool val) { sfr_smoothing = val; }

void Mtf_core::set_sliding(bool val)
{
    sliding = val;
    find_fiducials = true;
}

void Mtf_core::set_samples_per_edge(int s) { samples_per_edge = s; }

void Mtf_core::set_snap_angle(double angle)
{
    snap_to = true;
    snap_to_angle = angle;
}

void Mtf_core::set_find_fiducials(bool val) { find_fiducials = val; }

void Mtf_core::set_ridges_only(bool b) { ridges_only = b; }

void Mtf_core::use_full_sfr() { mtf_width = 4 * NYQUIST_FREQ; }

void Mtf_core::set_mtf_contrast(double contrast)
{
    mtf_contrast = std::max(0.01, std::min(contrast, 0.9));
}

double Mtf_core::get_mtf_contrast() const { return mtf_contrast; }

size_t Mtf_core::num_objects() { return valid_obj.size(); }

void Mtf_core::set_esf_model(std::unique_ptr<EsfModel> &&model)
{
    esf_model = std::move(model);
}

std::unique_ptr<EsfModel> &Mtf_core::get_esf_model() { return esf_model; }

void Mtf_core::set_esf_model_alpha_parm(double alpha)
{
    esf_model->set_alpha(alpha);
}

const std::vector<std::pair<cv::Point2d, cv::Point2d>> &
Mtf_core::get_sliding_edges() const
{
    return sliding_edges;
}

EsfSampler *Mtf_core::get_esf_sampler() const { return esf_sampler; }

Bayer::cfa_pattern_t Mtf_core::get_cfa_pattern() const { return cfa_pattern; }

bool Mtf_core::is_single_roi() const { return single_roi_mode; }

void Mtf_core::set_allow_partial(bool value) { allow_partial = value; }
