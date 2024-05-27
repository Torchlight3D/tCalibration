#include "distance_scale.h"

#include <opencv2/calib3d.hpp>

#include <tCore/Math>

#include "bundle.h"
#include "fiducial_positions.h"
#include "five_point_focal_length_radial_distortion.h"
#include "logger.h"
#include "point_helpers.h"

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using Matrix34d = Eigen::Matrix<double, 3, 4>;

namespace {

int n_choose_k(int n, int k)
{
    if (k == n) {
        return 1;
    }
    if (k == 1) {
        return n;
    }
    return n_choose_k(n - 1, k) + n_choose_k(n - 1, k - 1);
}

void sub_enumerate_n_choose_k(int n, int k, std::vector<std::vector<int>> &all,
                              int &j, std::vector<int> &a, int i)
{
    a[i] = n - 1;
    if (i == k - 1) {
        all[j] = a;
        j++;
        return;
    }

    for (int c = n - 1; c > 0; c--) {
        sub_enumerate_n_choose_k(c, k, all, j, a, i + 1);
    }
}

void enumerate_n_choose_k(int n, int k, std::vector<std::vector<int>> &arr)
{
    int j = 0;
    std::vector<int> a(k, 0);
    for (int c = n; c >= k; c--) {
        sub_enumerate_n_choose_k(c, k, arr, j, a, 0);
    }
}

int compute_edit_distance(const std::vector<int> &source,
                          const std::vector<int> &target)
{
    const uint16_t infinity = ~0;

    // build matrix
    int nrows = source.size() + 1;
    int ncols = target.size() + 1;

    std::vector<uint16_t> matrix(2 * ncols);

    // initialise first row
    for (int col = 0; col < ncols; col++) {
        matrix[0 * ncols + col] = col;
    }

    for (int row = 1; row < nrows; row++) {
        // initialise this first element of the current row
        matrix[1 * ncols + 0] = row;
        for (int col = 1; col < ncols; col++) {
            unsigned int dist;
            unsigned int mindist = infinity;

            dist = matrix[col - 1];
            if ((source[row - 1] != target[col - 1])) {
                dist += 2;
            }

            if (dist < mindist) {
                mindist = dist;
            }

            dist = matrix[col] + 1;
            if (dist < mindist) {
                mindist = dist;
            }

            dist = matrix[ncols + col - 1] + 1;
            if (dist < mindist) {
                mindist = dist;
            }

            matrix[1 * ncols + col] = mindist;
        }

        // somewhat slow, perhaps, but fast enough!
        memcpy(matrix.data(), ((uint16_t *)matrix.data()) + ncols,
               ncols * sizeof(uint16_t));
    }

    uint32_t rval = matrix[0 * ncols + ncols - 1];

    return rval;
}
} // namespace

bool t_intersect(double &pix, double &piy, double v1x, double v1y, double d1x,
                 double d1y, double v2x, double v2y, double d2x, double d2y)
{
    double denom = (d2y * d1x - d2x * d1y);

    if (std::abs(denom) < 1e-11) {
        // this happens when the lines are parallel
        // the caller handles this correctly by not
        // adding an additional intersection point
        return false;
    }

    double u = (d2x * (v1y - v2y) - d2y * (v1x - v2x)) / denom;

    pix = v1x + u * d1x;
    piy = v1y + u * d1y;

    return true;
}

Distance_scale::Distance_scale()
    : chart_scale(1.0),
      largest_block_index(-1),
      focal_length(10000),
      fiducials_found(false)
{
}

void Distance_scale::construct(Mtf_core &mtf_core, bool pose_based,
                               cv::Rect *dimension_correction,
                               double user_focal_ratio,
                               const std::string &correspondence_file)
{
    user_provided_focal_ratio = user_focal_ratio;

    // At least two, or nothing
    for (size_t i{0}; i < std::max(size_t{0}, mtf_core.ellipses.size() - 1);
         i++) {
        Ellipse_detector &e = mtf_core.ellipses[i];
        for (size_t j = i + 1; j < mtf_core.ellipses.size(); j++) {
            Ellipse_detector &f = mtf_core.ellipses[j];
            double cdist = sqrt(SQR(f.centroid_x - e.centroid_x) +
                                SQR(f.centroid_y - e.centroid_y));
            if (cdist < std::min(e.minor_axis, f.minor_axis)) {
                // keep only the larger fiducial
                if (e.major_axis > f.major_axis) {
                    f.valid = false;
                }
                else {
                    e.valid = false;
                }
            }
        }
    }

    int zcount = 0;
    for (const auto &e : mtf_core.ellipses) {
        if (e.valid && e.code == 0) {
            zero.x += 0.5 * e.centroid_x;
            zero.y += 0.5 * e.centroid_y;
            zcount++;
        }
    }

    if (zcount == 2 && pose_based) {
        std::map<int, Ellipse_detector *> by_code;
        cv::Point2d first;
        cv::Point2d last;
        zcount = 0;
        double max_fiducial_diameter = 0;
        for (auto &detector : mtf_core.ellipses) {
            if (!detector.valid) {
                continue;
            }

            if (detector.code == 0) {
                if (zcount == 0) {
                    first = detector.centroid();
                }
                else {
                    last = detector.centroid();
                }
                zcount++;
            }
            else {
                if (by_code.find(detector.code) != by_code.end()) {
                    logger.debug("collision: code %d found at both [%lf "
                                 "%lf] and [%lf %lf]\n",
                                 detector.code, detector.centroid_x,
                                 detector.centroid_y,
                                 by_code[detector.code]->centroid_x,
                                 by_code[detector.code]->centroid_y);
                }
                by_code[detector.code] = &detector;
            }
            max_fiducial_diameter =
                std::max(detector.major_axis, max_fiducial_diameter);
        }

        logger.debug("absolute centre: (%lf, %lf)\n", zero.x, zero.y);
        transverse = normalize(first - last);
        logger.debug("transverse vector: (%lf, %lf)\n", transverse.x,
                     transverse.y);
        // sign of transverse unknown at this stage
        longitudinal = cv::Point2d(-transverse.y, transverse.x);
        logger.debug("longitudinal vector: (%lf, %lf)\n", longitudinal.x,
                     longitudinal.y);

        // minimum of 5 unique fiducials plus one spare plus the zeros
        if (by_code.size() > 6) {
            // TODO: move this to mtf core implementation?
            // find the four fiducials closest to zero
            std::vector<std::pair<double, int>> by_distance_from_zero;
            for (const auto &e : mtf_core.ellipses) {
                if (!e.valid) {
                    continue;
                }

                // We dont know how to tell the two zeros appart, so skip them
                if (e.code == 0) {
                    continue;
                }

                double distance = norm(e.centroid() - zero);
                by_distance_from_zero.push_back(
                    std::make_pair(distance, e.code));
            }

            std::sort(by_distance_from_zero.begin(),
                      by_distance_from_zero.end());

            // TODO: we should add a check here to discard fiducials that
            // are much further than expected something like a ratio of the
            // distance between the zeros ?
            logger.debug("%s", "distance to first four fiducials: ");
            std::vector<int> first_four;
            for (int i = 0; i < 4; i++) {
                logger.debug("%lf ", by_distance_from_zero[i].first);
                first_four.push_back(by_distance_from_zero[i].second);
            }

            logger.debug("%s\n", "");
            std::sort(first_four.begin(), first_four.end());
            logger.debug("first four fiducials are: %d %d %d %d\n",
                         first_four[0], first_four[1], first_four[2],
                         first_four[3]);

            int min_edit_dist = 100;
            int min_fid_coding = 3;
            for (size_t fid_coding = 0;
                 fid_coding < fiducial_code_mapping.size(); fid_coding++) {
                std::vector<int> ref = {fiducial_code_mapping[fid_coding][2],
                                        fiducial_code_mapping[fid_coding][3],
                                        fiducial_code_mapping[fid_coding][4],
                                        fiducial_code_mapping[fid_coding][5]};

                int dist = compute_edit_distance(ref, first_four);
                if (dist < min_edit_dist) {
                    min_edit_dist = dist;
                    min_fid_coding = fid_coding;
                }
            }
            logger.debug(
                "best matching fid coding is %d with edit distance %d\n",
                min_fid_coding, min_edit_dist);
            // if edit_dist => 4, then we might pick the wrong coding, so
            // this should be an abort?

            // use knowledge of the first four fiducials to orient the
            // transverse direction
            std::vector<std::pair<int, double>> candidates{
                {fiducial_code_mapping[min_fid_coding][2], 1.0},
                {fiducial_code_mapping[min_fid_coding][3], 1.0},
                {fiducial_code_mapping[min_fid_coding][4], -1.0},
                {fiducial_code_mapping[min_fid_coding][5], -1.0}};
            for (const auto &c : candidates) {
                if (by_code.find(c.first) != by_code.end()) {
                    cv::Point2d dir = by_code[c.first]->centroid() - zero;
                    dir *= c.second;
                    if (dir.x * transverse.x + dir.y * transverse.y < 0) {
                        transverse = -transverse;
                        longitudinal = cv::Point2d(-transverse.y, transverse.x);
                    }

                    // One sample is enough
                    break;
                }
            }

            if (dimension_correction) {
                // principal point relative to original uncropped image
                prin = cv::Point2d(
                    dimension_correction->width / 2.0 - dimension_correction->x,
                    dimension_correction->height / 2.0 -
                        dimension_correction->y);
                // image dimensions of uncropped image should be used
                img_scale = std::max(dimension_correction->height,
                                     dimension_correction->width);
            }
            else {
                prin = cv::Point2d(mtf_core.img.cols / 2.0,
                                   mtf_core.img.rows / 2.0);
                img_scale = std::max(mtf_core.img.rows, mtf_core.img.cols);
            }

            page_scale_factor = fiducial_scale_factor[min_fid_coding];

            std::vector<Vector2d> ba_img_points;
            std::vector<Vector3d> ba_world_points;
            logger.debug("principal point = (%lf, %lf)\n", prin.x, prin.y);
            for (const auto &[_, e] : by_code) {
                if (!e->valid) {
                    continue;
                }

                // We dont know how to tell the two zeros appart, so just skip
                if (e->code == 0) {
                    continue;
                }

                int main_idx = -1;
                for (size_t i = 0; i < n_fiducials && main_idx == -1; i++) {
                    if (fiducial_code_mapping[min_fid_coding][i] == e->code) {
                        main_idx = i;
                    }
                }

                ba_img_points.emplace_back(
                    (e->centroid_x - prin.x) / img_scale,
                    (e->centroid_y - prin.y) / img_scale);
                ba_world_points.emplace_back(
                    main_fiducials[main_idx].rcoords.y *
                        fiducial_scale_factor[min_fid_coding] *
                        fiducial_position_scale_factor[min_fid_coding].second,
                    main_fiducials[main_idx].rcoords.x *
                        fiducial_scale_factor[min_fid_coding] *
                        fiducial_position_scale_factor[min_fid_coding].first,
                    1.);
                fid_img_points.push_back(e->centroid());
                fid_world_points.push_back(cv::Point3d(
                    ba_world_points.back()[0], ba_world_points.back()[1],
                    ba_world_points.back()[2]));
            }

            std::vector<Matrix34d> projection_matrices;
            std::vector<std::vector<double>> radial_distortions;
            cv::Mat rot_matrix = cv::Mat(3, 3, CV_64FC1);
            cv::Mat rod_angles = cv::Mat(3, 1, CV_64FC1);
            Eigen::MatrixXd P;

            struct Solution
            {
                double bpe = 0.;
                Eigen::MatrixXd proj = Eigen::MatrixXd::Zero();
                double distort = 0.;
                std::vector<int> inlier_list = {};
                double f = 1.;

                bool operator<(const Solution &b) const { return bpe < b.bpe; }
            };

            std::vector<Solution> solutions;
            std::vector<Vector2d> feature_points(5);
            std::vector<Vector3d> world_points(5);

            enumerate_combinations(ba_img_points.size(), 5);

            size_t most_inliers = 0;
            // this should work unless extreme distortion is present?
            double inlier_threshold = max_fiducial_diameter;
            double global_bpr = 1e50;
            for (size_t ri = 0; ri < combinations.size(); ri++) {
                for (int i = 0; i < 5; i++) {
                    feature_points[i] = ba_img_points[combinations[ri][i]];
                    world_points[i] = ba_world_points[combinations[ri][i]];
                    world_points[i][2] +=
                        +0.000001 * (i + 1) * (i % 2 == 0 ? -1 : 1);
                }

                theia::FivePointFocalLengthRadialDistortion(
                    feature_points, world_points,
                    1, // number of distortion parms
                    &projection_matrices, &radial_distortions);

                for (size_t i{0}; i < projection_matrices.size(); i++) {
                    auto &K = projection_matrices[i];

                    // Only consider solutions in front of camera
                    if (K(2, 3) > 0) {
                        K *= -1;
                    }

                    // check if Rodriques produces the same rotation matrix
                    double r1n = (K.block<1, 3>(0, 0)).norm();
                    double r3n = (K.block<1, 3>(2, 0)).norm();
                    double w = sqrt((r3n * r3n) / (r1n * r1n));

                    Matrix3d RM = K.block<3, 3>(0, 0);
                    RM.row(2) /= w;
                    RM /= RM.row(0).norm();

                    for (int rr = 0; rr < 3; rr++) {
                        for (int cc = 0; cc < 3; cc++) {
                            rot_matrix.at<double>(rr, cc) = RM(rr, cc);
                        }
                    }
                    cv::Rodrigues(rot_matrix, rod_angles);
                    cv::Rodrigues(rod_angles, rot_matrix);

                    double rot_err = 0;
                    for (int rr = 0; rr < 3; rr++) {
                        for (int cc = 0; cc < 3; cc++) {
                            rot_err += std::abs(rot_matrix.at<double>(rr, cc) -
                                                RM(rr, cc));
                        }
                    }

                    if (rot_err < 0.01 && w > 0.01 &&
                        w < 18) { // roughly allow 2 mm to 3600 mm focal
                                  // lengths on a 36 mm wide sensor
                        std::vector<double> residuals;

                        Matrix3d RMM(RM);
                        RMM.row(2) *= w;
                        Vector3d TV = K.col(3) / K.block<1, 3>(0, 0).norm();

                        // TODO: technically, we could re-run the five-point
                        // solver with eccentricity correction but since
                        // this should have very little effect, rather leave
                        // that for the final bundle adjustment

                        // COP in world coordinates
                        Vector3d ec_cop = TV;
                        // undo focal length scaling of z-component
                        ec_cop[2] /= w;

                        std::vector<int> inliers;
                        for (size_t i = 0; i < ba_img_points.size(); i++) {
                            Vector2d bp =
                                (RMM * ba_world_points[i] + TV).hnormalized();

                            // NB: note the sign of the distortion
                            double rad = 1. - radial_distortions[i][0] *
                                                  bp.squaredNorm();
                            bp /= rad;

                            if (auto err = (ba_img_points[i] - bp).norm();
                                err * img_scale < inlier_threshold * 0.5) {
                                residuals.push_back(err);
                                inliers.push_back(i);
                            }
                        }

                        // do not waste time on outlier-dominated solutions
                        if (inliers.size() < 4) {
                            continue;
                        }

                        double bpr = 0;
                        double wsum = 0;
                        for (size_t i = 0; i < inliers.size(); i++) {
                            double weight = ba_world_points[inliers[i]].norm();

                            // TODO: outer points have more weight. bad idea, or
                            // a way to emphasize distortion?
                            bpr += residuals[i] * residuals[i] * weight;
                            wsum += weight;
                        }
                        bpr = sqrt(bpr / wsum) * img_scale;

                        most_inliers = std::max(most_inliers, inliers.size());

                        solutions.push_back(Solution(
                            bpr, projection_matrices[i],
                            -radial_distortions[i][0], inliers, 1.0 / w));
                        if (bpr < global_bpr) {
                            global_bpr = bpr;
                            logger.debug(
                                "%lu[%lu]: rotation error: %lf, bpr=%lf "
                                "pixels, f=%lf pixels, inliers=%lu "
                                "(#best=%lu), distortion=%lf\n",
                                i, ri, rot_err, bpr, img_scale / w,
                                inliers.size(), solutions.size(),
                                -radial_distortions[i][0]);
                        }
                    }
                }
                projection_matrices.clear();
                radial_distortions.clear();
            }
            combinations.clear(); // discard the combinations

            if (solutions.empty()) {
                logger.error("%s\n", "Error: No solutions to camera "
                                     "calibration found. Aborting");
                fiducials_found = false;
                return;
            }

            // now try to characterize the "mean solution" amongst the
            // better-performing solutions
            std::vector<double> focal_ratio_list;
            std::vector<double> bpe_list;
            for (const auto &sol : solutions) {
                if (sol.inlier_list.size() == most_inliers) {
                    focal_ratio_list.push_back(sol.f);
                    bpe_list.push_back(sol.bpe);
                }
            }
            logger.debug(
                "total solutions: %lu, max inlier solutions: %lu (#%d)\n",
                solutions.size(), bpe_list.size(), most_inliers);
            sort(focal_ratio_list.begin(), focal_ratio_list.end());
            sort(bpe_list.begin(), bpe_list.end());

            double bpe_threshold = bpe_list[0.35 * bpe_list.size()];
            double focal_ratio_min =
                focal_ratio_list[0.15 * focal_ratio_list.size()];
            double focal_ratio_max =
                focal_ratio_list[0.85 * focal_ratio_list.size()];
            logger.debug("bpe threshold=%lf, fr min=%lf, fr max=%lf\n",
                         bpe_threshold, focal_ratio_min, focal_ratio_max);

            std::vector<double> fr_keepers;
            for (const auto &sol : solutions) {
                if (sol.inlier_list.size() == most_inliers &&
                    sol.bpe <= bpe_threshold && sol.f >= focal_ratio_min &&
                    sol.f <= focal_ratio_max) {
                    fr_keepers.push_back(sol.f);
                }
            }
            sort(fr_keepers.begin(), fr_keepers.end());

            double median_fr = fr_keepers[fr_keepers.size() / 2];

            double median_distortion = -10;
            std::vector<Solution> kept_solutions;
            if (user_focal_ratio > 0) {
                std::vector<double> distortion_list;
                median_fr = 1e50;
                for (const auto &sol : solutions) {
                    if (sol.inlier_list.size() == most_inliers &&
                        std::abs(sol.f - user_focal_ratio) <
                            std::abs(median_fr - user_focal_ratio)) {
                        median_fr = sol.f;
                    }
                    if (sol.inlier_list.size() == most_inliers) {
                        distortion_list.push_back(sol.distort);
                    }
                }
                sort(distortion_list.begin(), distortion_list.end());
                median_distortion = distortion_list[distortion_list.size() / 2];
            }

            for (const auto &sol : solutions) {
                if (sol.inlier_list.size() == most_inliers &&
                    sol.f == median_fr) {
                    kept_solutions.push_back(sol);
                }
            }
            solutions = kept_solutions;

            logger.debug("Retained %lu promising solutions\n",
                         solutions.size());
            for (const auto &sol : solutions) {
                logger.debug("\t solution with bpe = %lf, dist=%le, "
                             "#inliers=%lu, fr=%lf\n",
                             sol.bpe, sol.distort, sol.inlier_list.size(),
                             sol.f);
            }
            double w = 0;

            int min_idx = 0;

            P = solutions[min_idx].proj;
            distortion = solutions[min_idx].distort;

            std::vector<int> &inliers = solutions[min_idx].inlier_list;

            double r1n = (P.block<1, 3>(0, 0)).norm();
            double r3n = (P.block<1, 3>(2, 0)).norm();
            w = sqrt((r3n * r3n) / (r1n * r1n));
            focal_length = 1.0 / w;

            // remove the arbitrary scaling factor
            P /= P.block<1, 3>(0, 0).norm();
            // remove focal length from last row
            P.row(2) /= w;

            Matrix3d RM = P.block<3, 3>(0, 0);
            Vector3d Pcop = P.col(3);

            if (user_focal_ratio > 0) {
                w = 1.0 / user_focal_ratio;
                distortion = median_distortion;
            }

            logger.debug("focal length = %lf (times max sensor dim), or "
                         "%lf pixels\n",
                         1.0 / w, img_scale / w);

            for (int rr = 0; rr < 3; rr++) {
                for (int cc = 0; cc < 3; cc++) {
                    rot_matrix.at<double>(rr, cc) = RM(rr, cc);
                }
            }
            cv::Rodrigues(rot_matrix, rod_angles);

            std::vector<Vector2d> inlier_feature_points(inliers.size());
            std::vector<Vector3d> inlier_world_points(inliers.size());

            for (size_t k = 0; k < inliers.size(); k++) {
                inlier_feature_points[k] = ba_img_points[inliers[k]];
                inlier_world_points[k] = ba_world_points[inliers[k]];
            }
            fiducial_code_set = min_fid_coding;

            if (correspondence_file.length() > 1) {
                FILE *corr_fout = fopen(correspondence_file.c_str(), "wt");
                fprintf(corr_fout, "# MTF Mapper fiducial positions, "
                                   "output format version 2\n");
                fprintf(corr_fout, "# column 1: fiducial x location (image "
                                   "space, pixels)\n");
                fprintf(corr_fout, "# column 2: fiducial y location (image "
                                   "space, pixels)\n");
                fprintf(corr_fout,
                        "# column 3: fiducial x location (chart space, mm)\n");
                fprintf(corr_fout,
                        "# column 4: fiducial y location (chart space, mm)\n");
                for (size_t k = 0; k < inlier_feature_points.size(); k++) {
                    fprintf(corr_fout, "%.3lf %.3lf %.3lf %.3lf\n",
                            inlier_feature_points[k][0] * img_scale + prin.x,
                            inlier_feature_points[k][1] * img_scale + prin.y,
                            inlier_world_points[k][0],
                            inlier_world_points[k][1]);
                }
                fclose(corr_fout);
            }

            Bundle_adjuster ba(inlier_feature_points, inlier_world_points, Pcop,
                               rod_angles, distortion, w,
                               5.0 * fiducial_scale_factor[min_fid_coding],
                               img_scale);
            if (user_focal_ratio < 0) {
                ba.focal_lower = focal_ratio_min / 1.1;
                ba.focal_upper = focal_ratio_max * 1.1;
                // TODO: a bit crude ...
                ba.focal_mode_constraint =
                    acos(std::abs(RM(2, 2))) / M_PI * 180 < 15 ? 1.0 : 0.0;
            }
            else {
                ba.focal_lower = user_focal_ratio / 1.1;
                ba.focal_upper = user_focal_ratio * 1.1;
                ba.focal_mode_constraint =
                    acos(std::abs(RM(2, 2))) / M_PI * 180 < 15 ? 1.0 : 0.0;
            }

            logger.debug("focal mode = %lf, f=%lf\n", ba.focal_mode_constraint,
                         1 / w);
            ba.solve();
            ba.unpack(rotation, translation, distortion, w);
            logger.debug("after: f=%lf, limits=(%lf, %lf)\n", 1 / w,
                         ba.focal_lower, ba.focal_upper);
            bundle_rmse = ba.evaluate(ba.best_sol) * img_scale;
            logger.debug("solution %d has rmse=%lf\n", min_idx, bundle_rmse);

            double prev_rmse = bundle_rmse;
            // take another stab, in case NM stopped at a point that
            // was not really a minimum
            // TODO: in theory, we could repeat this until RMSE stabilizes?
            bool improved = false;
            do {
                ba.solve();
                ba.unpack(rotation, translation, distortion, w);
                bundle_rmse = ba.evaluate(ba.best_sol) * img_scale;
                logger.debug("next solution %d has rmse=%lf\n", min_idx,
                             bundle_rmse);
                improved = false;
                if (prev_rmse - bundle_rmse > 1e-4) {
                    improved = true;
                }
                prev_rmse = bundle_rmse;
            } while (improved);

            // calculate bundle rmse without constraints
            bundle_rmse = ba.evaluate(ba.best_sol, 0.0) * img_scale;
            logger.debug("final rmse (without penalty) = %lf\n", bundle_rmse);

            focal_length = 1.0 / w;

            // prepare for backprojection
            Matrix3d K;
            K << 1, 0, 0, 0, 1, 0, 0, 0, 1.0 / focal_length;

            invP = (K * rotation).inverse();

            // prepare for forward and backward projection transforms
            fwdP = K * rotation;
            fwdT = Vector3d(translation[0], translation[1],
                            translation[2] / focal_length);

            cop = Vector3d(translation[0], translation[1],
                           translation[2] / focal_length);
            cop = -invP * cop;

            centre_depth = backproject(zero.x, zero.y)[2];

            logger.debug("ultimate f=%lf centre_depth=%lf distortion=%le\n",
                         focal_length * img_scale, centre_depth, distortion);
            logger.debug("%s\n", "rotation=");
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    logger.debug("%12.8lf ", rotation(r, c));
                }
                logger.debug("%s\n", "");
            }
            logger.debug("translation=[%lf %lf %lf]\n", translation[0],
                         translation[1], translation[2]);

            roll_angle = asin(rotation(0, 1) / sqrt(1 - SQR(rotation(0, 2))));
            yaw_angle = asin(-rotation(0, 2));
            pitch_angle = asin(rotation(1, 2) / sqrt(1 - SQR(rotation(0, 2))));
            logger.debug("Tait-Bryan angles: %lf %lf %lf\n",
                         roll_angle / M_PI * 180, yaw_angle / M_PI * 180,
                         pitch_angle / M_PI * 180);

            fiducials_found = true;
            logger.debug("Chart z-angle=%.1lf degrees\n",
                         pitch_angle / M_PI * 180);
            logger.debug("Chart y-angle=%.1lf degrees\n",
                         yaw_angle / M_PI * 180);
        }

        // construct a distance scale
    }
    else {
        logger.debug("%s\n", "Warning: Manual focus profile chart mode "
                             "requested, but central dots not found.");
        const std::vector<Block> &blocks = mtf_core.get_blocks();

        double delta_1 = 1;
        double delta_2 = 1;
        std::vector<std::pair<double, int>> by_size;

        if (blocks.size() > 0) {
            // find largest block
            for (size_t i = 0; i < blocks.size(); i++) {
                by_size.push_back(std::make_pair(blocks[i].get_area(), int(i)));
            }
            sort(by_size.begin(), by_size.end());

            const int sstart = 5;
            delta_1 = by_size[by_size.size() - 1].first /
                      by_size[by_size.size() - sstart - 1].first;
            delta_2 = by_size[by_size.size() - sstart - 1].first /
                      by_size[by_size.size() - sstart - 2].first;
        }

        if (delta_1 / delta_2 > 80) {
            largest_block_index = by_size.back().second;
            const Block &lblock = blocks[by_size.back().second];
            // we have a clear largest block. now determine its orientation
            std::vector<double> xp;
            std::vector<double> yp;
            for (size_t i = 0; i < blocks.size(); i++) {
                xp.push_back(blocks[i].centroid.x);
                yp.push_back(blocks[i].centroid.y);
            }
            sort(xp.begin(), xp.end());
            sort(yp.begin(), yp.end());
            double idx_x =
                (find(xp.begin(), xp.end(), lblock.centroid.x) - xp.begin()) /
                double(xp.size());
            double idx_y =
                (find(yp.begin(), yp.end(), lblock.centroid.y) - yp.begin()) /
                double(yp.size());
            logger.debug("xfrac=%lf, yfrac=%lf\n", idx_x, idx_y);

            cv::Point2d median(xp[xp.size() / 2], yp[yp.size() / 2]);

            // orientations relative to chart, not image
            int top_i = lblock.get_edge_index(Block::TOP);
            int bot_i = lblock.get_edge_index(Block::BOTTOM);
            int left_i = lblock.get_edge_index(Block::LEFT);
            int right_i = lblock.get_edge_index(Block::RIGHT);
            if (std::abs(idx_x - 0.5) < std::abs(idx_y - 0.5)) {
                // outer rows arranged in columns
                if (std::abs(lblock.get_edge_centroid(top_i).y - median.y) <
                    std::abs(lblock.get_edge_centroid(bot_i).y - median.y)) {
                    // chart is upside down
                    std::swap(top_i, bot_i);
                    std::swap(left_i, right_i);
                }
            }
            else {
                // outer rows arranged in rows
                std::swap(bot_i, right_i);
                std::swap(top_i, left_i);

                if (std::abs(lblock.get_edge_centroid(top_i).x - median.x) <
                    std::abs(lblock.get_edge_centroid(bot_i).x - median.x)) {
                    // chart is upside down
                    std::swap(top_i, bot_i);
                    std::swap(left_i, right_i);
                }
            }
            transverse = normalize(lblock.get_edge_centroid(right_i) -
                                   lblock.get_edge_centroid(left_i));
            longitudinal = normalize(lblock.get_edge_centroid(bot_i) -
                                     lblock.get_edge_centroid(top_i));
            zero = lblock.get_edge_centroid(bot_i);
            double block_width = cv::norm(lblock.get_edge_centroid(right_i) -
                                          lblock.get_edge_centroid(left_i));

            // assume central block is 62 mm wide
            chart_scale = 62.0 / block_width;
            logger.debug("Warning: choosing (potentially) poor chart scale "
                         "of %lf mm/pixel\n",
                         chart_scale);
        }
        else {
            logger.debug("%s\n", "Warning: Could not identify largest "
                                 "block, choosing poor defaults");
            chart_scale = 0.15;
            zero = cv::Point2d(932, 710);
            transverse = cv::Point2d(1, 0);
            longitudinal = cv::Point2d(0, 1);
        }
        logger.debug("zero: %lf %lf\n", zero.x, zero.y);
        logger.debug("transverse: %lf %lf\nlongitudinal: %lf %lf\n",
                     transverse.x, transverse.y, longitudinal.x,
                     longitudinal.y);
    }
}

cv::Point2d Distance_scale::normalize_img_coords(double pixel_x,
                                                 double pixel_y) const
{
    double xs = (pixel_x - prin.x) / img_scale;
    double ys = (pixel_y - prin.y) / img_scale;

    double rhat = sqrt(xs * xs + ys * ys);
    double r = 0;

    if (rhat > 1e-10) {
        double pa = distortion * rhat;
        double pb = -1;
        double pc = rhat;

        double q = -0.5 * (pb + sgn(pb) * sqrt(pb * pb - 4 * pa * pc));
        double r1 = q / pa;
        double r2 = pc / q;

        if (r1 > 0 && r2 > 0) {
            r = std::min(r1, r2);
        }
        else {
            if (r1 <= 0) {
                r = r2;
            }
            else {
                r = r1;
            }
        }

        xs = xs * rhat / r;
        ys = ys * rhat / r;
    }

    return {xs, ys};
}

Eigen::Vector3d Distance_scale::backproject(double pixel_x,
                                            double pixel_y) const
{
    cv::Point2d ic = normalize_img_coords(pixel_x, pixel_y);

    Vector3d dv(ic.x, ic.y, 1.0);
    dv = invP * dv;
    dv /= dv.norm();

    double s = (1 - cop[2]) / dv[2];
    Vector3d ip = cop + s * dv;

    // now we have ip in world coordinates, but we actually want it in
    // camera coordinates

    // z-axis sign?
    ip = rotation * ip +
         Vector3d(translation[0], translation[1], -translation[2]);

    return ip;
}

void Distance_scale::estimate_depth_img_coords(double pixel_x, double pixel_y,
                                               double &depth) const
{
    Vector3d bp = backproject(pixel_x, pixel_y);
    depth = bp[2] - centre_depth;
}

void Distance_scale::estimate_depth_world_coords(double world_x, double world_y,
                                                 double &depth) const
{
    Vector3d bp = rotation * Vector3d(world_x, world_y, 0) +
                  Vector3d(translation[0], translation[1], -translation[2]);

    depth = bp[2] - centre_depth;
}

cv::Point2d Distance_scale::estimate_world_coords(double pixel_x,
                                                  double pixel_y) const
{
    cv::Point2d ic = normalize_img_coords(pixel_x, pixel_y);

    Vector3d dv(ic.x, ic.y, 1.0);
    dv = invP * dv;
    dv /= dv.norm();

    double s = (1 - cop[2]) / dv[2];
    Vector3d ip = cop + s * dv;

    // ip[2] (=z) will always be in the plane
    return {ip[0], ip[1]};
}

cv::Point2d Distance_scale::world_to_image(double world_x, double world_y,
                                           double world_z) const
{
    Vector2d bp =
        (fwdP * Vector3d(world_x, world_y, world_z) + fwdT).hnormalized();

    double rad = 1 + distortion * bp.squaredNorm();
    bp /= rad;
    bp *= img_scale;

    bp[0] += prin.x;
    bp[1] += prin.y;

    return {bp[0], bp[1]};
}

double Distance_scale::get_normal_angle_z() const
{
    return tl::math::radToDeg(std::acos(std::abs(rotation(2, 2))));
}

double Distance_scale::get_normal_angle_y() const
{
    // choosing the max here means that it does not matter
    // what the chart orientation is relative to the sensor,
    // i.e., a 90-degree rotation of the chart (or sensor)
    // is already factored out
    return tl::math::radToDeg(std::acos(
        std::max(std::abs(rotation(0, 1)), std::abs(rotation(1, 1)))));
}

void Distance_scale::enumerate_combinations(int n, int k)
{
    int t = n_choose_k(n, k);

    combinations = std::vector<std::vector<int>>(t, std::vector<int>(k, 0));

    enumerate_n_choose_k(n, k, combinations);
}
