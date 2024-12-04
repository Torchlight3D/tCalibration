#include "focussurface.h"

#include <format>

#include <glog/logging.h>

#include "mtf50edgequalityrating.h"

Focus_surface::Focus_surface(std::vector<Mtf_profile_sample> &data, int order_n,
                             int order_m, Distance_scale &distance_scale)
    : data(data), order_n(order_n), order_m(order_m), maxy(-1e50), maxx(-1e50)
{
    double miny = 1e50;
    for (size_t i = 0; i < data.size(); i++) {
        maxx = std::max(std::abs(data[i].p.x), maxx);
        maxy = std::max(std::abs(data[i].p.y), maxy);
        miny = std::min(std::abs(data[i].p.y), miny);
    }

    LOG(INFO) << std::format("chart extents: y:({}, {}) mm, x:({}) mm", miny,
                             maxy, maxx);

    // TODO: this loop is a great candidate for OpenMP, but some care must
    // be taken to protect the (few) shared structures (like "peaks")

    double min_fit_err = 1e50;
    Eigen::VectorXd best_sol;
    std::vector<Sample> dummy_data;
    Ratpoly_fit best_fit(dummy_data, 5, 5);

    FILE *ffout = fopen("peaks.txt", "wt");

    maxy = 136; // empirical?

    std::vector<Sample> peak_pts;
    // |15 to 110| in steps of 2.5, width=5 ??
    for (int s = -1; s <= 1; s += 2) {
        // for (double d=max(10.0, 1.2*miny*cscale); d <= 0.95*maxy*cscale;
        // d += 1.0) {
        for (double d = 10.0; d <= 0.9 * maxy; d += 2.0) {
            double midy = s * d;

            double mean_x = 0;
            double wsum = 0;

            // TODO: this may become a little slow with many
            std::vector<Sample> pts_row;
            for (size_t i = 0; i < data.size(); i++) {
                // points
                double dy = midy - data[i].p.y;
                // at least 5 mm from centre of chart
                if (std::abs(dy) < 15 && std::abs(data[i].p.y) > 5 &&
                    std::abs(data[i].p.x) < 175 &&
                    std::abs(data[i].p.y) < 136) {
                    // sdev of 3 mm in y direction
                    double yw = exp(-dy * dy / (2 * 3 * 3));
                    pts_row.push_back(
                        Sample(data[i].p.x, data[i].mtf, yw,
                               data[i].quality <= poor_quality ? 0.25 : 1));
                    mean_x += pts_row.back().weight * data[i].p.y;
                    wsum += pts_row.back().weight;
                }
            }

            if (pts_row.size() < 3 * 14) {
                continue;
            }

            // now filter out the really bad outliers
            constexpr int sh = 4;
            constexpr double sgw[] = {-21 / 231.0, 14 / 231.0, 39 / 231.0,
                                      54 / 231.0,  59 / 231.0, 54 / 231.0,
                                      39 / 231.0,  14 / 231.0, -21 / 231.0};
            std::sort(pts_row.begin(), pts_row.end());

            // just pretend our samples are equally spaced
            std::vector<Sample> ndata;
            for (size_t i = sh; i < pts_row.size() - sh; i++) {
                double val = 0;
                for (int w = -sh; w <= sh; w++) {
                    val += sgw[w + sh] * pts_row[i + w].y;
                }

                if (std::abs(val - pts_row[i].y) / val < 0.05) {
                    ndata.push_back(pts_row[i]);
                }
            }
            pts_row = ndata;

            Eigen::VectorXd sol;
            double lpeak;

            mean_x /= wsum;

            Ratpoly_fit cf(pts_row, 4, order_m);
            sol = rpfit(cf, true, true);
            while (cf.order_n > 1 && cf.order_m > 0 && cf.has_poles(sol)) {
                cf.order_m--;
                sol = rpfit(cf, true, true);
                if (cf.has_poles(sol)) {
                    cf.order_n--;
                    sol = rpfit(cf, true, true);
                }
            }
            if (cf.has_poles(sol)) {
                // no solution without poles, give up, skip this sample?
                LOG(INFO)
                    << "Warning: no viable RP fit. Skipping curve centred at y="
                    << mean_x;
                continue;
            }

            double err = cf.evaluate(sol);
            lpeak = cf.peak(sol);

            double merr = err / double(pts_row.size());
            if (merr < min_fit_err) {
                LOG(INFO) << std::format("min fit err {} at dist {}", merr,
                                         midy);
                min_fit_err = merr;
            }

            if (std::abs(midy) < 20 &&
                std::abs(merr - min_fit_err) / merr < 1) {
                best_sol = sol;
                dummy_data = pts_row;
                best_fit.order_n = cf.order_n;
                best_fit.order_m = cf.order_m;
                best_fit.xs_min = cf.xs_min;
                best_fit.xs_scale = cf.xs_scale;
                best_fit.ysf = cf.ysf;
            }

            peak_pts.push_back(Sample(mean_x, lpeak, 1, 1.0));
            ridge_peaks.push_back(cv::Point2d(lpeak, mean_x));

            fprintf(ffout, "%lf %lf\n", mean_x, lpeak);
        }
    }

    fclose(ffout);

    if (peak_pts.size() < 10) {
        LOG(ERROR) << "Not enough peak points to construct peak focus curve.";
        return;
    }

    Ratpoly_fit cf(peak_pts, 2, 2);
    cf.base_value = 1;
    cf.pscale = 0;
    Eigen::VectorXd sol = rpfit(cf, true, true);
    while (cf.order_m > 0 && cf.has_poles(sol)) {
        cf.order_m--;
        LOG(INFO) << "reducing order_m to %d" << cf.order_m;
        sol = rpfit(cf, true, true);
    }
    if (cf.has_poles(sol)) {
        // no solution without poles, give up, skip this sample?
        LOG(INFO) << "Warning: no viable RP fit to fpeaks data";
    }

#if 1
    // attempt IRLS step
    std::vector<double> iweights(peak_pts.size(), 0);
    for (size_t i = 0; i < peak_pts.size(); i++) {
        iweights[i] = peak_pts[i].yweight;
    }
    double prev_err = 1e50;
    for (int iter = 0; iter < 50; iter++) {
        double errsum = 0;
        double wsum = 0;
        for (size_t i = 0; i < peak_pts.size(); i++) {
            double y = cf.rpeval(sol, cf.scale(peak_pts[i].x)) / cf.ysf;
            double e = std::abs(y - peak_pts[i].y);
            peak_pts[i].yweight = iweights[i] / std::max(0.0001, e);

            if (iter > 3 && e > 20) {
                LOG(INFO) << std::format("outlier at {} mm, suppressing", e);
                peak_pts[i].yweight = 0;
            }

            double w = peak_pts[i].yweight;
            errsum += e * w;
            wsum += w;
        }
        errsum /= wsum;
        LOG(INFO) << std::format("iter {} err: {}", iter, errsum);
        Eigen::VectorXd oldsol = sol;
        sol = rpfit(cf, true, true);
        if (iter > 10 && (prev_err - errsum) / prev_err < 0.0001) {
            LOG(INFO) << "bailing out at iter " << iter;
            if (errsum > prev_err) {
                LOG(INFO) << "reverting to older solution";
                sol = oldsol;
            }
            break;
        }
        prev_err = errsum;
    }
#endif

    // now perform some bootstrapping to obtain bounds on the peak focus
    // curve:
    std::vector<double> mc_pf;
    std::map<double, std::vector<double>> mc_curve;
    for (int iters = 0; iters < 30; iters++) {
        std::vector<Sample> sampled_peak_pts;
        for (int j = 0; j < peak_pts.size() * 0.5; j++) {
            int idx =
                (int)floor(peak_pts.size() * double(rand()) / double(RAND_MAX));
            sampled_peak_pts.push_back(peak_pts[idx]);
        }
        Ratpoly_fit mc_cf(sampled_peak_pts, cf.order_n, cf.order_m);
        mc_cf.base_value = 1;
        mc_cf.pscale = 0;
        Eigen::VectorXd mc_sol = rpfit(mc_cf, true, true);
        mc_pf.push_back(mc_cf.rpeval(mc_sol, 0) / mc_cf.ysf);

        for (double y = -maxy; y < maxy; y += 10) {
            double x = mc_cf.rpeval(mc_sol, mc_cf.scale(y)) / mc_cf.ysf;
            mc_curve[y].push_back(x);
        }
    }
    std::ranges::sort(mc_pf);

    for (auto &[y, curve] : mc_curve) {
        std::ranges::sort(curve);
        ridge_p05.push_back(cv::Point2d(curve[0.05 * curve.size()], y));
        ridge_p95.push_back(cv::Point2d(curve[0.95 * curve.size()], y));
    }

    for (double y = -maxy; y < maxy; y += 1) {
        double x = cf.rpeval(sol, cf.scale(y)) / cf.ysf;
        ridge.push_back(cv::Point2d(x, y));
    }

    double x_inter = cf.rpeval(sol, 0) / cf.ysf;

    int x_inter_index =
        lower_bound(mc_pf.begin(), mc_pf.end(), x_inter) - mc_pf.begin();
    LOG(INFO) << "x_inter percentile: " << x_inter_index * 100. / mc_pf.size();
    LOG(INFO) << std::format("x_inter 95%% confidence interval: [{}, {}]",
                             mc_pf[0.05 * mc_pf.size()],
                             mc_pf[0.95 * mc_pf.size()]);

    distance_scale.estimate_depth_world_coords(x_inter, 0.0, focus_peak);
    distance_scale.estimate_depth_world_coords(mc_pf[0.05 * mc_pf.size()], 0.0,
                                               focus_peak_p05);
    distance_scale.estimate_depth_world_coords(mc_pf[0.95 * mc_pf.size()], 0.0,
                                               focus_peak_p95);

    LOG(INFO) << "focus_mm (on chart x axis) " << x_inter;

    LOG(INFO) << "focus_plane " << focus_peak;
    LOG(INFO) << std::format("fp_interval: [{}, {}]", focus_peak_p05,
                             focus_peak_p95);

    double curve_min = 1e50;
    double curve_max = -1e50;
    for (size_t i = 0; i < dummy_data.size(); i++) {
        curve_min = std::min(curve_min, dummy_data[i].x);
        curve_max = std::max(curve_max, dummy_data[i].x);
    }
    FILE *profile = fopen("nprofile.txt", "wt");
    double curve_peak = best_fit.peak(best_sol);
    double curve_offset = curve_peak - x_inter;
    LOG(INFO) << std::format("curve peak = {}, x_inter = {}", curve_peak,
                             x_inter);
    for (double cx = curve_min; cx <= curve_max; cx += 1) {
        double mtf =
            best_fit.rpeval(best_sol, best_fit.scale(cx)) / best_fit.ysf;
        double depth = 0;
        distance_scale.estimate_depth_world_coords(cx - curve_offset, 0.0,
                                                   depth);
        fprintf(profile, "%lf %lf\n", depth, mtf);
    }
    fclose(profile);
}

Eigen::VectorXd Focus_surface::rpfit(Ratpoly_fit &cf, bool scale, bool refine)
{
    const std::vector<Sample> &pts_row = cf.get_data();

    if (scale) {
        double xmin = 1e50;
        double xmax = -1e50;
        double ysf = 0;
        for (size_t i = 0; i < pts_row.size(); i++) {
            xmin = std::min(xmin, pts_row[i].x);
            xmax = std::max(xmax, pts_row[i].x);
            ysf = std::max(ysf, std::abs(pts_row[i].y));
        }
        cf.xs_min = 0.5 * (xmin + xmax);
        cf.xs_scale = 2.0 / (xmax - xmin);
        cf.ysf = ysf = 1.0 / ysf;
    }

    int tdim = cf.dimension();
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(tdim, tdim);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(tdim);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(tdim);

    Eigen::VectorXd sol;

    for (int iter = 0; iter < 1; iter++) {
        cov.setZero();
        b.setZero();
        a.setZero();

        for (size_t i = 0; i < pts_row.size(); i++) {
            const Sample &sp = pts_row[i];
            double w = sp.weight * sp.yweight;
            a[0] = 1 * w;
            double prod = cf.scale(sp.x); // top poly
            for (int j = 1; j <= cf.order_n; j++) {
                a[j] = w * cf.cheb(j, prod);
            }
            // bottom poly
            for (int j = 1; j <= cf.order_m; j++) {
                a[j + cf.order_n] = cf.cheb(j, prod) * w * sp.y * cf.ysf;
            }

            for (int col = 0; col < tdim; col++) {
                for (int icol = 0; icol < tdim; icol++) {
                    // build covariance of design matrix : A'*A
                    cov(col, icol) += a[col] * a[icol];
                }
                // build rhs of system : A'*b
                b[col] += cf.base_value * a[col] * sp.y * cf.ysf * w;
            }
        }

        for (int col = cf.order_n + 1; col < cov.cols(); col++) {
            cov.col(col) = -cov.col(col);
        }

        sol = cov.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    }

    // now perform non-linear optimization

    if (refine) {
        sol = cf.gauss_newton_armijo(sol);
    }

    return sol;
}

double Focus_surface::evaluate(Eigen::VectorXd &) { return 0; }

int Focus_surface::dimension() { return (order_n + 1 + order_m); }

double Focus_surface::softclamp(double x, double lower, double upper, double p)
{
    double s = (x - lower) / (upper - lower);
    if (s > p) {
        return 1.0 / (1.0 + exp(-3.89182 * s));
    }
    return s < 0 ? 0 : s;
}
