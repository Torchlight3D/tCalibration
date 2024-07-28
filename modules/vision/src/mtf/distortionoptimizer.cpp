#include "distortionoptimizer.h"

#include <cmath>
#include <format>

#include <glog/logging.h>

namespace {

// we can safely reduce the number of ridge points because of the
// serial corellation we expect from the algorithm that extracted
// the ridge (in edge_model)
std::vector<cv::Point2d> downsample(const std::vector<cv::Point2d>& pts)
{
    constexpr size_t kMinRidgeLength = 30;
    if (pts.size() < kMinRidgeLength) {
        return pts;
    }

    size_t step = 3;
    if (pts.size() > 3 * kMinRidgeLength) {
        step = 5;
    }

    std::vector<cv::Point2d> out_pts;
    for (size_t i = 0; i < pts.size(); i += step) {
        out_pts.push_back(pts[i]);
    }

    return out_pts;
}

} // namespace

Distortion_optimizer::Distortion_optimizer(const std::vector<Block>& in_blocks,
                                           const cv::Point2d& prin)
    : prin(prin), max_val(1, 1), maxrad(0)
{
    // corners are at radius 1
    radius_norm = sqrt(prin.x * prin.x + prin.y * prin.y);

    // weight each edge according to its orientation relative to the principal
    // point
    for (const auto& block : in_blocks) {
        for (int k = 0; k < 4; k++) {
            if (block.get_ridge(k).size() > 2) {
                cv::Point2d cent = block.get_edge_centroid(k);
                cv::Point2d normal = block.get_normal(k);
                cv::Point2d radial(cent.x - prin.x, cent.y - prin.y);
                if (norm(radial) > 1e-6) {
                    radial *= 1.0 / (norm(radial));
                    double w = fabs(radial.ddot(normal));

                    // edge angle < 80 degrees w.r.t. radial direction
                    if (w > 0.17365) {
                        double wrad = norm(cent - prin) / radius_norm;
                        ridges.push_back(Ridge(downsample(block.get_ridge(k)),
                                               block.get_edge_centroid(k),
                                               block.get_normal(k), w));
                        maxrad = std::max(wrad, maxrad);
                        max_val.x = std::max(max_val.x, fabs(cent.x - prin.x));
                        max_val.y = std::max(max_val.y, fabs(cent.y - prin.y));
                    }
                }
            }
        }
    }

    LOG(INFO) << std::format("Kept {} of {} edges", ridges.size(),
                             4 * in_blocks.size());
    LOG(INFO) << "maxrad = " << maxrad;

    // pack initial parameters
    Eigen::VectorXd init(2);

    init[0] = 0;
    init[1] = 0;

    best_sol = initial = init;
}

cv::Point2d Distortion_optimizer::get_max_val() const { return max_val; }

void Distortion_optimizer::solve()
{
    Eigen::VectorXd scale(best_sol.size());

    scale << 1e-2, 1e-2;

    double initial_err = evaluate(best_sol, 0.0);
    LOG(INFO) << "initial distortion rmse: " << initial_err;

    // TODO: this could be parallelised
    double minerr = 1e30;
    Eigen::VectorXd v(2);
    for (double k1 = -2; k1 < 2; k1 += 0.5) {
        v[0] = k1;
        for (double k2 = -2; k2 < 2; k2 += 0.5) {
            v[1] = k2;
            if (!model_not_invertible(v)) {
                double err = evaluate(v, 0.0);
                if (err < minerr) {
                    minerr = err;
                    best_sol = v;
                }
            }
        }
    }

    nelder_mead_failed = false;
    seed_simplex(best_sol, scale);
    best_sol = iterate(1e-6);

    LOG(INFO) << std::format("initial coeffs: {} {}", best_sol[0], best_sol[1]);

    double opt_err = evaluate(best_sol, 0.0);
    LOG(INFO) << "opt distortion rmse: " << opt_err;

    // now compute outliers ...
    std::vector<float> residuals;
    for (const auto& edge : ridges) {
        residuals.push_back(edge.residual);
    }

    if (residuals.size() > 4) {
        // side effect: residuals is now sorted
        double mc = medcouple(residuals);
        double outlier_threshold = residuals[residuals.size() * 0.75] +
                                   1.5 * exp(4 * mc) *
                                       (residuals[residuals.size() * 0.75] -
                                        residuals[residuals.size() * 0.25]);
        size_t outlier_count = 0;
        for (auto& edge : ridges) {
            if (edge.residual >= outlier_threshold) {
                outlier_count++;
                edge.weight = 0;
            }
        }
        LOG(INFO) << std::format(
            "medcouple={}, outlier threshold={}, min={}, median={}, max={}", mc,
            outlier_threshold, residuals.front(),
            residuals[residuals.size() / 2], residuals.back());
        LOG(INFO) << std::format("Found {} outliers out of {} values",
                                 outlier_count, ridges.size());

        if (outlier_count > 0 &&
            ridges.size() - outlier_count > ridges.size() * 0.7) {
            // better safe than sorry
            minerr = 1e30;
            for (double k1 = -2; k1 < 2; k1 += 0.5) {
                v[0] = k1;
                for (double k2 = -2; k2 < 2; k2 += 0.5) {
                    v[1] = k2;
                    if (!model_not_invertible(v)) {
                        double err = evaluate(v, 0.0);
                        if (err < minerr) {
                            minerr = err;
                            best_sol = v;
                        }
                    }
                }
            }

            LOG(INFO) << "restarting after outlier suppression:";
            seed_simplex(best_sol, scale);
            best_sol = iterate(1e-6);
        }
    }

    // restart, just to make sure
    seed_simplex(best_sol, scale);
    best_sol = iterate(1e-6);

    double final_err = evaluate(best_sol, 0.0);
    LOG(INFO) << "final distortion rmse: " << final_err;
}

// two-parameter division model; also works when k2(v[1]) == 0
cv::Point2d Distortion_optimizer::inv_warp(const cv::Point2d& p,
                                           const Eigen::VectorXd& v)
{
    double px = (p.x - prin.x);
    double py = (p.y - prin.y);

    const double rd = sqrt((px) * (px) + (py) * (py)) / radius_norm;
    const double r2 = rd * rd;
    double ru = 1 + (v[0] + v[1] * r2) * r2;

    px = px / ru + prin.x;
    py = py / ru + prin.y;

    return {px, py};
}

double Distortion_optimizer::model_not_invertible(const Eigen::VectorXd& v)
{
    const double k1 = v[0];
    const double k2 = v[1];
    const double r1 = std::max(0.5, std::min(maxrad * 1.05, 1.0));
    const double r12 = r1 * r1;
    const double r14 = r12 * r12;

    constexpr double kThreshold = 3.;
    if (fabs(k1) > kThreshold || fabs(k2) > kThreshold) {
        return 1.0;
    }

    if (k1 * r12 <= -2) {
        return 1.0;
    }

    if (k1 * r12 < 2) {
        if (k2 * r14 <= -1 - k1 * r12)
            return 1.0;
        if (k2 * r14 >= (1 - k1 * r12) / 3.0)
            return 1.0;
        return 0.0;
    }

    // k1r12 >= 2
    if (k2 * r14 <= -1 - k1 * r12) {
        return 1.0;
    }

    if (k2 * r14 >= -k1 * k1 * r14 / 12.0) {
        return 1.0;
    }

    return 0.0;
}

double Distortion_optimizer::medcouple(std::vector<float>& x)
{
    sort(x.begin(), x.end());

    double xm = x[x.size() / 2];
    double xscale = 2 * x.back();

    std::vector<double> zplus;
    std::vector<double> zminus;

    for (size_t i = x.size() - 1; i > 0 && x[i] >= xm; i--) {
        zplus.push_back((x[i] - xm) / xscale);
    }
    for (size_t i = 0; i < x.size() && x[i] <= xm; i++) {
        zminus.push_back((x[i] - xm) / xscale);
    }

    size_t p = zplus.size();
    size_t q = zminus.size();

    std::vector<float> h(q * p);
    for (size_t i = 0; i < p; i++) {
        for (size_t j = 0; j < q; j++) {
            float& hval = h[j * p + i];

            if (zplus[i] == zminus[j]) {
                int64_t dv = int64_t(p) - 1 - int64_t(i) - int64_t(j);
                hval = dv < 0 ? -1 : (dv == 0 ? 0 : 1);
            }
            else {
                hval = (zplus[i] + zminus[j]) / (zplus[i] - zminus[j]);
            }
        }
    }

    nth_element(h.begin(), h.begin() + h.size() / 2, h.end());
    return h[h.size() / 2];
}

double weight_penalty(double x)
{
    x = fabs(x);
    if (x > 0.05)
        return 0.01;
    if (x < 0.005)
        return x;

    constexpr double c = 0.05 / 0.045;
    constexpr double m = -1.0 / 0.045;
    return x * ((x - 0.005) * m + c + 0.01);
}

double Distortion_optimizer::evaluate(const Eigen::VectorXd& v, double penalty)
{
    double count = 0;
    double merr = 0;
    for (auto& edge : ridges) {
        cv::Point2d cent = inv_warp(edge.centroid, v);
        cv::Point2d dir(-edge.normal.y, edge.normal.x);

        std::vector<cv::Point2d> ridge(edge.ridge.size());

        // estimate edge length to establish scale
        cv::Point2d first_point = inv_warp(edge.ridge.front(), v);
        cv::Point2d last_point = inv_warp(edge.ridge.back(), v);
        double scale = std::max(fabs((first_point - cent).ddot(dir)),
                                fabs((last_point - cent).ddot(dir)));

        double sum_x = 0;
        double sum_y = 0;
        double sum_xx = 0;
        double sum_xy = 0;
        for (size_t ri = 0; ri < edge.ridge.size(); ri++) {
            cv::Point2d p = inv_warp(edge.ridge[ri], v);

            cv::Point2d d = p - cent;
            double perp = d.ddot(edge.normal);
            double par = d.ddot(dir) / scale;

            // keep the refined values
            ridge[ri] = cv::Point2d(par, perp);

            sum_x += par;
            sum_y += perp;
            sum_xx += par * par;
            sum_xy += par * perp;
        }

        double n = edge.ridge.size();
        double beta =
            (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        double alpha = (sum_y - beta * sum_x) / n;

        // for now, do another pass to calculate the standard error
        // we can probably speed this up by using the fit directly
        std::vector<double> residual(ridge.size());
        for (size_t ri = 0; ri < ridge.size(); ri++) {
            double res = (alpha + beta * ridge[ri].x) - ridge[ri].y;
            residual[ri] = res;
        }

        // Durbin-Watson test to see if the residuals are correlated i.t.o AR(1)
        double res_sq_sum = residual[0] * residual[0];
        double res_delta_sq_sum = 0;
        for (size_t i = 1; i < residual.size(); i++) {
            res_sq_sum += residual[i] * residual[i];
            res_delta_sq_sum += (residual[i] - residual[i - 1]) *
                                (residual[i] - residual[i - 1]);
        }
        double t0 = 1.0 / (res_delta_sq_sum / res_sq_sum);

        if (std::isfinite(t0) && !std::isnan(t0)) {
            // squash the residuals seen by the outlier detection
            edge.residual = log1p(t0);
            merr += t0 * edge.weight;
            count += edge.weight;
        }
        else {
            edge.residual = 1e6;
        }
    }

    merr /= count;
    return merr + penalty * (model_not_invertible(v) * 1e4 +
                             merr * (weight_penalty(v[0]) / 100.0 +
                                     weight_penalty(v[1]) / 20.0));
}

void Distortion_optimizer::seed_simplex(Eigen::VectorXd& v,
                                        const Eigen::VectorXd& lambda)
{
    np = std::vector<Eigen::VectorXd>(v.size() + 1);
    // seed the simplex
    for (int i = 0; i < v.size(); i++) {
        np[i] = v;
        np[i][i] += lambda[i];
    }
    np[v.size()] = v;

    ny = Eigen::VectorXd(v.size() + 1);
    // now obtain their function values
    for (int i = 0; i < v.size() + 1; i++) {
        ny[i] = evaluate(np[i]);
    }
}

inline void Distortion_optimizer::simplex_sum(Eigen::VectorXd& psum)
{
    psum.setZero();
    for (size_t m = 0; m < np.size(); m++) {
        psum += np[m];
    }
}

void Distortion_optimizer::nelder_mead(double ftol, int& num_evals)
{
    constexpr int max_allowed_iterations = 5000;
    constexpr double epsilon = 1.0e-10;

    Eigen::VectorXd psum(np[0].size());
    num_evals = 0;
    simplex_sum(psum);

    for (;;) {
        size_t inhi;
        size_t ilo = 0;
        size_t ihi = ny[0] > ny[1] ? (inhi = 1, 0) : (inhi = 0, 1);

        for (size_t i = 0; i < np.size(); i++) {
            if (ny[i] <= ny[ilo]) {
                ilo = i;
            }
            if (ny[i] > ny[ihi]) {
                inhi = ihi;
                ihi = i;
            }
            else if (ny[i] > ny[inhi] && i != ihi) {
                inhi = i;
            }
        }
        double rtol = 2.0 * fabs(ny[ihi] - ny[ilo]) /
                      (fabs(ny[ihi]) + fabs(ny[ilo]) + epsilon);
        if (rtol < ftol) {
            std::swap(ny[0], ny[ilo]);
            for (size_t i = 0; i < (size_t)np[0].size(); i++) {
                std::swap(np[0][i], np[ilo][i]);
            }
            break;
        }
        if (num_evals >= max_allowed_iterations) {
            nelder_mead_failed = true;
            return;
        }
        num_evals += 2;

        double ytry = try_solution(psum, ihi, -1.0);
        if (ytry <= ny[ilo]) {
            ytry = try_solution(psum, ihi,
                                2.0); // expansion should also be modified
        }
        else {
            if (ytry >= ny[inhi]) {
                double ysave = ny[ihi];
                ytry = try_solution(psum, ihi, 0.5);
                if (ytry >= ysave) {
                    for (size_t i = 0; i < np.size(); i++) {
                        if (i != ilo) {
                            np[i] = psum = (np[i] + np[ilo]) * 0.5;
                            ny[i] = evaluate(psum);
                        }
                    }
                    num_evals += np[0].size();
                    simplex_sum(psum);
                }
            }
            else {
                num_evals--;
            }
        }
    }
}

double Distortion_optimizer::try_solution(Eigen::VectorXd& psum, int ihi,
                                          double fac)
{
    double fac1 = (1.0 - fac) / double(psum.size());
    double fac2 = fac1 - fac;
    Eigen::VectorXd ptry = psum * fac1 - np[ihi] * fac2;
    double ytry = evaluate(ptry);

    if (ytry < ny[ihi]) {
        ny[ihi] = ytry;
        psum += ptry - np[ihi];
        np[ihi] = ptry;
    }
    return ytry;
}

Eigen::VectorXd Distortion_optimizer::iterate(double tol)
{
    int evals = 0;
    nelder_mead(tol, evals);
    int min_idx = 0;
    for (int i = 0; i < ny.rows(); i++) {
        if (ny(i, 0) < ny(min_idx, 0)) {
            min_idx = i;
        }
    }
    return np[min_idx];
}
