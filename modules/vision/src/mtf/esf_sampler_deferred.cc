#include "esf_sampler_deferred.h"

EsfDeferredSampler::EsfDeferredSampler(Undistort *undistort, double max_dot,
                                       Bayer::cfa_mask_t cfa_mask,
                                       double border_width)
    : EsfSampler(max_dot, cfa_mask, 1e6, border_width), undistort_(undistort)
{
}

cv::Point2d EsfDeferredSampler::bracket_minimum(double t0, const cv::Point2d &l,
                                                const cv::Point2d &p,
                                                const cv::Point2d &pt)
{
    double h = 0.01;
    cv::Point2d p_t0 = undistort_->transform_point(t0 * l + p);
    double d_t0 = norm(p_t0 - pt);
    cv::Point2d p_tph = undistort_->transform_point((t0 + h) * l + p);
    double d_tph = norm(p_tph - pt);

    double a;
    double b;
    double eta;

    if (d_t0 > d_tph) {
        // forward step
        a = t0;
        eta = t0 + h;
        while (true) { // TODO: add a way to break out
            h = h * 2;
            b = a + h;

            cv::Point2d p_b = undistort_->transform_point(b * l + p);
            double d_b = norm(p_b - pt);
            cv::Point2d p_eta = undistort_->transform_point(eta * l + p);
            double d_eta = norm(p_eta - pt);

            if (d_b >= d_eta) {
                return cv::Point2d(a, b);
            }

            a = eta;
            eta = b;
        }
    }
    else {
        // backward step
        eta = t0;
        b = t0 + h;
        while (true) {
            h = h * 2;
            a = b - h;

            cv::Point2d p_a = undistort_->transform_point(a * l + p);
            double d_a = norm(p_a - pt);
            cv::Point2d p_eta = undistort_->transform_point(eta * l + p);
            double d_eta = norm(p_eta - pt);

            if (d_a >= d_eta) {
                return cv::Point2d(a, b);
            }

            b = eta;
            eta = a;
        }
    }
}

cv::Point2d EsfDeferredSampler::derivative(double t0, const cv::Point2d &l,
                                           const cv::Point2d &p)
{
    constexpr double epsilon = 1e-8;
    cv::Point2d p_t = undistort_->transform_point(t0 * l + p);
    cv::Point2d p_th = undistort_->transform_point((t0 + epsilon) * l + p);
    return (1.0 / epsilon) * (p_th - p_t);
}

double EsfDeferredSampler::quadmin(const cv::Point2d &a, const cv::Point2d &b,
                                   const cv::Point2d &c)
{
    double denom = (b.x - a.x) * (b.y - c.y) - (b.x - c.x) * (b.y - a.y);
    double num = (b.x - a.x) * (b.x - a.x) * (b.y - c.y) -
                 (b.x - c.x) * (b.x - c.x) * (b.y - a.y);
    return b.x - 0.5 * num / denom;
}

void EsfDeferredSampler::sample(Edge_model &edge_model,
                                std::vector<Ordered_point> &local_ordered,
                                const std::map<int, scanline> &scanset,
                                double &edge_length, const cv::Mat &geom_img,
                                const cv::Mat &sampling_img,
                                Bayer::cfa_mask_t cfa_mask)
{
    cfa_mask = cfa_mask == Bayer::DEFAULT ? default_cfa_mask : cfa_mask;

    double max_along_edge = -1e50;
    double min_along_edge = 1e50;

    std::map<int, scanline> m_scanset;
    for (auto it = scanset.begin(); it != scanset.end(); ++it) {
        int y = it->first;
        for (int x = it->second.start; x <= it->second.end; ++x) {
            cv::Point tp = undistort_->transform_pixel(x, y);
            auto fit = m_scanset.find(tp.y);
            if (fit == m_scanset.end()) {
                m_scanset.insert(std::make_pair(tp.y, scanline(tp.x, tp.x)));
            }
            else {
                fit->second.update(tp.x);
            }
        }
    }

    for (auto it = m_scanset.begin(); it != m_scanset.end(); ++it) {
        int y = it->first;
        if (y < border_width || y > sampling_img.rows - 1 - border_width)
            continue;
        int rowcode = (y & 1) << 1;

        for (int x = it->second.start; x <= it->second.end; ++x) {
            if (x < border_width || x > sampling_img.cols - 1 - border_width)
                continue;

            int code = 1 << ((rowcode | (x & 1)) ^ 3);
            if ((code & cfa_mask) == 0)
                continue;

            // transform warped pixel to idealized rectilinear
            cv::Point2d tp = undistort_->inverse_transform_point(x, y);

            bool inside_image = lrint(tp.x) >= 0 &&
                                lrint(tp.x) < geom_img.cols &&
                                lrint(tp.y) >= 0 && lrint(tp.y) < geom_img.rows;
            if (!inside_image)
                continue;

            cv::Point2d d = tp - edge_model.get_centroid();
            double perp = d.ddot(edge_model.get_normal());
            double par = d.ddot(edge_model.get_direction());

            if (!undistort_->rectilinear_equivalent()) {
                // 'par' is gamma from the paper
                // apply bracketing
                cv::Point2d pd(x, y);
                cv::Point2d bracketed =
                    bracket_minimum(par, edge_model.get_direction(),
                                    edge_model.get_centroid(), pd);

                // apply quadratic interpolation
                cv::Point2d p1(
                    bracketed.x,
                    norm(undistort_->transform_point(
                             bracketed.x * edge_model.get_direction() +
                             edge_model.get_centroid()) -
                         pd));
                cv::Point2d p2(0.5 * (bracketed.x + bracketed.y),
                               norm(undistort_->transform_point(
                                        (0.5 * (bracketed.x + bracketed.y)) *
                                            edge_model.get_direction() +
                                        edge_model.get_centroid()) -
                                    pd));
                cv::Point2d p3(
                    bracketed.y,
                    norm(undistort_->transform_point(
                             bracketed.y * edge_model.get_direction() +
                             edge_model.get_centroid()) -
                         pd));
                double tau_star = quadmin(p1, p2, p3);

                // find tangent, then project onto normal
                cv::Point2d tangent =
                    derivative(tau_star, edge_model.get_direction(),
                               edge_model.get_centroid());
                tangent *= 1.0 / norm(tangent);
                cv::Point2d lnorm(-tangent.y, tangent.x);
                cv::Point2d ppd = undistort_->transform_point(
                    tau_star * edge_model.get_direction() +
                    edge_model.get_centroid());

                perp = (pd - ppd).ddot(lnorm);
            }
            if (fabs(perp) < max_dot) {
                local_ordered.push_back(
                    Ordered_point(perp, sampling_img.at<uint16_t>(y, x)));
                max_along_edge = std::max(max_along_edge, par);
                min_along_edge = std::min(min_along_edge, par);
            }
        }
    }

    edge_length = max_along_edge - min_along_edge;
}
