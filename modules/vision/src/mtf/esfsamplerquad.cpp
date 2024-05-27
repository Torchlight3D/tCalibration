#include "esfsamplerquad.h"

#include <array>

namespace tl {

EsfQuadSampler::EsfQuadSampler(double max_dot, Bayer::cfa_mask_t cfa_mask,
                               double border_width)
    : EsfSampler(max_dot, cfa_mask, 1e6, border_width)
{
}

void EsfQuadSampler::sample(Edge_model &edge_model,
                            std::vector<Ordered_point> &local_ordered,
                            const std::map<int, scanline> &scanset,
                            double &edge_length, const cv::Mat &geom_img,
                            const cv::Mat &sampling_img,
                            Bayer::cfa_mask_t cfa_mask)
{
    cfa_mask = cfa_mask == Bayer::DEFAULT ? default_cfa_mask : cfa_mask;

    double max_along_edge = -1e50;
    double min_along_edge = 1e50;

    const std::array<double, 3> &qp = edge_model.quad_coeffs();
    for (auto it = scanset.begin(); it != scanset.end(); ++it) {
        int y = it->first;
        if (y < border_width || y > geom_img.rows - 1 - border_width) {
            continue;
        }

        int rowcode = (y & 1) << 1;
        for (int x = it->second.start; x <= it->second.end; ++x) {
            if (x < border_width || x > geom_img.cols - 1 - border_width) {
                continue;
            }

            int code = 1 << ((rowcode | (x & 1)) ^ 3);
            if ((code & cfa_mask) == 0) {
                continue;
            }

            cv::Point2d d = cv::Point2d(x, y) - edge_model.get_centroid();
            double perp = d.ddot(edge_model.get_normal());
            double par = d.ddot(edge_model.get_direction());

            // (par, perp) is in the local coordinate frame of the parabola qp
            // so find the closest point on qp
            const auto roots = quad_tangency(cv::Point2d(par, perp), qp);
            perp = 1e20;
            for (const auto &r : roots) {
                cv::Point2d on_qp(r, r * r * qp[0] + r * qp[1] + qp[2]);
                cv::Point2d recon = on_qp.x * edge_model.get_direction() +
                                    on_qp.y * edge_model.get_normal() +
                                    edge_model.get_centroid();
                cv::Point2d delta = cv::Point2d(x, y) - recon;
                double dist = std::copysign(
                    norm(delta), delta.ddot(edge_model.get_normal()));
                if (std::abs(dist) < std::abs(perp)) {
                    perp = dist;
                }
            }

            if (std::abs(perp) < max_dot) {
                local_ordered.push_back(
                    Ordered_point(perp, sampling_img.at<uint16_t>(y, x)));
                max_along_edge = std::max(max_along_edge, par);
                min_along_edge = std::min(min_along_edge, par);

                // if ((x & 1) || (y & 1)) {
                //     cv::Vec3b &color = od_img.at<cv::Vec3b>(y, x);
                //     color[0] = 255;
                //     color[1] = 0;
                //     color[2] = 0;
                // }
            }
        }
    }

    edge_length = max_along_edge - min_along_edge;
}

std::vector<double> EsfQuadSampler::quad_tangency(
    const cv::Point2d &p, const std::array<double, 3> &qp)
{
    const double &a = qp[0];
    const double &b = qp[1];
    const double &c = qp[2];

    double ca = 2 * a * a;
    double cb = 3 * a * b;
    double cc = 1 + 2 * a * c - 2 * a * p.y + b * b;
    double cd = b * c - p.y * b - p.x;

    // convert to normalized cubic with cubic coefficient = 1.0
    double pa = cb / ca;
    double pb = cc / ca;
    double pc = cd / ca;

    double Q = (pa * pa - 3 * pb) / 9.0;
    double R = (2 * pa * pa * pa - 9 * pa * pb + 27 * pc) / 54.0;

    std::vector<double> roots;
    if (R * R < Q * Q * Q) { // three real roots
        double theta = std::acos(R / std::sqrt(Q * Q * Q));
        roots.push_back(-2 * std::sqrt(Q) * std::cos(theta / 3.0) - pa / 3.0);
        roots.push_back(-2 * std::sqrt(Q) * std::cos((theta + 2 * M_PI) / 3.0) -
                        pa / 3.0);
        roots.push_back(-2 * std::sqrt(Q) * std::cos((theta - 2 * M_PI) / 3.0) -
                        pa / 3.0);
    }
    else { // one real root, two complex roots
        double A = -std::copysign(
            std::cbrt(std::abs(R) + std::sqrt(R * R - Q * Q * Q)), R);
        double B = A == 0 ? 0 : Q / A;
        roots.push_back(A + B - pa / 3.0);
    }

    return roots;
}

} // namespace tl
