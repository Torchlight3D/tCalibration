#include "esfsamplerpiecewisequad.h"

#include <Eigen/Dense>

namespace tl {

namespace {
inline double tricube(double d)
{
    const auto w = (1 - std::abs(d) * std::abs(d) * std::abs(d));
    return w * w * w;
}
} // namespace

EsfPiecewiseQuadSampler::EsfPiecewiseQuadSampler(double max_dot,
                                                 Bayer::cfa_mask_t cfa_mask,
                                                 double border_width)
    : EsfQuadSampler(max_dot, cfa_mask, border_width)
{
}

std::vector<double> EsfPiecewiseQuadSampler::piecewise_quadfit(
    const std::vector<cv::Point2d> &pts) const
{
    /// assumes pts.size() >= 3

    // compute integral array, which will allow us to calculate
    // box-filtered versions of pts in constant time
    std::vector<cv::Point2d> integral(pts.size() + 1);
    cv::Point2d cumulative = pts.front();
    for (size_t i = 0; i < pts.size(); i++) {
        integral[i] = cumulative;
        cumulative += pts[i];
    }
    integral.back() = cumulative;

    const int nbins =
        16; // seems reasonable relative to 2nd derivative filter below
    std::vector<cv::Point2d> reduced(nbins);
    for (int i = 0; i < nbins; i++) {
        int li = std::max(0, int((i - 1.5) * int(pts.size()) / nbins));
        int ui = std::min((int)pts.size() - 1,
                          int((i + 2.5) * int(pts.size()) / nbins));

        reduced[i] = (integral[ui] - integral[li]) * (1.0 / (ui - li));
    }

    const int hw = 3;
    double sgw[2 * hw + 1] = {
        5.952381e-02,  6.938894e-18, -3.571429e-02, -4.761905e-02,
        -3.571429e-02, 6.938894e-18, 5.952381e-02,
    };

    std::vector<cv::Point2d> deriv(reduced.size());
    for (int i = 0; i < (int)reduced.size(); i++) {
        double wsum = 0;
        deriv[i].x = reduced[i].x;
        deriv[i].y = 0;
        for (int j = -hw; j <= hw; j++) {
            int ei = i + j;
            if (ei >= 0 && ei < (int)reduced.size()) {
                wsum += sgw[j + hw];
                deriv[i].y += sgw[j + hw] * reduced[ei].y;
            }
        }
        deriv[i].y /= wsum;
    }

    // find zero crossings, pick one closest to centre?
    // how do we decide if we have multiple crossings?
    int i1 = 0;
    for (int i = 0; i < (int)deriv.size() - 1; i++) {
        if (deriv[i].y * deriv[i + 1].y < 0) {
            if (std::abs(deriv[i].x) < std::abs(deriv[i1].x)) {
                i1 = i;
            }
        }
    }
    // interpolate
    double slope =
        (deriv[i1 + 1].x - deriv[i1].x) / (deriv[i1 + 1].y - deriv[i1].y);
    double crossing = (0 - deriv[i1].y) * slope + deriv[i1].x;

    Eigen::MatrixXd A(pts.size(), 6);
    Eigen::VectorXd b(pts.size());
    Eigen::MatrixXd C(2, 6);

    A.setZero();

    double span = std::abs(pts.back().x - pts.front().x);
    double x_scale = span * 0.5 / sqrt(0.5);

    if (crossing < pts.front().x + 0.3 * span ||
        crossing > pts.front().x + 0.7 * span) {
        crossing = 0;
    }

    double T = crossing / x_scale;

    // constraints: first row is f(T) = g(T), second row is f'(T) = g'(T)
    C << T * T, T, 1, -T * T, -T, -1, 2 * T, 1, 0, -2 * T, -1, 0;

    for (size_t r = 0; r < pts.size(); r++) {
        double sx = pts[r].x / x_scale;
        double w1 = sx <= T ? 1 : 0;
        A(r, 0) = w1 * sx * sx;
        A(r, 1) = w1 * sx;
        A(r, 2) = w1 * 1.0;

        double w2 = sx > T ? 1 : 0;
        A(r, 3) = w2 * sx * sx;
        A(r, 4) = w2 * sx;
        A(r, 5) = w2 * 1.0;

        b[r] = pts[r].y * (w1 + w2);
    }

    // null space method
    // W. Gander et al., Scientific Computing - An Introduction using Maple and
    // MATLAB, Texts in Computational Science and Engineering 11,
    // DOI 10.1007/978-3-319-04325-8 6,
    // Springer International Publishing Switzerland 2014
    // algorithm 6.22, page 334
    int n = A.cols();
    int p = C.rows();
    auto qr1 = C.transpose().householderQr();
    Eigen::MatrixXd Q = qr1.householderQ();
    Eigen::MatrixXd R =
        qr1.matrixQR().topRows(p).template triangularView<Eigen::Upper>();

    Eigen::VectorXd d = Eigen::VectorXd::Zero(p);
    Eigen::VectorXd y1 = R.transpose().triangularView<Eigen::Lower>().solve(d);

    Eigen::VectorXd x1 = Q.leftCols(p) * y1;
    Eigen::VectorXd y2 =
        (A * Q.rightCols(n - p)).colPivHouseholderQr().solve(b - A * x1);
    Eigen::VectorXd sol = x1 + Q.rightCols(n - p) * y2;

    sol[0] /= x_scale * x_scale;
    sol[1] /= x_scale;
    sol[3] /= x_scale * x_scale;
    sol[4] /= x_scale;
    T *= x_scale;

    // sol[0-5] : quadratic parameters
    // sol[6-7] : constraints values (not used)

    std::vector<double> parms(7);
    parms[0] = T;
    parms[1] = sol[0];
    parms[2] = sol[1];
    parms[3] = sol[2];
    parms[4] = sol[3];
    parms[5] = sol[4];
    parms[6] = sol[5];
    return parms;
}

void EsfPiecewiseQuadSampler::sample(Edge_model &edge_model,
                                     std::vector<Ordered_point> &local_ordered,
                                     const std::map<int, scanline> &scanset,
                                     double &edge_length,
                                     const cv::Mat &geom_img,
                                     const cv::Mat &sampling_img,
                                     Bayer::cfa_mask_t cfa_mask)
{
    cfa_mask = cfa_mask == Bayer::DEFAULT ? default_cfa_mask : cfa_mask;

    double max_along_edge = -1e50;
    double min_along_edge = 1e50;

    std::vector<cv::Point2d> local_pts;
    for (auto p : edge_model.ridge) {
        cv::Point2d delta = p - edge_model.get_centroid();
        local_pts.push_back(cv::Point2d(delta.ddot(edge_model.get_direction()),
                                        delta.ddot(edge_model.get_normal())));
    }

    std::vector<double> qpp(7, 0.0);
    if (local_pts.size() < 20) { // if we have a really short edge, fall back on
                                 // quad estimated from target edge
        qpp[1] = qpp[4] = edge_model.quad_coeffs()[0];
        qpp[2] = qpp[5] = edge_model.quad_coeffs()[1];
        qpp[3] = qpp[6] = edge_model.quad_coeffs()[2];
    }
    else {
        qpp = piecewise_quadfit(local_pts);
    }
    std::array<double, 3> qp;

    for (auto it = scanset.begin(); it != scanset.end(); ++it) {
        int y = it->first;
        if (y < border_width || y > geom_img.rows - 1 - border_width)
            continue;
        int rowcode = (y & 1) << 1;

        for (int x = it->second.start; x <= it->second.end; ++x) {
            if (x < border_width || x > geom_img.cols - 1 - border_width)
                continue;

            int code = 1 << ((rowcode | (x & 1)) ^ 3);
            if ((code & cfa_mask) == 0)
                continue;

            cv::Point2d d = cv::Point2d(x, y) - edge_model.get_centroid();
            double perp = d.ddot(edge_model.get_normal());
            double par = d.ddot(edge_model.get_direction());

            if (par < qpp[0]) {
                qp[0] = qpp[1];
                qp[1] = qpp[2];
                qp[2] = qpp[3];
            }
            else {
                qp[0] = qpp[4];
                qp[1] = qpp[5];
                qp[2] = qpp[6];
            }

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

} // namespace tl
