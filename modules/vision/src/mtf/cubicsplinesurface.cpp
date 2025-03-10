#include "cubicsplinesurface.h"

#include <Eigen/QR>
#include <unsupported/Eigen/KroneckerProduct>
#include <opencv2/core/mat.hpp>

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace {

inline double B3(double t)
{
    if (t < 0)
        return 0;
    if (t >= 4)
        return 0;
    if (t < 1)
        return t * t * t / 6.0;
    if (t < 2)
        return (-3 * t * t * t + 12 * t * t - 12 * t + 4) / 6.0;
    if (t < 3)
        return (3 * t * t * t - 24 * t * t + 60 * t - 44) / 6.0;
    return (4 - t) * (4 - t) * (4 - t) / 6.0;
}

inline size_t find_knot(const std::vector<double>& knots, double x)
{
    int idx = std::ranges::upper_bound(knots, x) - knots.begin();
    return std::max(0, idx - 1);
}
} // namespace

Cubic_spline_surface::Cubic_spline_surface(int ncells_x, int ncells_y)
    : ncells_x(ncells_x),
      ncells_y(ncells_y),
      kmax_x(ceil(ncells_x / 4.0)),
      kmax_y(ceil(ncells_y / 4.0))
{
    for (double y = -4; y < kmax_y + 2; y += 1.0) {
        knots_y.push_back(y);
    }

    for (double x = -4; x < kmax_x + 2; x += 1.0) {
        knots_x.push_back(x);
    }
}

Eigen::MatrixXd Cubic_spline_surface::spline_fit(
    const std::vector<Eigen::Vector3d>& in_data, double lambda,
    const Eigen::Vector2d& img_dims, int pruning_threshold)
{
    std::vector<Vector4d> data(in_data.size());

    Vector2d dmin(0, 0);
    Vector2d dmax(img_dims[0], img_dims[1]);
    MatrixXd occupied(knots_y.size(), knots_x.size());
    occupied.setZero();

    cv::Mat occ_count(knots_y.size(), knots_x.size(), CV_32FC1,
                      cv::Scalar::all(0.0));
    cv::Mat occ_val(knots_y.size(), knots_x.size(), CV_32FC1,
                    cv::Scalar::all(0.0));

    double min_z = 1e50;
    double max_z = -1e50;
    // rescale input data into knot space
    for (size_t i = 0; i < in_data.size(); i++) {
        if ((in_data[i][0] - dmin[0]) / (dmax[0] - dmin[0]) >= 1 ||
            (in_data[i][1] - dmin[1]) / (dmax[1] - dmin[1]) >= 1) {
            printf("bounds exceeded with (%lf, %lf) vs (%lf, %lf), normalized "
                   "= (%lf, %lf)\n",
                   in_data[i][0], in_data[i][1], dmax[0], dmax[1],
                   (in_data[i][0] - dmin[0]) / (dmax[0] - dmin[0]),
                   (in_data[i][1] - dmin[1]) / (dmax[1] - dmin[1]));
            exit(-1);
        }
        data[i][0] = kmax_x * (in_data[i][0] - dmin[0]) / (dmax[0] - dmin[0]);
        data[i][1] = kmax_y * (in_data[i][1] - dmin[1]) / (dmax[1] - dmin[1]);
        data[i][2] = in_data[i][2];
        data[i][3] = 1.0;
        min_z = std::min(min_z, data[i][2]);
        max_z = std::max(max_z, data[i][2]);

        int col = find_knot(knots_x, data[i][0]);
        int row = find_knot(knots_y, data[i][1]);
        occupied(row, col) = 1;
        occ_count.at<float>(row, col) += 1;
        occ_val.at<float>(row, col) += data[i][2];
    }

    size_t occ_fr_count = 0;
    for (int r = 0; r < occ_val.rows; r++) {
        for (int c = 0; c < occ_val.cols; c++) {
            if (occ_count.at<float>(r, c) > 0) {
                occ_val.at<float>(r, c) /= occ_count.at<float>(r, c);
                occ_fr_count++;
            }
        }
    }
    double occ_fraction = occ_fr_count / (kmax_x * kmax_y);

    cv::Mat first_val = occ_val.clone();
    cv::Mat occ_dist(knots_y.size(), knots_x.size(), CV_32FC1,
                     cv::Scalar::all(0.0));

    for (int i = 0; i < std::max(occ_val.rows, occ_val.cols); i++) {
        // hard-coded dilation with 3-pixel cross, but skipping pixels with zero
        // counts
        cv::Mat dilated =
            cv::Mat(occ_val.rows, occ_val.cols, CV_32FC1, cv::Scalar::all(0.0));
        for (int r = 0; r < occ_val.rows; r++) {
            for (int c = 0; c < occ_val.cols; c++) {
                float local_max = -1e10;
                int delta[5][2] = {{0, 0}, {-1, 0}, {1, 0}, {0, 1}, {0, -1}};
                for (int d = 0; d < 5; d++) {
                    int dr = r + delta[d][0];
                    int dc = c + delta[d][1];

                    if (dr >= 0 && dc >= 0 && dr < occ_val.rows &&
                        dc < occ_val.cols && occ_count.at<float>(dr, dc) > 0) {
                        local_max =
                            std::max(occ_val.at<float>(dr, dc), local_max);
                    }
                }
                if (local_max > -1e10) {
                    dilated.at<float>(r, c) = local_max;
                }
            }
        }

        for (int r = 0; r < occ_val.rows; r++) {
            for (int c = 0; c < occ_val.cols; c++) {
                if (occ_count.at<float>(r, c) == 0 &&
                    occ_val.at<float>(r, c) != dilated.at<float>(r, c)) {
                    occ_count.at<float>(r, c) = 1;
                    occ_dist.at<float>(r, c) = i + 1;
                    occ_val.at<float>(r, c) = dilated.at<float>(r, c);
                    first_val.at<float>(r, c) = dilated.at<float>(r, c);
                }
            }
        }
        occ_val = dilated.clone();
    }

    constexpr double small_wt = 1e-4;
    for (int r = 0; r < occupied.rows(); r++) {
        for (int c = 0; c < occupied.cols(); c++) {
            double fill_value = first_val.at<float>(r, c);
            if (occupied(r, c) == 0) {
                data.push_back(Vector4d(knots_x[c] + 0.5, knots_y[r] + 0.5,
                                        fill_value, small_wt));
            }
        }
    }

    size_t L = knots_x.size();
    size_t K = knots_y.size();
    size_t m = data.size();
    MatrixXd Bx(m, L);
    MatrixXd By(m, K);

    // TODO: Actually, we do not need Bx and By at all, and we can probably do
    // an in-place construction directly into C ...
    MatrixXd C;
    {
        for (size_t col = 0; col < L; col++) {
            for (size_t row = 0; row < m; row++) {
                Bx(row, col) = B3(data[row][0] - knots_x[col]);
            }
        }
        for (size_t col = 0; col < K; col++) {
            for (size_t row = 0; row < m; row++) {
                By(row, col) = B3(data[row][1] - knots_y[col]);
            }
        }

        C = Eigen::kroneckerProduct(Bx, MatrixXd::Ones(1, K))
                .cwiseProduct(
                    Eigen::kroneckerProduct(MatrixXd::Ones(1, L), By));
    }

    MatrixXd P;
    {
        MatrixXd Px;
        MatrixXd Py;
        {
            MatrixXd Dx = MatrixXd::Zero(L - 1, L);
            for (size_t col = 0; col < L - 1; col++) {
                double scale = knots_x[col] >= -1 ? 1.0 : 0.5;
                Dx(col, col) = -scale;
                Dx(col, col + 1) = scale;
            }
            Px = Dx.transpose() * Dx;
        }

        {
            MatrixXd Dy = MatrixXd::Zero(K - 1, K);
            for (size_t col = 0; col < K - 1; col++) {
                double scale = knots_y[col] >= -1 ? 1.0 : 0.5;
                Dy(col, col) = -scale;
                Dy(col, col + 1) = scale;
            }
            Py = Dy.transpose() * Dy;
        }

        P = Eigen::kroneckerProduct(MatrixXd::Identity(K, K), Px) +
            Eigen::kroneckerProduct(MatrixXd::Identity(L, L), Py);
    }

    VectorXd rhs(m);
    VectorXd W(m);
    for (size_t i = 0; i < data.size(); i++) {
        rhs[i] = data[i][2];
        W[i] = data[i][3];
    }

    // if the chart appears a bit sparse, we increase the amount of smoothing a
    // bit
    if (occ_fraction < 0.6) {
        lambda *= 10;
    }

    VectorXd sol = (C.transpose() * W.asDiagonal() * C + lambda * P)
                       .colPivHouseholderQr()
                       .solve(C.transpose() * W.asDiagonal() * rhs);

    MatrixXd grid(ncells_y, ncells_x);
    grid.setZero();
    for (size_t gc = 0; gc < (size_t)grid.cols(); gc++) {
        double px = double(gc) * kmax_x / (grid.cols());
        for (size_t gr = 0; gr < (size_t)grid.rows(); gr++) {
            double py = double(gr) * kmax_y / (grid.rows());
            double sum = 0;

            size_t sol_idx = 0;
            for (size_t row = 0; row < L; row++) {
                for (size_t col = 0; col < K; col++) {
                    sum += sol[sol_idx++] * B3(px - knots_x[row]) *
                           B3(py - knots_y[col]);
                }
            }

            int kx = find_knot(knots_x, px);
            int ky = find_knot(knots_y, py);
            if (occ_dist.at<float>(ky, kx) > pruning_threshold) {
                sum = 0;
            }
            grid(gr, gc) = sum;
        }
    }

    return grid;
}
