#include "bundle.h"

#include <opencv2/calib3d/calib3d.hpp>

#include "common_types.h"
#include "ellipse.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

Bundle_adjuster::Bundle_adjuster(Vector2dList& img_points,
                                 Vector3dList& world_points, Eigen::Vector3d& t,
                                 cv::Mat in_rod_angles, double distortion,
                                 double w, double fid_diameter,
                                 double img_scale)
    : img_points(img_points),
      world_points(world_points),
      rot_mat(3, 3, CV_64FC1),
      rod_angles(3, 1, CV_64FC1),
      fid_diameter(fid_diameter),
      img_scale(img_scale)
{
    // pack initial parameters
    VectorXd init(8);
    init[0] = t[0];
    init[1] = t[1];
    init[2] = t[2];
    init[3] = in_rod_angles.at<double>(0, 0);
    init[4] = in_rod_angles.at<double>(1, 0);
    init[5] = in_rod_angles.at<double>(2, 0);
    init[6] = distortion;
    init[7] = w;

    best_sol = initial = init;
}

void Bundle_adjuster::solve()
{
    VectorXd scale(best_sol.size());
    scale << 1e-6, 1e-6, 0.01, // origin
        1e-3, 1e-3, 1e-3,      // angles
        1e-5,                  // distortion
        0.025 * img_scale;     // 1/focal length

    nelder_mead_failed = false;
    seed_simplex(best_sol, scale);
    best_sol = iterate(1e-12);
}

void Bundle_adjuster::unpack(Eigen::Matrix3d& R, Eigen::Vector3d& t,
                             double& distortion, double& w)
{
    VectorXd& v = best_sol;
    rod_angles.at<double>(0, 0) = v[3];
    rod_angles.at<double>(1, 0) = v[4];
    rod_angles.at<double>(2, 0) = v[5];
    cv::Rodrigues(rod_angles, rot_mat);

    // global_scale * K * R | t
    R << rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1),
        rot_mat.at<double>(0, 2), rot_mat.at<double>(1, 0),
        rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
        rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1),
        rot_mat.at<double>(2, 2);

    t = Vector3d(v[0], v[1], v[2]);

    distortion = v[6];
    w = v[7];
}

double Bundle_adjuster::evaluate(const Eigen::VectorXd& v, double penalty)
{
    rod_angles.at<double>(0, 0) = v[3];
    rod_angles.at<double>(1, 0) = v[4];
    rod_angles.at<double>(2, 0) = v[5];
    cv::Rodrigues(rod_angles, rot_mat);

    // global_scale * K * R | t
    Matrix3d R;
    R << rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1),
        rot_mat.at<double>(0, 2), rot_mat.at<double>(1, 0),
        rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
        rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1),
        rot_mat.at<double>(2, 2);

    // we can backproject the centres of the circles, and compare that to
    // the centres of the ellipses extracted from the image. this will not
    // compensate for the eccentricity (which moves the centre of a
    // projected circle so that it no longer coincides with the centre of
    // the ellipse). We could compute an eccentricity correction following
    // Ahn, but I choose to rather project the complete circle geometry from
    // the world coordinate system into the image plane. the resulting
    // projected ellipse will naturally have the same centre as the ones we
    // extract from the image, so eccentricity is corrected for

    const double d = fid_diameter;

    Vector3d T(v[0], v[1], v[2]);
    Vector3d Cp(0, 0, img_scale / v[7]);
    MatrixXd Jp(3, 2);
    Jp << 1, 0, 0, 1, 0, 0;

    double rmse = 0;
    for (size_t i = 0; i < world_points.size(); i++) {
        Vector3d Ce = R * world_points[i] + T;
        Vector3d ebar = R.transpose() * (-Ce);
        Vector3d cbar = R.transpose() * Ce;

        Matrix3d bA;
        bA << 1.0 / (d * d), 0, -ebar[0] / (ebar[2] * d * d), 0, 1.0 / (d * d),
            -ebar[1] / (ebar[2] * d * d), -ebar[0] / (ebar[2] * d * d),
            -ebar[1] / (ebar[2] * d * d),
            (SQR(ebar[0] / d) + SQR(ebar[1] / d) - 1) / SQR(ebar[2]);

        Vector3d bB;
        bB << 0, 0, 2 / ebar[2];
        double bc = -1;

        Matrix3d A = R * bA * R.transpose();
        Vector3d B = R * (bB - 2.0 * bA * cbar);
        double c =
            (cbar.transpose() * bA * cbar - bB.transpose() * cbar)(0, 0) + bc;

        MatrixXd hA = Jp.transpose() * A * Jp;
        VectorXd hB = Jp.transpose() * (B + 2 * A * Cp);
        double hc = (Cp.transpose() * A * Cp + B.transpose() * Cp)(0, 0) + c;

        Matrix3d EM;
        EM << hA(0, 0), hA(0, 1), 0.5 * hB[0], hA(0, 1), hA(1, 1), 0.5 * hB[1],
            0.5 * hB[0], 0.5 * hB[1], hc;

        Ellipse_detector ed;
        ed._matrix_to_ellipse(EM);
        Vector2d srp = 1.0 / img_scale * Vector2d(ed.centroid_x, ed.centroid_y);

        // apply lens distortion to reconstructed ellipe centre
        double rad = 1 + v[6] * (srp[0] * srp[0] + srp[1] * srp[1]);
        srp /= rad;

        rmse += (srp - img_points[i]).squaredNorm();
    }

    rmse = sqrt(rmse / world_points.size());

    double finit = 1.0 / initial[7];
    double fr = 1.0 / v[7];
    double fd = std::abs(1.0 / v[7] - finit);
    return rmse +
           penalty *
               ((fr < focal_lower ? (fr - focal_lower) * (fr - focal_lower)
                                  : 0) +
                (fr > focal_upper ? (fr - focal_upper) * (fr - focal_upper)
                                  : 0) +
                focal_mode_constraint *
                    ((fd > 0.1 * finit)
                         ? fd * fd
                         : ((fd > 0.05) ? fd * fd * fd * fd : 0)));
}

void Bundle_adjuster::seed_simplex(Eigen::VectorXd& v,
                                   const Eigen::VectorXd& lambda)
{
    np = std::vector<VectorXd>(v.size() + 1);
    // seed the simplex
    for (int i = 0; i < v.size(); i++) {
        np[i] = v;
        np[i][i] += lambda[i];
    }
    np[v.size()] = v;

    ny = VectorXd(v.size() + 1);
    // now obtain their function values
    for (int i = 0; i < v.size() + 1; i++) {
        ny[i] = evaluate(np[i]);
    }
}

void Bundle_adjuster::simplex_sum(Eigen::VectorXd& psum)
{
    psum.setZero();
    for (size_t m = 0; m < np.size(); m++) {
        psum += np[m];
    }
}

void Bundle_adjuster::nelder_mead(double ftol, int& num_evals)
{
    constexpr int max_allowed_iterations = 5000;
    constexpr double epsilon = 1.0e-10;

    VectorXd psum(np[0].size());
    num_evals = 0;
    simplex_sum(psum);

    while (true) {
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
        double rtol = 2.0 * std::abs(ny[ihi] - ny[ilo]) /
                      (std::abs(ny[ihi]) + std::abs(ny[ilo]) + epsilon);
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
            ytry = try_solution(psum, ihi, 2.0);
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

double Bundle_adjuster::try_solution(Eigen::VectorXd& psum, int ihi, double fac)
{
    double fac1 = (1.0 - fac) / double(psum.size());
    double fac2 = fac1 - fac;
    VectorXd ptry = psum * fac1 - np[ihi] * fac2;
    double ytry = evaluate(ptry);

    if (ytry < ny[ihi]) {
        ny[ihi] = ytry;
        psum += ptry - np[ihi];
        np[ihi] = ptry;
    }
    return ytry;
}

Eigen::VectorXd Bundle_adjuster::iterate(double tol)
{
    int evals = 0;
    nelder_mead(tol, evals);

    int min_idx = 0;
    for (size_t i = 0; i < (size_t)ny.size(); i++) {
        if (ny[i] < ny[min_idx]) {
            min_idx = i;
        }
    }
    return np[min_idx];
}

bool Bundle_adjuster::optimization_failure() const
{
    return nelder_mead_failed;
}
