#include "inertialbasedscaleestimation.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <glog/logging.h>

namespace tl {

using Eigen::ArrayXXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {

// Discrete Linear Time-Invariant ODE with possible Gaussian noise
//
// INPUT:    F   : 3x3 Feedback matrix
//           L   : 3x1 Noise effect matrix
//           Qc  : Scalar spectral density
//           dt  : time-step
//
// OUTPUT:   A   : Transition matrix
//           Q   : Discrete process covariance matrix
//
void ltiDisc(const Matrix3d& F, const Vector3d& L, double qc, double dt,
             Matrix3d& out_A, Matrix3d& out_Q)
{
    // Closed form integration of the covariance by matrix fraction
    // decomposition
    MatrixXd Phi(6, 6);
    Phi.block<3, 3>(0, 0) = F;
    Phi.block<3, 3>(0, 3) = L * qc * L.transpose();
    Phi.block<3, 3>(3, 0) = Matrix3d::Zero();
    Phi.block<3, 3>(3, 3) = -F.transpose();

    out_A = (F * dt).exp();

    MatrixXd zeroone(6, 3);
    zeroone.block<3, 3>(0, 0) = Matrix3d::Zero();
    zeroone.block<3, 3>(3, 0) = Matrix3d::Identity();

    MatrixXd AB(6, 3);
    AB = (Phi * dt).exp() * zeroone;
    out_Q = AB.block<3, 3>(0, 0) * (AB.block<3, 3>(3, 0).inverse());
}

// Calculate a smooth estimate of the second derivative (accelerations) of
// the input position data.  We use a Kalman filter, followed by a
// Rauch–Tung–Striebel filter to perform the smoothing
//
// INPUT:    pose   : A series of poses, evenly spaced in time
//           dt     : The interval between samples in 'pose'
//
// OUTPUT:          : Smoothed estimates of the acceleratio of the input
//
ArrayXXd KalmanRTS(const std::vector<Vector3d>& pose, double dt)
{
    // Assume that we have roughly uniform sampling to optimize computations,
    //  otherwise we would need to recompute A and Q in the loop which is
    //  slower.
    const size_t numPts = pose.size();

    ArrayXXd acc_kfs(3, numPts);

    // clang-format off
    Matrix3d F;
    F << 0, 1, 0,
         0, 0, 1,
         0, 0, 0;
    // clang-format on
    double R = 0.01;
    Vector3d L(0, 0, 1);
    Vector3d H(1, 0, 0);
    Vector3d m0(0, 0, 0);
    Matrix3d P0 = Matrix3d::Identity() * 1e4;

    // Compute maximum likelihood estimate of qc on a grid
    std::vector<double> qc_list = {45, 55, 65, 75, 85, 95, 105, 115, 125};
    std::vector<double> lh_list = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    ArrayXXd pos(3, numPts);
    for (size_t pp = 0; pp < numPts; ++pp) {
        pos(0, pp) = pose[pp].x();
        pos(1, pp) = pose[pp].y();
        pos(2, pp) = pose[pp].z();
    }

    const size_t numQc = qc_list.size();
    for (size_t jj = 0; jj < numQc; ++jj) {
        double qc = qc_list[jj];

        Matrix3d A;
        Matrix3d Q;

        ltiDisc(F, L, qc, dt, A, Q);

        double lh = 0;
        for (size_t ii = 0; ii < 3; ++ii) {
            // Kalman filter
            Vector3d m = m0;
            Matrix3d P = P0;
            MatrixXd kf_m = MatrixXd::Zero(3, numPts);
            std::vector<Matrix3d> kf_P(numPts);

            for (size_t kk = 0; kk < numPts; ++kk) {
                m = A * m;
                P = A * P * A.transpose() + Q;

                double nu = pos(ii, kk) - H.dot(m);
                double S = H.transpose() * P * H + R;
                Vector3d K = (P * H) / S;

                m = m + K * nu;
                P = P - K * S * K.transpose();

                lh =
                    lh + 0.5 * log(2 * M_PI) + 0.5 * log(S) + 0.5 * nu * nu / S;

                kf_m.block<3, 1>(0, kk) = m;
                kf_P[kk] = P;
            }
        }
        lh_list[jj] = lh;
    }

    size_t idx =
        std::min_element(lh_list.begin(), lh_list.end()) - lh_list.begin();
    double qc = qc_list[idx];

    // Kalman filter and smoother
    Matrix3d A;
    Matrix3d Q;
    ltiDisc(F, L, qc, dt, A, Q);

    for (int ii = 0; ii < 3; ++ii) {
        // Kalman filter
        Vector3d m = m0;
        Matrix3d P = P0;
        MatrixXd kf_m = MatrixXd::Zero(3, numPts);
        std::vector<Matrix3d> kf_P(numPts);

        for (size_t kk = 0; kk < numPts; ++kk) {
            m = A * m;
            P = A * P * A.transpose() + Q;

            double S = H.transpose() * P * H + R;
            Vector3d K = (P * H) / S;

            m = m + K * (pos(ii, kk) - H.transpose() * m);
            P = P - K * S * K.transpose();

            kf_m.block<3, 1>(0, kk) = m;
            kf_P[kk] = P;
        }

        // Rauch–Tung–Striebel smoother
        Vector3d ms = m;
        Matrix3d Ps = P;
        MatrixXd rts_m = MatrixXd::Zero(3, numPts);
        rts_m.block<3, 1>(0, numPts - 1) = ms;
        std::vector<Matrix3d> rts_P(numPts);
        rts_P.back() = Ps;

        for (int kk = numPts - 2; kk >= 0; --kk) {
            MatrixXd mp(3, 1);
            mp = A * kf_m.block<3, 1>(0, kk);

            Matrix3d Pp = A * kf_P[kk] * A.transpose() + Q;
            Matrix3d Ck = kf_P[kk] * A.transpose() * Pp.inverse();
            ms = kf_m.block<3, 1>(0, kk) + Ck * (ms - mp);
            Ps = kf_P[kk] + Ck * (Ps - Pp) * Ck.transpose();
            rts_m.block<3, 1>(0, kk) = ms;
            rts_P[kk] = Ps;
        }

        acc_kfs.row(ii) = rts_m.row(2).array();
    }

    return acc_kfs;
}

// Fast, single-pass linear interpolator
// INPUT :
//     X : Original sampling along the X-axis (e.g. the time of a time-series)
//     Y : Original data, sampled at values of 'X'
//   X_I : The interpolation sampling (can include values before and after the
//   limits of X)
// OUTPUT:
//     Interpolated values of Y along X_I
//
VectorXd linearInterpolation(const VectorXd& x, const VectorXd& y,
                             const std::vector<double>& x_i)
{
    const size_t sz_x = x.size();
    const size_t sz_xi = x_i.size();
    VectorXd y_i(sz_xi);

    size_t i0 = 0;
    size_t i1 = 1;
    for (size_t ii = 0; ii < sz_xi; ++ii) {
        // Enforce strict monotonicity
        assert(x(i1) > x(i0));

        // Find the first value of X that is larger than our interpolation query
        // value, x_i(ii)
        while ((x_i[ii] > x(i1)) && (i1 < sz_x - 1)) {
            i1++;
        }
        i0 = i1;

        // Find the first value of X that is smaller  than our interpolation
        // query value
        while ((x_i[ii] < x(i0)) && (i0 > 0)) {
            i0--;
        }

        // Ensure that they are not the same during end extrapolation
        if (i0 == i1) {
            i0--;
        }

        y_i(ii) =
            y(i0) + (x_i[ii] - x(i0)) * ((y(i1) - y(i0)) / (x(i1) - x(i0)));
    }

    return y_i;
}

// Rotate point p according to unit quaternion q.
//
// INPUT:  q : Unit quaternion(s) as Nx4 Array
//         p : Points as Nx3 arrau
//
// OUTPUT: Rotated point(s) as Nx3 array
//
ArrayXXd rotatePoints(const ArrayXXd& quats, const ArrayXXd& pts)
{
    assert(quats.rows() == pts.rows());
    const size_t N = pts.rows();

    ArrayXXd out(N, 3);
    for (size_t ii = 0; ii < N; ++ii) {
        Vector3d pt = Quaterniond(quats(ii, 0), quats(ii, 1), quats(ii, 2),
                                  quats(ii, 3)) *
                      Vector3d(pts(ii, 0), pts(ii, 1), pts(ii, 2));
        out.row(ii) << pt.x(), pt.y(), pt.z();
    }

    return out;
}

// "Direct Form II Transposed" implementation of the standard difference
//  equation, assuming 'b' vector is all ones
std::vector<double> df2t(const MatrixXd& x, size_t window)
{
    const int x_sz = x.rows();
    std::vector<double> y(x_sz, 0);
    for (int ii = 0; ii < x_sz; ++ii) {
        y[ii] = 0;
        for (size_t jj = 0; jj < window; ++jj) {
            int idx = ii - jj;
            if (idx >= 0) {
                y[ii] += x(idx, 0);
            }
        }
        y[ii] /= (float)window;
    }
    return y;
}

MatrixXd movingAverage(const MatrixXd& in, int window)
{
    MatrixXd out(in.rows(), in.cols());
    int row_ct = 0;

    std::vector<double> y_filt = df2t(in, window);

    std::vector<double> begin(window - 2);
    begin[0] = in(0, 0);
    for (int ii = 1; ii < window - 2; ++ii) {
        begin[ii] = in(ii, 0) + begin[ii - 1];
    }

    for (int ii = 0; ii < window - 2; ii += 2, row_ct++) {
        out(row_ct, 0) = begin[ii] / (ii + 1);
    }

    for (size_t ii = window - 1; ii < y_filt.size(); ++ii, ++row_ct) {
        out(row_ct, 0) = y_filt[ii];
    }

    const size_t sz_in = in.size() - 1;
    std::vector<double> end;
    end.push_back(in(in.rows() - 1, 0));
    for (size_t ii = sz_in - 1; ii >= (sz_in - window + 3); --ii) {
        end.push_back(in(ii, 0) + end.back());
    }

    for (int ii = end.size() - 1, jj = 0; ii >= 0; ii -= 2, jj += 2, row_ct++) {
        out(row_ct, 0) = end[ii] / (window - 2 - jj);
    }

    return out;
}

// Finds the relative rotation between the camera and IMU when using the
// provided time offset td. Gyroscope bias is estimated in the process.
//
// INPUT:    angVis  : Visual angular velocities [rad/s] (Mx3 matrix)
//           angImu  : Inertial angular velocities [rad/s] (Mx3 matrix)
//           t       : Timestamps in seconds
//           td      : Time offset in seconds
//
// OUTPUT:   Rs      : Rotation between the camera and IMU (3x3 matrix)
//           bias    : Gyroscope bias [rad/s] (1x3 vector)
//           f       : Function value (sum of squared differences)
//
void solveClosedForm(const MatrixXd& angVis, const MatrixXd& angImu,
                     const std::vector<double>& t, double td, Matrix3d& Rs,
                     MatrixXd& bias, double& f)
{
    assert(angVis.rows() == angImu.rows());
    assert(angVis.rows() == (int)t.size());

    const size_t N = angVis.rows();

    // Adjust visual angular velocities based on current offset
    //  * we pretend the angVis data are collected -td before
    //    their timestamps, and then linearlly interpolate quaternions
    //    at T=t1-td and T=t2-td to find the value at T=t1
    //  * we linearly interpret euler angles. This is bad and it
    //    really should be slerping quaternions
    VectorXd t_td(N);
    for (size_t ii = 0; ii < t.size(); ++ii) {
        t_td(ii) = t[ii] - td;
    }

    VectorXd angX = angVis.col(0);
    VectorXd angY = angVis.col(1);
    VectorXd angZ = angVis.col(2);

    MatrixXd newAng(N, 3);
    MatrixXd meanVis = MatrixXd::Zero(1, 3);
    newAng.col(0) = linearInterpolation(t_td, angX, t);
    newAng.col(1) = linearInterpolation(t_td, angY, t);
    newAng.col(2) = linearInterpolation(t_td, angZ, t);
    meanVis(0, 0) = newAng.col(0).mean();
    meanVis(0, 1) = newAng.col(1).mean();
    meanVis(0, 2) = newAng.col(2).mean();

    MatrixXd meanImu = MatrixXd::Zero(1, 3);
    for (size_t ii = 0; ii < N; ++ii) {
        meanImu += angImu.row(ii);
    }
    meanImu = meanImu / N;

    // Center the point sets
    MatrixXd cenAngImu(N, 3);
    for (size_t ii = 0; ii < N; ++ii) {
        cenAngImu.row(ii) = angImu.row(ii) - meanImu;
    }

    MatrixXd cenAngVis(N, 3);
    for (size_t ii = 0; ii < N; ++ii) {
        cenAngVis.row(ii) = newAng.row(ii) - meanVis;
    }

    // NOTE: May be slow for large matrices; if so, try BDCSVD
    Eigen::JacobiSVD<MatrixXd> svd(cenAngImu.transpose() * cenAngVis,
                                   Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Ensure a right-handed coordinate system and correct if necessary
    Matrix3d C = Matrix3d::Identity();
    if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0) {
        C(2, 2) = -1;
    }

    // OUTPUT: Rotation between camera and IMU
    Rs = svd.matrixV() * C * svd.matrixU().transpose();

    // OUTPUT: The gyroscope bias is the translation
    bias = meanVis - meanImu * Rs;

    // Residual
    MatrixXd D = newAng - (angImu * Rs.transpose() + bias.replicate(N, 1));

    // OUTPUT
    f = D.array().square().sum();
}

// Estimate temporal and spatial alignment between the camera and IMU.
// Gyroscope bias is also estimated in the process.
//
// INPUT:    qtVis   : Visual orientations (Nx4 matrix)
//           tVis    : Visual timestamps in seconds (Nx1 vector)
//           angImu  : Inertial angular velocities [rad/s] (Mx3 matrix)
//           tImu    : Inertial timestamps in seconds (Mx1 vector)
//
// OUTPUT:   Rs      : Rotation between the camera and IMU (3x3 matrix)
//           td      : Time offset between the camera and IMU (scalar)
//           bg      : Gyroscope bias [rad/s] (1x3 vector)
//
// RETURN:   Number of golden-section search iterations
//
int estimateAlignment(const ArrayXXd& in_qtVis,
                      const std::vector<double>& in_tVis,
                      const ArrayXXd& in_angImu,
                      const std::vector<double>& in_tImu, Matrix3d& Rs,
                      double& td, Vector3d& bg)
{
    assert(in_qtVis.rows() == (int)in_tVis.size());
    assert(in_angImu.rows() == (int)in_tImu.size());

    // Only use time spans where both sensors have values
    const double timeStop = std::min(in_tVis.back(), in_tImu.back());

    // Truncate input that extends beyond timeStop
    size_t vis_sz = in_tVis.size();
    for (size_t ii = 0; ii < in_tVis.size(); ++ii) {
        if (timeStop < in_tVis[ii]) {
            vis_sz = ii;
            break;
        }
    }

    size_t imu_sz = in_tImu.size();
    for (size_t ii = 0; ii < in_tImu.size(); ++ii) {
        if (timeStop < in_tImu[ii]) {
            imu_sz = ii;
            break;
        }
    }

    std::vector<double> tVis(&in_tVis[0], &in_tVis[vis_sz]);
    std::vector<double> tImu(&in_tImu[0], &in_tImu[imu_sz]);

    ArrayXXd qtVis = in_qtVis.block(0, 0, vis_sz, 4);
    ArrayXXd angImu = in_angImu.block(0, 0, imu_sz, 3);

    // Upsample visual data to match the sampling of the IMU
    double dt = 0.0;
    for (size_t ii = 1; ii < imu_sz; ++ii) {
        dt += tImu[ii] - tImu[ii - 1];
    }
    dt /= (imu_sz - 1);

    VectorXd t_vis(vis_sz); // temporary
    for (size_t ii = 0; ii < vis_sz; ++ii) {
        t_vis(ii) = tVis[ii];
    }

    // NB: We really should be using slerps here, not linear interpolation
    MatrixXd newQtVis(imu_sz, 4);
    newQtVis.col(0) = linearInterpolation(t_vis, qtVis.col(0), tImu);
    newQtVis.col(1) = linearInterpolation(t_vis, qtVis.col(1), tImu);
    newQtVis.col(2) = linearInterpolation(t_vis, qtVis.col(2), tImu);
    newQtVis.col(3) = linearInterpolation(t_vis, qtVis.col(3), tImu);

    // Compute visual angular velocities
    MatrixXd qtDiffs(imu_sz, 4);
    for (size_t ii = 1; ii < imu_sz; ++ii) {
        qtDiffs.row(ii - 1) = newQtVis.row(ii) - newQtVis.row(ii - 1);
    }
    qtDiffs.row(imu_sz - 1) = qtDiffs.row(imu_sz - 2);

    MatrixXd qtAngVis(imu_sz, 3);
    const double mm = -2.0 / dt;
    for (size_t ii = 0; ii < imu_sz; ++ii) {
        Quaterniond a(qtDiffs(ii, 0), qtDiffs(ii, 1), qtDiffs(ii, 2),
                      qtDiffs(ii, 3));
        Quaterniond b(newQtVis(ii, 0), newQtVis(ii, 1), newQtVis(ii, 2),
                      newQtVis(ii, 3));
        Quaterniond c = a * b.inverse();
        qtAngVis.row(ii) << mm * c.x(), mm * c.y(), mm * c.z();
    }

    qtAngVis.col(0) = movingAverage(qtAngVis.col(0), 15);
    qtAngVis.col(1) = movingAverage(qtAngVis.col(1), 15);
    qtAngVis.col(2) = movingAverage(qtAngVis.col(2), 15);

    angImu.col(0) = movingAverage(angImu.col(0), 15);
    angImu.col(1) = movingAverage(angImu.col(1), 15);
    angImu.col(2) = movingAverage(angImu.col(2), 15);

    const double gRatio = (1.0f + sqrt(5.0)) / 2.0;
    const double tolerance = 1e-4;

    double maxOffset = 0.5;
    double a = -maxOffset;
    double b = maxOffset;

    double c = b - (b - a) / gRatio;
    double d = a + (b - a) / gRatio;
    int iter = 0;

    while (fabs(c - d) > tolerance) {
        // Evaluate function at f(c) and f(d)
        Matrix3d Rsc, Rsd;
        MatrixXd biasc, biasd;
        double fc, fd;

        solveClosedForm(qtAngVis, angImu, tImu, c, Rsc, biasc, fc);
        solveClosedForm(qtAngVis, angImu, tImu, d, Rsd, biasd, fd);

        if (fc < fd) {
            b = d;
            Rs = Rsc;
            bg = biasc.transpose();
        }
        else {
            a = c;
            Rs = Rsd;
            bg = biasd.transpose();
        }

        c = b - (b - a) / gRatio;
        d = a + (b - a) / gRatio;

        ++iter;
    }

    td = (b + a) / 2; // output

    return iter;
}

// Align inertial and visual-data both temporarily and spatially.
//
// INPUT:    accVis  : Visual acceleration [unknown scale] (Nx3 matrix)
//           qtVis   : Visual orientations (Nx4 matrix)
//           tVis    : Visual timestamps in seconds (Nx1 vector)
//           accImu  : Inertial accelerations [m/s^2] (Mx3 matrix)
//           tImu    : Inertial timestamps in seconds (Mx1 vector)
//           Rs      : Rotation between the camera and IMU (3x3 matrix)
//           td      : Time offset between the camera and IMU (scalar)
//
// OUTPUT:   accVis  : Aligned visual acceleration (Kx3 matrix)
//           qtVis   : Aligned visual orientations (Kx4 matrix)
//           accImu  : Aligned inertial accelerations [m/s^2] (Kx3 matrix)
//           t       : Timestamps in seconds (Kx1 vector)
//
void alignCameraAndIMU(const ArrayXXd& in_accVis, const ArrayXXd& in_qtVis,
                       const std::vector<double>& in_tVis,
                       const ArrayXXd& in_accImu,
                       const std::vector<double>& in_tImu, const Matrix3d& Rs,
                       double td, ArrayXXd& out_accVis, ArrayXXd& out_qtVis,
                       MatrixXd& out_accImu, std::vector<double>& out_t)
{
    // Only use time spans where both sensors have values
    const double timeStop = std::min(in_tVis.back(), in_tImu.back());

    // Truncate input that extends beyond timeStop
    size_t vis_sz = in_tVis.size();
    for (size_t ii = 0; ii < in_tVis.size(); ++ii) {
        if (timeStop < in_tVis[ii]) {
            vis_sz = ii;
            break;
        }
    }

    size_t imu_sz = in_tImu.size();
    for (size_t ii = 0; ii < in_tImu.size(); ++ii) {
        if (timeStop < in_tImu[ii]) {
            imu_sz = ii;
            break;
        }
    }

    std::vector<double> tVis(&in_tVis[0], &in_tVis[vis_sz]);
    std::vector<double> tImu(&in_tImu[0], &in_tImu[imu_sz]);

    ArrayXXd accVis = in_accVis.block(0, 0, vis_sz, 3);
    ArrayXXd qtVis = in_qtVis.block(0, 0, vis_sz, 4);
    ArrayXXd accImu = in_accImu.block(0, 0, imu_sz, 3);

    // Upsample visual data to match the sampling of the IMU
    VectorXd t_vis(vis_sz); // temporary
    for (size_t ii = 0; ii < vis_sz; ++ii) {
        t_vis(ii) = tVis[ii] - td;
    }

    const size_t K = tImu.size();
    out_accVis = ArrayXXd::Zero(K, 3);
    out_accVis.col(0) = linearInterpolation(t_vis, accVis.col(0), tImu);
    out_accVis.col(1) = linearInterpolation(t_vis, accVis.col(1), tImu);
    out_accVis.col(2) = linearInterpolation(t_vis, accVis.col(2), tImu);

    out_qtVis = ArrayXXd::Zero(K, 4);
    out_qtVis.col(0) = linearInterpolation(t_vis, qtVis.col(0), tImu);
    out_qtVis.col(1) = linearInterpolation(t_vis, qtVis.col(1), tImu);
    out_qtVis.col(2) = linearInterpolation(t_vis, qtVis.col(2), tImu);
    out_qtVis.col(3) = linearInterpolation(t_vis, qtVis.col(3), tImu);

    out_accImu = accImu.matrix() * Rs;

    out_t = tImu;
}

// Initial estimates for scale, gravity and IMU bias
//
// INPUT:    accVis  : Visual acceleration [unknown scale] (Nx3 matrix)
//           qtVis   : Visual orientations (Nx4 matrix)
//           accImu  : Inertial accelerations [m/s^2] (Mx3 matrix)
//
// OUTPUT:   A       : Matrix of visual accelerations and rotations (3*Nx7
// matrix)
//           b       : Row-matrix of IMU accelerations (3*Nx1 matrix)
//           scale   : Initial estimate at scale
//           gravity : The initial gravity vector estimate (3x1 vector)
//           bias    : Initial estimate at gyroscope bias(3x1 vector)
//
void initializeEstimates(const ArrayXXd& accVis, const ArrayXXd& qtVis,
                         const ArrayXXd& accImu, MatrixXd& A, MatrixXd& b,
                         double& scale, Vector3d& g, Vector3d& bias)
{
    const size_t N = accVis.rows();

    A.resize(3 * N, 7);
    b.resize(3 * N, 1);
    for (size_t nn = 0; nn < N; ++nn) {
        const size_t rr = 3 * nn;

        A.block<3, 1>(rr, 0) = accVis.row(nn).transpose();
        Quaterniond qq(qtVis(nn, 0), qtVis(nn, 1), qtVis(nn, 2), qtVis(nn, 3));
        A.block<3, 3>(rr, 1) = qq.normalized().toRotationMatrix();
        A.block<3, 3>(rr, 4) = Matrix3d::Identity();

        b.block<3, 1>(rr, 0) = accImu.row(nn).transpose();
    }

    // Using Householder rank-revealing QR decomposition of a matrix with
    // column-pivoting.  This technique provides a food compromise between
    // speed and accuracy:
    //   https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
    VectorXd x = A.colPivHouseholderQr().solve(b);

    scale = x(0);
    g << x(1), x(2), x(3);
    bias << x(4), x(5), x(6);
}

} // namespace

class InertialBasedScaleEstimation::Impl
{
public:
    enum class InternalState
    {
        NotStarted = 0,
        DataSet,
        InitialAlignment,
        ScaleEstimated,
    };

    ErrorCode setVisualData(const std::vector<double>& time,
                            const std::vector<Vector3d>& position,
                            const std::vector<Quaterniond>& rotation);

    ErrorCode setInertialData(const std::vector<double>& accelerometer_time,
                              const std::vector<Vector3d>& linear_acceleration,
                              const std::vector<double>& gyroscope_time,
                              const std::vector<Vector3d>& angular_velocity);

    ErrorCode initialAlignmentEstimation();

    ErrorCode estimateScale(double& scale, Vector3d& g, Vector3d& bias,
                            double fMax);

public:
    InternalState _state;

    // Visual data
    std::vector<double> _visTime;
    double _visDt;
    std::vector<Vector3d> _visPosition;
    ArrayXXd _visQuat;

    // Inertial data
    std::vector<double> _imuTime;
    ArrayXXd _imuAccleration;
    ArrayXXd _imuAngVel;

    // generated by initialAlignmentEstimation
    Matrix3d _Rs; // IMU to camera rotation
    MatrixXd _A;
    MatrixXd _b;
    double _scale0;
    double _td; // IMU to camera time offset
    Vector3d _g0;
    Vector3d _bias0;
    std::vector<double> _alignTime;
};

InertialBasedScaleEstimation::ErrorCode
InertialBasedScaleEstimation::Impl::setVisualData(
    const std::vector<double>& time, const std::vector<Vector3d>& position,
    const std::vector<Quaterniond>& rotation)
{
    if ((time.size() != position.size()) ||
        (position.size() != rotation.size())) {
        return ErrorCode::InconsistentDataSizes;
    }

    _visTime = time;
    // Normalize time data
    double t0 = _visTime[0];
    for (auto& tt : _visTime) {
        tt = tt - t0;
    }

    _visDt = 0;
    const size_t szTime = _visTime.size();
    for (size_t ii = 1; ii < szTime; ++ii) {
        _visDt += (_visTime[ii] - _visTime[ii - 1]);
    }
    _visDt = _visDt / (szTime - 1);

    _visPosition = position;

    // Copy the rotations into an ArrayXX for faster processing later
    const size_t szRot = rotation.size();
    _visQuat.resize(szRot, 4);
    for (size_t ii = 0; ii < szRot; ++ii) {
        _visQuat.row(ii) << rotation[ii].w(), rotation[ii].x(),
            rotation[ii].y(), rotation[ii].z();
    }

    _state = InternalState::DataSet;
    return ErrorCode::Success;
}

InertialBasedScaleEstimation::ErrorCode
InertialBasedScaleEstimation::Impl::setInertialData(
    const std::vector<double>& accTime, const std::vector<Vector3d>& accData,
    const std::vector<double>& gyrTime, const std::vector<Vector3d>& gyrData)
{
    if (accData.empty() || gyrData.empty()) {
        return ErrorCode::NoData;
    }
    if ((accTime.size() != accData.size()) ||
        (gyrTime.size() != gyrData.size())) {
        return ErrorCode::InconsistentDataSizes;
    }

    // Copy the linear accelerations into an ArrayXX for faster processing later
    _imuAccleration.resize(accData.size(), 3);
    for (size_t i{0}; i < accData.size(); ++i) {
        _imuAccleration.row(i) = accData[i];
    }

    // Resample gyroscope readings to match the sampling of the accelerometer
    double t0 = std::min(accTime[0], gyrTime[0]);
    _imuTime.clear();
    for (const auto& tt : accTime) {
        _imuTime.push_back(tt - t0);
    }

    const size_t gyrCount = gyrTime.size();
    VectorXd timeGyr(gyrCount);
    for (size_t i{0}; i < gyrCount; ++i) {
        timeGyr(i) = gyrTime[i] - t0;
    }

    VectorXd gyrImu_x(gyrCount);
    VectorXd gyrImu_y(gyrCount);
    VectorXd gyrImu_z(gyrCount);
    for (size_t i{0}; i < gyrCount; ++i) {
        gyrImu_x(i) = gyrData[i].x();
        gyrImu_y(i) = gyrData[i].y();
        gyrImu_z(i) = gyrData[i].z();
    }

    _imuAngVel.resize(_imuTime.size(), 3);
    _imuAngVel.col(0) = linearInterpolation(timeGyr, gyrImu_x, _imuTime);
    _imuAngVel.col(1) = linearInterpolation(timeGyr, gyrImu_y, _imuTime);
    _imuAngVel.col(2) = linearInterpolation(timeGyr, gyrImu_z, _imuTime);

    _state = InternalState::DataSet;
    return ErrorCode::Success;
};

InertialBasedScaleEstimation::ErrorCode
InertialBasedScaleEstimation::Impl::initialAlignmentEstimation()
{
    if (_state == InternalState::NotStarted) {
        return ErrorCode::NoData;
    }

    // Kalman & RTS filtering
    ArrayXXd visAcc = KalmanRTS(_visPosition, _visDt).transpose();

    // Estimate visual and IMU alignment
    double td;
    Vector3d bg;
    estimateAlignment(_visQuat, _visTime, _imuAngVel, _imuTime, _Rs, td, bg);

    // Align camera and IMU measurements
    ArrayXXd out_accVis;
    ArrayXXd out_qtVis;
    MatrixXd out_accImu;
    alignCameraAndIMU(visAcc, _visQuat, _visTime, _imuAccleration, _imuTime,
                      _Rs, td, out_accVis, out_qtVis, out_accImu, _alignTime);

    // Transform visual accelerations from world frame to local frame
    ArrayXXd rotAccVis = rotatePoints(out_qtVis, out_accVis);

    // Find initial estimates for the scale, gravity and bias by solving
    // an unconstrained linear system of equations Ax = b
    initializeEstimates(rotAccVis, out_qtVis, out_accImu, _A, _b, _scale0, _g0,
                        _bias0);

    _state = InternalState::InitialAlignment;
    return ErrorCode::Success;
}

InertialBasedScaleEstimation::ErrorCode
InertialBasedScaleEstimation::Impl::estimateScale(double& scale, Vector3d& g,
                                                  Vector3d& bias, double fMax)
{
    if (_state == InternalState::NotStarted) {
        return ErrorCode::NoData;
    }
    if (_state == InternalState::DataSet) {
        return ErrorCode::NoInitialAlignment;
    }

    // TODO: Finish here
    // Final estimation in the frequency domain
    // bool success = optimizeEstimate(_A, _b, _scale0, _g0, _bias0, _alignTime,
    //                                 fMax, scale, g, bias);

    // if (success) {
    //     _state = InternalState::SCALE_ESTIMATED;
    //     return ErrorCode::SUCCESS;
    // }

    return ErrorCode::FailedFinalEstimation;
}

///------- InertialBaseScaleEstimation starts from here
InertialBasedScaleEstimation::InertialBasedScaleEstimation()
    : d(std::make_unique<Impl>())
{
}

InertialBasedScaleEstimation::~InertialBasedScaleEstimation() = default;

InertialBasedScaleEstimation::ErrorCode
InertialBasedScaleEstimation::setVisualData(
    const std::vector<double>& time, const std::vector<Vector3d>& position,
    const std::vector<Quaterniond>& rotation)
{
    return d->setVisualData(time, position, rotation);
}

InertialBasedScaleEstimation::ErrorCode
InertialBasedScaleEstimation::setInertialData(
    const std::vector<double>& accelerometer_time,
    const std::vector<Vector3d>& linear_acceleration,
    const std::vector<double>& gyroscope_time,
    const std::vector<Vector3d>& angular_velocity)
{
    return d->setInertialData(accelerometer_time, linear_acceleration,
                              gyroscope_time, angular_velocity);
}

InertialBasedScaleEstimation::ErrorCode
InertialBasedScaleEstimation::initialAlignmentEstimation()
{
    return d->initialAlignmentEstimation();
}

InertialBasedScaleEstimation::ErrorCode
InertialBasedScaleEstimation::estimateScale(double& scale, Vector3d& g,
                                            Vector3d& bias, double fMax)
{
    return d->estimateScale(scale, g, bias, fMax);
}

Matrix3d InertialBasedScaleEstimation::imuToCameraRotation() const
{
    return d->_Rs;
}

double InertialBasedScaleEstimation::imuToCameraTimeOffset() const
{
    return d->_td;
}

} // namespace tl
