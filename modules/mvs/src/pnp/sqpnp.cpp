#include "sqpnp.h"
#include "impl/sqpnp_impl.h"

#include <numbers>

#include <tCore/Math>

namespace tl {

namespace {

using NearestRotationMatrixFinder =
    std::function<void(const Vector9d&, Vector9d&)>;

SQPSolution runSQP(const Vector9d& r0, const Matrix9d& omega,
                   NearestRotationMatrixFinder findNearestRotationMatrix)
{
    int step{0};
    Vector9d delta;
    Vector9d r = r0;
    auto delta_squared_norm = kMaxDouble;
    while (delta_squared_norm > DEFAULT_SQP_SQUARED_TOLERANCE &&
           step++ < DEFAULT_SQP_SQUARED_TOLERANCE) {
        SolveSQPSystem(r, omega, delta);
        r += delta;
        delta_squared_norm = delta.squaredNorm();
    }

    SQPSolution solution;
    solution.num_iterations = step;
    solution.r = r;
    // clear the estimate and/or flip the matrix sign if necessary
    double det_r = Determinant9x1(solution.r);
    if (det_r < 0) {
        solution.r = -r;
        det_r = -det_r;
    }
    if (det_r > DEFAULT_SQP_DET_THRESHOLD) {
        findNearestRotationMatrix(solution.r, solution.r_hat);
    }
    else {
        solution.r_hat = solution.r;
    }
    return solution;
}

} // namespace

bool SQPnP(const Vector2dList& image_points, const Vector3dList& world_points,
           QuaterniondList& rotations, Vector3dList& translations)
{
    // TODO: assert equal size
    size_t N = world_points.size();
    if (N != image_points.size() || N < 3 || image_points.size() < 3) {
        return false;
    }

    std::vector<double> weights;
    weights.resize(N, 1.0);
    Matrix9d Omega_ = Matrix9d::Zero();
    double sum_wx = 0, sum_wy = 0, sum_wx2_plus_wy2 = 0, sum_w = 0;
    double sum_X = 0, sum_Y = 0, sum_Z = 0;

    Matrix39 QA = Matrix39::Zero(); // Sum( Qi*Ai )

    for (size_t i = 0; i < N; i++) {
        const double& w = weights[i];

        double wx = image_points[i][0] * w;
        double wy = image_points[i][1] * w;
        double wsq_norm_m = w * image_points[i].squaredNorm();
        sum_wx += wx;
        sum_wy += wy;
        sum_wx2_plus_wy2 += wsq_norm_m;
        sum_w += w;

        double X = world_points[i][0];
        double Y = world_points[i][1];
        double Z = world_points[i][2];
        sum_X += X;
        sum_Y += Y;
        sum_Z += Z;

        // Accumulate Omega by kronecker( Qi, Mi*Mi' ) = A'*Qi*Ai.
        // NOTE: Skipping block (3:5, 3:5) because its same as (0:2, 0:2)
        double X2 = X * X;
        double XY = X * Y;
        double XZ = X * Z;
        double Y2 = Y * Y;
        double YZ = Y * Z;
        double Z2 = Z * Z;

        // a. Block (0:2, 0:2) populated by Mi*Mi'. NOTE: Only upper triangle
        Omega_(0, 0) += w * X2;
        Omega_(0, 1) += w * XY;
        Omega_(0, 2) += w * XZ;
        Omega_(1, 1) += w * Y2;
        Omega_(1, 2) += w * YZ;
        Omega_(2, 2) += w * Z2;

        // b. Block (0:2, 6:8) populated by -x*Mi*Mi'. NOTE: Only upper triangle
        Omega_(0, 6) += -wx * X2;
        Omega_(0, 7) += -wx * XY;
        Omega_(0, 8) += -wx * XZ;
        Omega_(1, 7) += -wx * Y2;
        Omega_(1, 8) += -wx * YZ;
        Omega_(2, 8) += -wx * Z2;

        // c. Block (3:5, 6:8) populated by -y*Mi*Mi'. NOTE: Only upper triangle
        Omega_(3, 6) += -wy * X2;
        Omega_(3, 7) += -wy * XY;
        Omega_(3, 8) += -wy * XZ;
        Omega_(4, 7) += -wy * Y2;
        Omega_(4, 8) += -wy * YZ;
        Omega_(5, 8) += -wy * Z2;

        // d. Block (6:8, 6:8) populated by (x^2+y^2)*Mi*Mi'. NOTE: Only upper
        // triangle
        Omega_(6, 6) += wsq_norm_m * X2;
        Omega_(6, 7) += wsq_norm_m * XY;
        Omega_(6, 8) += wsq_norm_m * XZ;
        Omega_(7, 7) += wsq_norm_m * Y2;
        Omega_(7, 8) += wsq_norm_m * YZ;
        Omega_(8, 8) += wsq_norm_m * Z2;

        // Accumulating Qi*Ai in QA
        double wX = w * X;
        double wY = w * Y;
        double wZ = w * Z;
        QA(0, 0) += wX;
        QA(0, 1) += wY;
        QA(0, 2) += wZ;
        QA(0, 6) += -wx * X;
        QA(0, 7) += -wx * Y;
        QA(0, 8) += -wx * Z;
        QA(1, 3) += wX;
        QA(1, 4) += wY;
        QA(1, 5) += wZ;
        QA(1, 6) += -wy * X;
        QA(1, 7) += -wy * Y;
        QA(1, 8) += -wy * Z;

        QA(2, 0) += -wx * X;
        QA(2, 1) += -wx * Y;
        QA(2, 2) += -wx * Z;
        QA(2, 3) += -wy * X;
        QA(2, 4) += -wy * Y;
        QA(2, 5) += -wy * Z;
        QA(2, 6) += wsq_norm_m * X;
        QA(2, 7) += wsq_norm_m * Y;
        QA(2, 8) += wsq_norm_m * Z;
    }

    // Fill-in lower triangles of off-diagonal blocks (0:2, 6:8), (3:5, 6:8) and
    // (6:8, 6:8)
    Omega_(1, 6) = Omega_(0, 7);
    Omega_(2, 6) = Omega_(0, 8);
    Omega_(2, 7) = Omega_(1, 8);
    Omega_(4, 6) = Omega_(3, 7);
    Omega_(5, 6) = Omega_(3, 8);
    Omega_(5, 7) = Omega_(4, 8);
    Omega_(7, 6) = Omega_(6, 7);
    Omega_(8, 6) = Omega_(6, 8);
    Omega_(8, 7) = Omega_(7, 8);

    // Fill-in upper triangle of block (3:5, 3:5)
    Omega_(3, 3) = Omega_(0, 0);
    Omega_(3, 4) = Omega_(0, 1);
    Omega_(3, 5) = Omega_(0, 2);
    Omega_(4, 4) = Omega_(1, 1);
    Omega_(4, 5) = Omega_(1, 2);
    Omega_(5, 5) = Omega_(2, 2);
    // Fill lower triangle of Omega
    Omega_(1, 0) = Omega_(0, 1);
    Omega_(2, 0) = Omega_(0, 2);
    Omega_(2, 1) = Omega_(1, 2);
    Omega_(3, 0) = Omega_(0, 3);
    Omega_(3, 1) = Omega_(1, 3);
    Omega_(3, 2) = Omega_(2, 3);
    Omega_(4, 0) = Omega_(0, 4);
    Omega_(4, 1) = Omega_(1, 4);
    Omega_(4, 2) = Omega_(2, 4);
    Omega_(4, 3) = Omega_(3, 4);
    Omega_(5, 0) = Omega_(0, 5);
    Omega_(5, 1) = Omega_(1, 5);
    Omega_(5, 2) = Omega_(2, 5);
    Omega_(5, 3) = Omega_(3, 5);
    Omega_(5, 4) = Omega_(4, 5);
    Omega_(6, 0) = Omega_(0, 6);
    Omega_(6, 1) = Omega_(1, 6);
    Omega_(6, 2) = Omega_(2, 6);
    Omega_(6, 3) = Omega_(3, 6);
    Omega_(6, 4) = Omega_(4, 6);
    Omega_(6, 5) = Omega_(5, 6);
    Omega_(7, 0) = Omega_(0, 7);
    Omega_(7, 1) = Omega_(1, 7);
    Omega_(7, 2) = Omega_(2, 7);
    Omega_(7, 3) = Omega_(3, 7);
    Omega_(7, 4) = Omega_(4, 7);
    Omega_(7, 5) = Omega_(5, 7);
    Omega_(7, 6) = Omega_(6, 7);
    Omega_(8, 0) = Omega_(0, 8);
    Omega_(8, 1) = Omega_(1, 8);
    Omega_(8, 2) = Omega_(2, 8);
    Omega_(8, 3) = Omega_(3, 8);
    Omega_(8, 4) = Omega_(4, 8);
    Omega_(8, 5) = Omega_(5, 8);
    Omega_(8, 6) = Omega_(6, 8);
    Omega_(8, 7) = Omega_(7, 8);

    // Q = Sum( wi*Qi ) = Sum( [ wi, 0, -wi*xi; 0, 1, -wi*yi; -wi*xi, -wi*yi,
    // wi*(xi^2 + yi^2)] )
    Eigen::Matrix3d Q;
    Q(0, 0) = sum_w;
    Q(0, 1) = 0;
    Q(0, 2) = -sum_wx;
    Q(1, 0) = 0;
    Q(1, 1) = sum_w;
    Q(1, 2) = -sum_wy;
    Q(2, 0) = -sum_wx;
    Q(2, 1) = -sum_wy;
    Q(2, 2) = sum_wx2_plus_wy2;

    // Qinv = inv( Q ) = inv( Sum( Qi) )
    Eigen::Matrix3d Qinv;
    InvertSymmetric3x3(Q, Qinv);

    // Compute P = -inv( Sum(wi*Qi) ) * Sum( wi*Qi*Ai ) = -Qinv * QA
    const Matrix39 P = -Qinv * QA;
    // Complete Omega (i.e., Omega = Sum(A'*Qi*A') + Sum(Qi*Ai)'*P =
    // Sum(A'*Qi*A')
    // + Sum(Qi*Ai)'*inv(Sum(Qi))*Sum( Qi*Ai)
    Omega_ += QA.transpose() * P;

    // Finally, decompose Omega
    Eigen::JacobiSVD<Matrix9d> svd{Omega_, Eigen::ComputeFullU};
    const auto& U = svd.matrixU();
    const auto& singularValues = svd.singularValues();

    // Find dimension of null space
    int nullSpaceDim{0};
    while (singularValues[7 - nullSpaceDim] < DEFAULT_RANK_TOLERANCE) {
        nullSpaceDim++;
    }

    // Dimension of null space of Omega must be <= 6
    constexpr int kMaxOmegaNullSpaceDim{6};
    if (++nullSpaceDim > kMaxOmegaNullSpaceDim) {
        return false;
    }

    // 3D point mean (for cheirality checks)
    const double inv_n = 1.0 / N;
    Eigen::Vector3d point_mean{sum_X * inv_n, sum_Y * inv_n, sum_Z * inv_n};

    NearestRotationMatrixFinder findNearestRotationMatrix;
    if (DEFAULT_NEAREST_ROTATION_METHOD == NearestRotationMethod::FOAM) {
        findNearestRotationMatrix = findNearestRotationMatrixByFOAM;
    }
    else {
        findNearestRotationMatrix = findNearestRotationMatrixBySVD;
    }

    auto min_sq_error = kMaxDouble;
    int num_eigen_points = nullSpaceDim > 0 ? nullSpaceDim : 1;
    // clear solutions
    SQPSolution solutions_[18];
    int num_solutions_ = 0;
    for (int i = 9 - num_eigen_points; i < 9; i++) {
        // NOTE: No need to scale by sqrt3 here, but better be there for other
        // computations (i.e., orthogonality test)
        // Eigen::Map<Matrix91>(U.block<9, 1>(0, i).data());
        const Vector9d e = sqrt3 * U.col(i);
        double orthogonality_sq_error = OrthogonalityError(e);
        // Find nearest rotation vector
        SQPSolution solution[2];

        // Avoid SQP if e is orthogonal
        if (orthogonality_sq_error <
            DEFAULT_ORTHOGONALITY_SQUARED_ERROR_THRESHOLD) {
            solution[0].r_hat = Determinant9x1(e) * e;
            solution[0].t = P * solution[0].r_hat;
            solution[0].num_iterations = 0;

            HandleSolution(Omega_, point_mean, solution[0], solutions_,
                           min_sq_error, num_solutions_);
        }
        else {
            findNearestRotationMatrix(e, solution[0].r);
            solution[0] =
                runSQP(solution[0].r, Omega_, findNearestRotationMatrix);
            solution[0].t = P * solution[0].r_hat;
            HandleSolution(Omega_, point_mean, solution[0], solutions_,
                           min_sq_error, num_solutions_);

            findNearestRotationMatrix(-e, solution[1].r);
            solution[1] =
                runSQP(solution[1].r, Omega_, findNearestRotationMatrix);
            solution[1].t = P * solution[1].r_hat;
            HandleSolution(Omega_, point_mean, solution[1], solutions_,
                           min_sq_error, num_solutions_);
        }
    }

    int c = 1;
    while (min_sq_error > 3 * singularValues[9 - num_eigen_points - c] &&
           9 - num_eigen_points - c > 0) {
        int index = 9 - num_eigen_points - c;

        // Eigen::Map<Matrix91>(U.block<9, 1>(0, index).data());
        const Vector9d e = U.col(index);
        SQPSolution solution[2];

        findNearestRotationMatrix(e, solution[0].r);
        solution[0] = runSQP(solution[0].r, Omega_, findNearestRotationMatrix);
        solution[0].t = P * solution[0].r_hat;
        HandleSolution(Omega_, point_mean, solution[0], solutions_,
                       min_sq_error, num_solutions_);

        findNearestRotationMatrix(-e, solution[1].r);
        solution[1] = runSQP(solution[1].r, Omega_, findNearestRotationMatrix);
        solution[1].t = P * solution[1].r_hat;
        HandleSolution(Omega_, point_mean, solution[1], solutions_,
                       min_sq_error, num_solutions_);

        c++;
    }

    for (int i = 0; i < num_solutions_; i++) {
        Eigen::Map<Eigen::Matrix3d> R(solutions_[i].r_hat.data(), 3, 3);
        rotations.emplace_back(Eigen::Quaterniond(R.transpose()));
        translations.emplace_back(solutions_[i].t);
    }

    return !rotations.empty();
}

} // namespace tl
