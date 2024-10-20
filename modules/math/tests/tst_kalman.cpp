#include <gtest/gtest.h>

// WARNING: Only for test purpose
#define private public
#define protected public

#include <tMath/KalmanFilter/ExtendedKalmanFilter>
#include <tMath/KalmanFilter/Types>
#include <tMath/KalmanFilter/UnscentedKalmanFilter>
#include <tMath/KalmanFilter/SquareRootExtendedKalmanFilter>
#include <tMath/KalmanFilter/SquareRootUnscentedKalmanFilter>

using T = float;

using namespace tl::kalman;

using Vec3f = Vector<T, 3>;

TEST(Cholesky, solve)
{
    // Set S (lower choleksy factor)
    Matrix<T, 3, 3> S_;
    // clang-format off
    S_ << 1, 0, 0,
          1, 1, 0,
          1, 1, 2;
    // clang-format on

    Cholesky<Matrix<T, 3, 3>> S;
    S.setL(S_);

    // Set P
    Matrix<T, 4, 3> P;
    // clang-format off
    P << 6, 4, 2,
         4, 8, 6,
         2, 6, 0,
         3, 5, 7;
    // clang-format on

    // Compute K
    Matrix<T, 4, 3> K = S.solve(P.transpose()).transpose();

    Matrix<T, 4, 3> P_ =
        K * S.matrixL().toDenseMatrix() * S.matrixU().toDenseMatrix();

    EXPECT_TRUE(P.isApprox(P_));
}

TEST(QR, compute)
{
    Matrix<T, 3, 5> A;
    // clang-format off
    A << 8, 7, 1, 1, 4,
         8, 2, 3, 9, 10,
         4, 8, 1, 7, 1;
    // clang-format on

    Eigen::HouseholderQR<Matrix<T, 5, 3>> qr(A.transpose());

    auto R = qr.matrixQR();

    ASSERT_FLOAT_EQ(-11.445523142259598f, R(0, 0));
    ASSERT_FLOAT_EQ(-11.358152736593492f, R(0, 1));
    ASSERT_FLOAT_EQ(-8.737040566610379f, R(0, 2));
    ASSERT_FLOAT_EQ(11.357480636664706f, R(1, 1));
    ASSERT_FLOAT_EQ(2.180356680396514f, R(1, 2));
    ASSERT_FLOAT_EQ(-7.064712795553326f, R(2, 2));
}

TEST(SquareRootBase, setCovariance)
{
    Covariance<Vec3f> cov;
    // clang-format off
    cov << 1, 2, 4,
           2, 13, 23,
           4, 23, 77;
    // clang-format on

    SquareRootBase<Vec3f> S;
    S.setCovariance(cov);

    EXPECT_TRUE(cov.isApprox(S.S.reconstructedMatrix(), 1e-5));
}

TEST(StandardBase, setCovarianceSquareRoot)
{
    Covariance<Vec3f> sqrRoot;
    // clang-format off
    sqrRoot << 1, 0, 0,
               2, 3, 0,
               4, 5, 6;
    // clang-format on

    Covariance<Vec3f> cov;
    // clang-format off
    cov << 1, 2, 4,
           2, 13, 23,
           4, 23, 77;
    // clang-format on

    StandardBase<Vec3f> S;
    S.setCovarianceSquareRoot(sqrRoot);

    EXPECT_TRUE(cov.isApprox(S.P, 1e-5));
}

TEST(ExtendedKalmanFilter, init)
{
    ExtendedKalmanFilter<Vector<T, 3>> ekf;
    ASSERT_TRUE(ekf.P.isIdentity()); // P should be identity

    // Same as above, but with general matrix type instead of vector
    ExtendedKalmanFilter<Matrix<T, 3, 1>> ekfMatrix;
    ASSERT_TRUE(ekfMatrix.P.isIdentity()); // P should be identity
}

TEST(SquareRootUnscentedKalmanFilter, init)
{
    auto ukf = SquareRootUnscentedKalmanFilter<Vector<T, 3>>(1, 2, 1);
    ASSERT_TRUE(ukf.S.isIdentity()); // S should be identity

    // Same as above, but with general matrix type instead of vector
    auto ukfMatrix = SquareRootUnscentedKalmanFilter<Matrix<T, 3, 1>>(1, 2, 1);
    ASSERT_TRUE(ukfMatrix.S.isIdentity()); // S should be identity
}

TEST(SquareRootUnscentedKalmanFilter, computeSigmaPoints)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf =
        SquareRootUnscentedKalmanFilter<Vector<T, 3>>(alpha, beta, kappa);

    // Init variables
    ukf.gamma = 2;
    ukf.x << 1.f, 2.f, 3.f;
    ukf.S.setIdentity();

    ukf.computeSigmaPoints();

    ASSERT_FLOAT_EQ(1, ukf.sigmaStatePoints(0, 0));
    ASSERT_FLOAT_EQ(2, ukf.sigmaStatePoints(1, 0));
    ASSERT_FLOAT_EQ(3, ukf.sigmaStatePoints(2, 0));

    // Center block diagonal
    ASSERT_FLOAT_EQ(3, ukf.sigmaStatePoints(0, 1));
    ASSERT_FLOAT_EQ(4, ukf.sigmaStatePoints(1, 2));
    ASSERT_FLOAT_EQ(5, ukf.sigmaStatePoints(2, 3));

    // Right block diagonal
    ASSERT_FLOAT_EQ(-1, ukf.sigmaStatePoints(0, 4));
    ASSERT_FLOAT_EQ(0, ukf.sigmaStatePoints(1, 5));
    ASSERT_FLOAT_EQ(1, ukf.sigmaStatePoints(2, 6));
}

TEST(SquareRootUnscentedKalmanFilter,
     computeCovarianceSquareRootFromSigmaPoints)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf =
        SquareRootUnscentedKalmanFilter<Vector<T, 3>>(alpha, beta, kappa);

    // Compute standard (non-square root) covariance from regular UKF
    // and test equality with square-root solution

    Cholesky<Matrix<T, 4, 4>> Rsqrt;
    Matrix<T, 4, 4> _Rsqrt;
    // clang-format off
    _Rsqrt << 1, 0, 0, 0,
              0, 2, 0, 0,
              0, 0, 3, 0,
              0, 0, 0, 4;
    // clang-format on
    Rsqrt.setL(_Rsqrt);

    Vector<T, 4> mean;
    mean << 2, 3, 4, 5;

    ukf.sigmaWeights_c << 3, 5, 5, 5, 5, 5, 5;

    Matrix<T, 4, 7> sigmaPoints;
    // clang-format off
    sigmaPoints << 1, 2, 1, 1, 0, 1, 1,
                   2, 3, 3, 2, 1, 1, 2,
                   3, 5, 4, 4, 1, 2, 2,
                   3, 4, 5, 2, 0, 4, 3;
    // clang-format on

    Cholesky<Matrix<T, 4, 4>> S;

    ASSERT_TRUE(ukf.computeCovarianceSquareRootFromSigmaPoints(
        mean, sigmaPoints, Rsqrt, S));

    // Compute reference P
    Matrix<T, 4, 4> P = Matrix<T, 4, 4>::Zero();
    for (int i = 0; i <= 2 * 3; ++i) {
        Vector<T, 4> vec = sigmaPoints.col(i) - mean;
        P += ukf.sigmaWeights_c[i] * vec * vec.transpose();
    }
    P += Rsqrt.reconstructedMatrix();

    // Compute squared S
    Matrix<T, 4, 4> Ssquared = S.reconstructedMatrix().eval();

    EXPECT_TRUE(P.isApprox(Ssquared, 3.5e-5));
}

TEST(SquareRootUnscentedKalmanFilter, computeKalmanGain)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf =
        SquareRootUnscentedKalmanFilter<Vector<T, 3>>(alpha, beta, kappa);

    ukf.sigmaWeights_c << 3, 5, 5, 5, 5, 5, 5;

    Vector<T, 2> mean;
    mean << 2, 3;

    typename SquareRootUnscentedKalmanFilter<
        Vector<T, 3>>::template SigmaPoints<decltype(mean)>
        sigmaPoints;
    sigmaPoints << 1, 2, 1, 1, 0, 1, 1, 2, 3, 3, 2, 1, 1, 2;

    Matrix<T, 2, 2> Ssquared;
    // clang-format off
    Ssquared << 35, 34,
                34, 46;
    // clang-format on

    Cholesky<Matrix<T, 2, 2>> S;
    S.compute(Ssquared);
    ASSERT_TRUE(S.info() == Eigen::Success);

    // x and sigmaStatePoints
    ukf.x << 3, 5, 7;
    // clang-format off
    ukf.sigmaStatePoints << 3, 5, 3, 3, 1, 3, 3,
                            5, 7, 7, 5, 3, 3, 5,
                            7, 11, 9,9, 3, 5, 5;
    // clang-format on

    // Reference value for P
    Matrix<T, 3, 2> P_xy_ref;
    // clang-format off
    P_xy_ref << 20, 20, 20,
                40, 40, 60;
    // clang-format on

    // Reference value for K
    Matrix<T, 3, 2> K_ref;
    // clang-format off
    K_ref << 0.528634361233480, 0.044052863436123, -0.969162995594713,
             1.585903083700440, -0.440528634361233, 1.629955947136563;
    // clang-format on

    // Kalman gain to be computed
    Matrix<T, 3, 2> K;
    ukf.computeKalmanGain(mean, sigmaPoints, S, K);

    EXPECT_TRUE(K_ref.isApprox(K, 1e-6));
}

TEST(SquareRootUnscentedKalmanFilter, updateStateCovariance)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf =
        SquareRootUnscentedKalmanFilter<Vector<T, 3>>(alpha, beta, kappa);

    // Setup S_y
    Matrix<T, 2, 2> Ssquared;
    Ssquared << 1, 0.1, 0.1, 0.5;
    Cholesky<Matrix<T, 2, 2>> S_y;
    S_y.compute(0.1 * Ssquared);
    ASSERT_TRUE(S_y.info() == Eigen::Success);

    // Setup Kalman Gain
    Matrix<T, 3, 2> K;
    // clang-format off
    K << 0.178408514951850, -0.304105423213381, -1.110998479472884,
         0.802838317283324, -1.246156445345498, 0.063524243960128;
    // clang-format on

    // Setup S
    ukf.S.setIdentity();

    // Setup reference value for S (computed from regular UKF formula)
    Matrix<T, 3, 3> P_ref = ukf.S.reconstructedMatrix() -
                            K * S_y.reconstructedMatrix() * K.transpose();

    // Perform update
    bool success = ukf.updateStateCovariance<Vector<T, 2>>(K, S_y);
    ASSERT_TRUE(success);

    Matrix<T, 3, 3> P = ukf.S.reconstructedMatrix();

    EXPECT_TRUE(P_ref.isApprox(P, 1e-6));
}

TEST(UnscentedKalmanFilter, init)
{
    auto ukf = UnscentedKalmanFilter<Vector<T, 3>>(1, 2, 1);
    ASSERT_TRUE(ukf.P.isIdentity()); // P should be identity

    // Same as above, but with general matrix type instead of vector
    auto ukfMatrix = UnscentedKalmanFilter<Matrix<T, 3, 1>>(1, 2, 1);
    ASSERT_TRUE(ukfMatrix.P.isIdentity()); // P should be identity
}

TEST(UnscentedKalmanFilter, computeSigmaPoints)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf = UnscentedKalmanFilter<Vector<T, 3>>(alpha, beta, kappa);

    // Init variables
    ukf.gamma = 2;
    ukf.x << 1.f, 2.f, 3.f;
    ukf.P.setIdentity();

    ASSERT_TRUE(ukf.computeSigmaPoints());

    ASSERT_FLOAT_EQ(1, ukf.sigmaStatePoints(0, 0));
    ASSERT_FLOAT_EQ(2, ukf.sigmaStatePoints(1, 0));
    ASSERT_FLOAT_EQ(3, ukf.sigmaStatePoints(2, 0));

    // Center block diagonal
    ASSERT_FLOAT_EQ(3, ukf.sigmaStatePoints(0, 1));
    ASSERT_FLOAT_EQ(4, ukf.sigmaStatePoints(1, 2));
    ASSERT_FLOAT_EQ(5, ukf.sigmaStatePoints(2, 3));

    // Right block diagonal
    ASSERT_FLOAT_EQ(-1, ukf.sigmaStatePoints(0, 4));
    ASSERT_FLOAT_EQ(0, ukf.sigmaStatePoints(1, 5));
    ASSERT_FLOAT_EQ(1, ukf.sigmaStatePoints(2, 6));
}

TEST(UnscentedKalmanFilter, computeCovarianceSquareRootFromSigmaPoints)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf = UnscentedKalmanFilter<Vector<T, 3>>(alpha, beta, kappa);

    Matrix<T, 4, 4> R;
    // clang-format off
    R << 1, 0, 0, 0,
         0, 4, 0, 0,
         0, 0, 9, 0,
         0, 0, 0, 16;
    // clang-format on

    Vector<T, 4> mean;
    mean << 2, 3, 4, 5;

    ukf.sigmaWeights_c << 3, 5, 5, 5, 5, 5, 5;

    Matrix<T, 4, 7> sigmaPoints;
    // clang-format off
    sigmaPoints << 1, 2, 1, 1, 0, 1, 1,
                   2, 3, 3, 2, 1, 1, 2,
                   3, 5, 4, 4, 1, 2, 2,
                   3, 4, 5, 2, 0, 4, 3;
    // clang-format on

    Matrix<T, 4, 4> P;
    ASSERT_TRUE(ukf.computeCovarianceFromSigmaPoints(mean, sigmaPoints, R, P));

    Matrix<T, 4, 4> P_ref;
    // clang-format off
    P_ref << 44, 43, 53, 86,
             43, 57, 63, 91,
             53, 63, 102, 106,
             86, 91, 106, 228;
    // clang-format on

    EXPECT_TRUE(P_ref.isApprox(P, 1e-6));
}

TEST(UnscentedKalmanFilter, computeKalmanGain)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf = UnscentedKalmanFilter<Vector<T, 3>>(alpha, beta, kappa);

    ukf.sigmaWeights_c << 3, 5, 5, 5, 5, 5, 5;

    Vector<T, 2> mean;
    mean << 2, 3;

    typename UnscentedKalmanFilter<Vector<T, 3>>::template SigmaPoints<
        decltype(mean)>
        sigmaPoints;
    sigmaPoints << 1, 2, 1, 1, 0, 1, 1, 2, 3, 3, 2, 1, 1, 2;

    Matrix<T, 2, 2> P_yy;
    P_yy << 35, 34, 34, 46;

    // x and sigmaStatePoints
    ukf.x << 3, 5, 7;
    ukf.sigmaStatePoints << 3, 5, 3, 3, 1, 3, 3, 5, 7, 7, 5, 3, 3, 5, 7, 11, 9,
        9, 3, 5, 5;

    // Reference value for P
    Matrix<T, 3, 2> P_xy_ref;
    // clang-format off
    P_xy_ref << 20, 20, 20,
                40, 40, 60;
    // clang-format on

    // Reference value for K
    Matrix<T, 3, 2> K_ref;
    // clang-format off
    K_ref << 0.528634361233480, 0.044052863436123, -0.969162995594713,
             1.585903083700440, -0.440528634361233, 1.629955947136563;
    // clang-format on

    // Kalman gain to be computed
    Matrix<T, 3, 2> K;
    ukf.computeKalmanGain(mean, sigmaPoints, P_yy, K);

    EXPECT_TRUE(K_ref.isApprox(K, 1e-6));
}

TEST(UnscentedKalmanFilter, updateStateCovariance)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf = UnscentedKalmanFilter<Vector<T, 3>>(alpha, beta, kappa);

    Matrix<T, 2, 2> P_yy;
    P_yy << 1, 0.1, 0.1, 0.5;
    P_yy *= 0.1;

    // Setup Kalman Gain
    Matrix<T, 3, 2> K;
    // clang-format off
    K << 0.178408514951850, -0.304105423213381, -1.110998479472884,
         0.802838317283324, -1.246156445345498, 0.063524243960128;
    // clang-format on

    // Setup P
    ukf.P.setIdentity();

    // Setup P_ref
    Matrix<T, 3, 3> P_ref;
    // clang-format off
    P_ref << 0.993278134696764, 0.027217594648895, 0.019295433443564,
             0.027217594648895, 0.862179812671265, -0.130287401631777,
             0.019295433443564, -0.130287401631777, 0.846090867814784;
    // clang-format on

    // Perform update
    bool success = ukf.updateStateCovariance<Vector<T, 2>>(K, P_yy);
    ASSERT_TRUE(success);

    EXPECT_TRUE(P_ref.isApprox(ukf.P, 1e-6));
}

template <class StateType>
class ConcreteUKF : public UnscentedKalmanFilterBase<StateType>
{
public:
    typedef UnscentedKalmanFilterBase<StateType> Base;
    using typename Base::T;

    ConcreteUKF(T alpha, T beta, T kappa) : Base(alpha, beta, kappa) {}
};

typedef float T;

TEST(UnscentedKalmanFilterBase, init)
{
    auto ukf = ConcreteUKF<Vector<T, 3>>(1, 2, 1);

    // x should be zero
    ASSERT_FLOAT_EQ(0, ukf.x[0]);
    ASSERT_FLOAT_EQ(0, ukf.x[1]);
    ASSERT_FLOAT_EQ(0, ukf.x[2]);
}

TEST(UnscentedKalmanFilterBase, computeSigmaWeights)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf = ConcreteUKF<Vector<T, 3>>(alpha, beta, kappa);
    ukf.computeWeights();

    ASSERT_FLOAT_EQ(2, ukf.gamma);
    ASSERT_FLOAT_EQ(1, ukf.lambda);

    ASSERT_FLOAT_EQ(0.25, ukf.sigmaWeights_m[0]);
    ASSERT_FLOAT_EQ(2.25, ukf.sigmaWeights_c[0]);

    for (size_t i = 1; i < 7; i++) {
        ASSERT_FLOAT_EQ(0.125, ukf.sigmaWeights_c[i]);
        ASSERT_FLOAT_EQ(0.125, ukf.sigmaWeights_m[i]);
    }
}

template <class StateType>
class QuadraticSystemModel : public SystemModel<StateType, StateType>
{
public:
    typedef SystemModel<StateType, StateType> Base;
    using typename Base::State;
    using typename Base::Control;

    State f(const State& x, const Control& u) const
    {
        // return x.^2 + u
        return x.cwiseProduct(x) + u;
    }
};

template <class StateType, class MeasurementType = StateType>
class QuadraticMeasurementModel
    : public MeasurementModel<StateType, MeasurementType>
{
public:
    typedef MeasurementModel<StateType, MeasurementType> Base;
    using typename Base::State;
    using typename Base::Measurement;

    static_assert(
        static_cast<decltype(Dynamic)>(MeasurementType::RowsAtCompileTime) <=
            static_cast<decltype(Dynamic)>(StateType::RowsAtCompileTime),
        "Measurement length must be less than or equal to State length");

    Measurement h(const State& x) const
    {
        // return x.^2
        return x.cwiseProduct(x)
            .template head<Measurement::RowsAtCompileTime>();
    }
};

TEST(UnscentedKalmanFilterBase, computeSigmaPointTransition)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf = ConcreteUKF<Vector<T, 3>>(alpha, beta, kappa);
    auto model = QuadraticSystemModel<Vector<T, 3>>();

    // Init variables
    ukf.sigmaStatePoints << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21;

    // Control vector
    Vector<T, 3> u;
    u << 1, 2, 3;

    // Compute reference result
    Matrix<T, 3, 7> ref =
        (ukf.sigmaStatePoints.cwiseProduct(ukf.sigmaStatePoints).colwise() + u)
            .eval();

    // Compute transition
    ukf.computeSigmaPointTransition(model, u);

    EXPECT_TRUE(ref.isApprox(ukf.sigmaStatePoints, 1e-10));
}

TEST(UnscentedKalmanFilterBase, computeSigmaPointMeasurements)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf = ConcreteUKF<Vector<T, 3>>(alpha, beta, kappa);
    auto model = QuadraticMeasurementModel<Vector<T, 3>, Vector<T, 2>>();

    // Init variables
    ukf.sigmaStatePoints << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21;

    // Compute Reference result
    Matrix<T, 2, 7> tmp = ukf.sigmaStatePoints.template topRows<2>();
    Matrix<T, 2, 7> ref = tmp.cwiseProduct(tmp).eval();

    typename ConcreteUKF<Vector<T, 3>>::template SigmaPoints<Vector<T, 2>>
        points;

    ukf.computeSigmaPointMeasurements(model, points);

    // Compare ref and result
    EXPECT_TRUE(ref.isApprox(points, 1e-10));
}

TEST(UnscentedKalmanFilterBase, computePredictionFromSigmaPoints)
{
    T alpha = 1, beta = 2, kappa = 1;

    auto ukf = ConcreteUKF<Vector<T, 3>>(alpha, beta, kappa);

    // Init variables
    ukf.sigmaWeights_m << 7, 6, 5, 4, 3, 2, 1;

    typename ConcreteUKF<Vector<T, 3>>::SigmaPoints<Vector<T, 4>> points;
    points << 1, 2, 3, 4, 5, 6, 7, 10, 20, 30, 40, 50, 60, 70, 100, 200, 300,
        400, 500, 600, 700, 1000, 2000, 3000, 4000, 5000, 6000, 7000;

    Vector<T, 4> x = ukf.computePredictionFromSigmaPoints<Vector<T, 4>>(points);

    ASSERT_FLOAT_EQ(84, x[0]);
    ASSERT_FLOAT_EQ(840, x[1]);
    ASSERT_FLOAT_EQ(8400, x[2]);
    ASSERT_FLOAT_EQ(84000, x[3]);
}
