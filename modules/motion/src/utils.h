#pragma once

#include <glog/logging.h>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

namespace Sophus {

template <typename _Scalar>
using SE3Tangent = SE3<_Scalar>::Tangent;

template <typename _Scalar>
using Sim3Tangent = Sim3<_Scalar>::Tangent;

/// @brief Decoupled version of logmap for SE(3)
///
/// For SE(3) element vector
/// \f[
/// \begin{pmatrix} R & t \\ 0 & 1 \end{pmatrix} \in SE(3),
/// \f]
/// returns \f$ (t, \log(R)) \in \mathbb{R}^6 \f$. Here rotation is not coupled
/// with translation.
///
/// @param[in] SE(3) member
/// @return tangent vector (6x1 vector)
template <typename _Scalar>
inline SE3Tangent<_Scalar> se3_logd(const SE3<_Scalar> &se3)
{
    SE3Tangent<_Scalar> upsilon_omega;
    upsilon_omega.template tail<3>() = se3.so3().log();
    upsilon_omega.template head<3>() = se3.translation();

    return upsilon_omega;
}

/// @brief Decoupled version of expmap for SE(3)
///
/// For tangent vector \f$ (\upsilon, \omega) \in \mathbb{R}^6 \f$ returns
/// \f[
/// \begin{pmatrix} \exp(\omega) & \upsilon \\ 0 & 1 \end{pmatrix} \in SE(3),
/// \f]
/// where \f$ \exp(\omega) \in SO(3) \f$. Here rotation is not coupled with
/// translation.
///
/// @param[in] tangent vector (6x1 vector)
/// @return  SE(3) member
template <typename _Scalar>
inline SE3<_Scalar> se3_expd(const Eigen::Vector<_Scalar, 6> &upsilon_omega)
{
    using Scalar = _Scalar;
    return SE3<Scalar>(SO3<Scalar>::exp(upsilon_omega.template tail<3>()),
                       upsilon_omega.template head<3>());
}

/// @brief Decoupled version of logmap for Sim(3)
///
/// For Sim(3) element vector
/// \f[
/// \begin{pmatrix} sR & t \\ 0 & 1 \end{pmatrix} \in SE(3),
/// \f]
/// returns \f$ (t, \log(R), log(s)) \in \mathbb{R}^3 \f$. Here rotation and
/// scale are not coupled with translation. Rotation and scale are commutative
/// anyway.
///
/// @param[in] Sim(3) member
/// @return tangent vector (7x1 vector)
template <typename _Scalar>
inline Sim3Tangent<_Scalar> sim3_logd(const Sim3<_Scalar> &sim3)
{
    Sim3Tangent<_Scalar> upsilon_omega_sigma;
    upsilon_omega_sigma.template tail<4>() = sim3.rxso3().log();
    upsilon_omega_sigma.template head<3>() = sim3.translation();

    return upsilon_omega_sigma;
}

/// @brief Decoupled version of expmap for Sim(3)
///
/// For tangent vector \f$ (\upsilon, \omega, \sigma) \in \mathbb{R}^7 \f$
/// returns
/// \f[
/// \begin{pmatrix} \exp(\sigma)\exp(\omega) & \upsilon \\ 0 & 1 \end{pmatrix}
///                                                              \in Sim(3),
/// \f]
/// where \f$ \exp(\omega) \in SO(3) \f$. Here rotation and scale are not
/// coupled with translation. Rotation and scale are commutative anyway.
///
/// @param[in] tangent vector (7x1 vector)
/// @return  Sim(3) member
template <typename _Scalar>
inline Sim3<_Scalar> sim3_expd(
    const Eigen::Vector<_Scalar, 7> &upsilon_omega_sigma)
{
    using Scalar = _Scalar;
    return Sim3<Scalar>(
        RxSO3<Scalar>::exp(upsilon_omega_sigma.template tail<4>()),
        upsilon_omega_sigma.template head<3>());
}

// Note on the use of const_cast in the following functions: The output
// parameter is only marked 'const' to make the C++ compiler accept a temporary
// expression here. These functions use const_cast it, so constness isn't
// honored here. See:
// https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html

/// @brief Right Jacobian for SO(3)
///
/// For \f$ \exp(x) \in SO(3) \f$ provides a Jacobian that approximates the sum
/// under expmap with a right multiplication of expmap for small \f$ \epsilon
/// \f$.  Can be used to compute:  \f$ \exp(\phi + \epsilon) \approx \exp(\phi)
/// \exp(J_{\phi} \epsilon)\f$
/// @param[in] phi (3x1 vector)
/// @param[out] J_phi (3x3 matrix)
template <typename Derived1, typename Derived2>
inline void rightJacobianSO3(const Eigen::MatrixBase<Derived1> &phi,
                             const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    Scalar phi_norm2 = phi.squaredNorm();
    Eigen::Matrix3<Scalar> phi_hat = Sophus::SO3<Scalar>::hat(phi);
    Eigen::Matrix3<Scalar> phi_hat2 = phi_hat * phi_hat;

    J.setIdentity();
    if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
        Scalar phi_norm = std::sqrt(phi_norm2);
        Scalar phi_norm3 = phi_norm2 * phi_norm;

        J -= phi_hat * (1 - std::cos(phi_norm)) / phi_norm2;
        J += phi_hat2 * (phi_norm - std::sin(phi_norm)) / phi_norm3;
    }
    else {
        // Taylor expansion around 0
        J -= phi_hat / 2;
        J += phi_hat2 / 6;
    }
}

/// @brief Right Inverse Jacobian for SO(3)
///
/// For \f$ \exp(x) \in SO(3) \f$ provides an inverse Jacobian that approximates
/// the logmap of the right multiplication of expmap of the arguments with a sum
/// for small \f$ \epsilon \f$.  Can be used to compute:  \f$ \log
/// (\exp(\phi) \exp(\epsilon)) \approx \phi + J_{\phi} \epsilon\f$
/// @param[in] phi (3x1 vector)
/// @param[out] J_phi (3x3 matrix)
template <typename Derived1, typename Derived2>
inline void rightJacobianInvSO3(const Eigen::MatrixBase<Derived1> &phi,
                                const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    Scalar phi_norm2 = phi.squaredNorm();
    Eigen::Matrix3<Scalar> phi_hat = Sophus::SO3<Scalar>::hat(phi);
    Eigen::Matrix3<Scalar> phi_hat2 = phi_hat * phi_hat;

    J.setIdentity();
    J += phi_hat / 2;
    if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
        Scalar phi_norm = std::sqrt(phi_norm2);

        // We require that the angle is in range [0, pi]. We check if we are
        // close to pi and apply a Taylor expansion to scalar multiplier of
        // phi_hat2. Technically, log(exp(phi)exp(epsilon)) is not continuous /
        // differentiable at phi=pi, but we still aim to return a reasonable
        // value for all valid inputs.
        // BASALT_ASSERT(phi_norm <= M_PI +
        // Sophus::Constants<Scalar>::epsilon());

        if (phi_norm < M_PI - Sophus::Constants<Scalar>::epsilonSqrt()) {
            // regular case for range (0,pi)
            J += phi_hat2 *
                 (1 / phi_norm2 - (1 + std::cos(phi_norm)) /
                                      (2 * phi_norm * std::sin(phi_norm)));
        }
        else {
            // 0th-order Taylor expansion around pi
            J += phi_hat2 / (M_PI * M_PI);
        }
    }
    else {
        // Taylor expansion around 0
        J += phi_hat2 / 12;
    }
}

// Alternative version of rightJacobianInvSO3 that normalizes the angle to
// [0,pi]. However, it's too complicated and we decided to instead assert the
// input range, assuming that it is almost never really required (e.g. b/c the
// input is computed from Log()). Regardless, we leave this version here for
// future reference.
// template <typename Derived1, typename Derived2>
// inline void rightJacobianInvSO3(const Eigen::MatrixBase<Derived1> &phi,
//                                 const Eigen::MatrixBase<Derived2> &J_phi) {
//   EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
//   EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
//   EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
//   EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

//   using Scalar = typename Derived1::Scalar;

//   Eigen::MatrixBase<Derived2> &J =
//       const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

//   Scalar phi_norm2 = phi.squaredNorm();

//   if (phi_norm2 < Sophus::Constants<Scalar>::epsilon()) {
//     // short-circuit small angle case: Avoid computing sqrt(phi_norm2).
//     Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
//     Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

//     J.setIdentity();
//     J += phi_hat / 2;

//     // Taylor expansion around 0
//     J += phi_hat2 / 12;
//   } else {
//     // non-small angle case: Compute angle.
//     Scalar phi_norm = std::sqrt(phi_norm2);

//     // Check phi_norm > pi case and compute phi_hat (we assume that we later
//     // don't use phi directly, and thus don't update it)
//     Eigen::Matrix<Scalar, 3, 3> phi_hat;
//     if (phi_norm > M_PI) {
//       // In the definition of the inverse Jacobian we consider the effect of
//       a
//       // perturbation on exp(phi). So here we normalize the angle to [0, 2pi)
//       // and then flip the axis if it is in (pi, 2pi).

//       // we know phi_norm > 0
//       Scalar phi_norm_wrapped = fmod(phi_norm, 2 * M_PI);

//       if (phi_norm_wrapped > M_PI) {
//         // flip axis and invert angle
//         phi_norm_wrapped = 2 * M_PI - phi_norm_wrapped;
//         phi_hat =
//             Sophus::SO3<Scalar>::hat(-phi * (phi_norm_wrapped / phi_norm));
//       } else {
//         // already in [0, pi]
//         phi_hat = Sophus::SO3<Scalar>::hat(phi * (phi_norm_wrapped /
//         phi_norm));
//       }

//       phi_norm = phi_norm_wrapped;
//       phi_norm2 = phi_norm * phi_norm;
//     } else {
//       // regular case: already in (0, pi]
//       phi_hat = Sophus::SO3<Scalar>::hat(phi);
//     }

//     Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

//     J.setIdentity();
//     J += phi_hat / 2;

//     // Angle is now in range [0, pi]. We check if we are close to 0 (in case
//     of
//     // a wrap-around) or pi and apply Taylor expansions to scalar multiplier
//     of
//     // phi_hat2
//     if (phi_norm < Sophus::Constants<Scalar>::epsilon()) {
//       // 1st-order Taylor expansion around 0
//       J += phi_hat2 / 12;
//     } else if (M_PI - phi_norm < Sophus::Constants<Scalar>::epsilon()) {
//       // 0th-order Taylor expansion around pi
//       J += phi_hat2 / (M_PI * M_PI);
//     } else {
//       // regular case for range (0,pi)
//       J += phi_hat2 * (1 / phi_norm2 - (1 + std::cos(phi_norm)) /
//                                            (2 * phi_norm *
//                                            std::sin(phi_norm)));
//     }
//   }
// }

/// @brief Left Jacobian for SO(3)
///
/// For \f$ \exp(x) \in SO(3) \f$ provides a Jacobian that approximates the sum
/// under expmap with a left multiplication of expmap for small \f$ \epsilon
/// \f$.  Can be used to compute:  \f$ \exp(\phi + \epsilon) \approx
/// \exp(J_{\phi} \epsilon) \exp(\phi) \f$
/// @param[in] phi (3x1 vector)
/// @param[out] J_phi (3x3 matrix)
template <typename Derived1, typename Derived2>
inline void leftJacobianSO3(const Eigen::MatrixBase<Derived1> &phi,
                            const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    Scalar phi_norm2 = phi.squaredNorm();
    Eigen::Matrix3<Scalar> phi_hat = Sophus::SO3<Scalar>::hat(phi);
    Eigen::Matrix3<Scalar> phi_hat2 = phi_hat * phi_hat;

    J.setIdentity();
    if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
        Scalar phi_norm = std::sqrt(phi_norm2);
        Scalar phi_norm3 = phi_norm2 * phi_norm;

        J += phi_hat * (1 - std::cos(phi_norm)) / phi_norm2;
        J += phi_hat2 * (phi_norm - std::sin(phi_norm)) / phi_norm3;
    }
    else {
        // Taylor expansion around 0
        J += phi_hat / 2;
        J += phi_hat2 / 6;
    }
}

/// @brief Left Inverse Jacobian for SO(3)
///
/// For \f$ \exp(x) \in SO(3) \f$ provides an inverse Jacobian that approximates
/// the logmap of the left multiplication of expmap of the arguments with a sum
/// for small \f$ \epsilon \f$.  Can be used to compute:  \f$ \log
/// (\exp(\epsilon) \exp(\phi)) \approx \phi + J_{\phi} \epsilon\f$
/// @param[in] phi (3x1 vector)
/// @param[out] J_phi (3x3 matrix)
template <typename Derived1, typename Derived2>
inline void leftJacobianInvSO3(const Eigen::MatrixBase<Derived1> &phi,
                               const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    Scalar phi_norm2 = phi.squaredNorm();
    Eigen::Matrix3<Scalar> phi_hat = Sophus::SO3<Scalar>::hat(phi);
    Eigen::Matrix3<Scalar> phi_hat2 = phi_hat * phi_hat;

    J.setIdentity();
    J -= phi_hat / 2;
    if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
        Scalar phi_norm = std::sqrt(phi_norm2);

        // We require that the angle is in range [0, pi]. We check if we are
        // close to pi and apply a Taylor expansion to scalar multiplier of
        // phi_hat2. Technically, log(exp(phi)exp(epsilon)) is not continuous /
        // differentiable at phi=pi, but we still aim to return a reasonable
        // value for all valid inputs.
        CHECK(phi_norm <= M_PI + Sophus::Constants<Scalar>::epsilon());

        if (phi_norm < M_PI - Sophus::Constants<Scalar>::epsilonSqrt()) {
            // regular case for range (0,pi)
            J += phi_hat2 *
                 (1 / phi_norm2 - (1 + std::cos(phi_norm)) /
                                      (2 * phi_norm * std::sin(phi_norm)));
        }
        else {
            // 0th-order Taylor expansion around pi
            J += phi_hat2 / (M_PI * M_PI);
        }
    }
    else {
        // Taylor expansion around 0
        J += phi_hat2 / 12;
    }
}

/// @brief Right Jacobian for decoupled SE(3)
///
/// For \f$ \exp(x) \in SE(3) \f$ provides a Jacobian that approximates the sum
/// under decoupled expmap with a right multiplication of decoupled expmap for
/// small \f$ \epsilon \f$.  Can be used to compute:  \f$ \exp(\phi + \epsilon)
/// \approx \exp(\phi) \exp(J_{\phi} \epsilon)\f$
/// @param[in] phi (6x1 vector)
/// @param[out] J_phi (6x6 matrix)
template <typename Derived1, typename Derived2>
inline void rightJacobianSE3Decoupled(const Eigen::MatrixBase<Derived1> &phi,
                                      const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 6, 6);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    J.setZero();
    Eigen::Vector3<Scalar> omega = phi.template tail<3>();
    rightJacobianSO3(omega, J.template bottomRightCorner<3, 3>());
    J.template topLeftCorner<3, 3>() =
        Sophus::SO3<Scalar>::exp(omega).inverse().matrix();
}

/// @brief Right Inverse Jacobian for decoupled SE(3)
///
/// For \f$ \exp(x) \in SE(3) \f$ provides an inverse Jacobian that approximates
/// the decoupled logmap of the right multiplication of the decoupled expmap of
/// the arguments with a sum for small \f$ \epsilon \f$.  Can be used to
/// compute:  \f$ \log
/// (\exp(\phi) \exp(\epsilon)) \approx \phi + J_{\phi} \epsilon\f$
/// @param[in] phi (6x1 vector)
/// @param[out] J_phi (6x6 matrix)
template <typename Derived1, typename Derived2>
inline void rightJacobianInvSE3Decoupled(
    const Eigen::MatrixBase<Derived1> &phi,
    const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 6, 6);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    J.setZero();

    Eigen::Vector3<Scalar> omega = phi.template tail<3>();
    rightJacobianInvSO3(omega, J.template bottomRightCorner<3, 3>());
    J.template topLeftCorner<3, 3>() = Sophus::SO3<Scalar>::exp(omega).matrix();
}

/// @brief Right Jacobian for decoupled Sim(3)
///
/// For \f$ \exp(x) \in Sim(3) \f$ provides a Jacobian that approximates the sum
/// under decoupled expmap with a right multiplication of decoupled expmap for
/// small \f$ \epsilon \f$.  Can be used to compute:  \f$ \exp(\phi + \epsilon)
/// \approx \exp(\phi) \exp(J_{\phi} \epsilon)\f$
/// @param[in] phi (7x1 vector)
/// @param[out] J_phi (7x7 matrix)
template <typename Derived1, typename Derived2>
inline void rightJacobianSim3Decoupled(const Eigen::MatrixBase<Derived1> &phi,
                                       const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 7);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 7, 7);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    J.setZero();

    Eigen::Vector4<Scalar> omega = phi.template tail<4>();
    rightJacobianSO3(omega.template head<3>(), J.template block<3, 3>(3, 3));
    J.template topLeftCorner<3, 3>() =
        Sophus::RxSO3<Scalar>::exp(omega).inverse().matrix();
    J(6, 6) = 1;
}

/// @brief Right Inverse Jacobian for decoupled Sim(3)
///
/// For \f$ \exp(x) \in Sim(3) \f$ provides an inverse Jacobian that
/// approximates the decoupled logmap of the right multiplication of the
/// decoupled expmap of the arguments with a sum for small \f$ \epsilon \f$. Can
/// be used to compute:  \f$ \log
/// (\exp(\phi) \exp(\epsilon)) \approx \phi + J_{\phi} \epsilon\f$
/// @param[in] phi (7x1 vector)
/// @param[out] J_phi (7x7 matrix)
template <typename Derived1, typename Derived2>
inline void rightJacobianInvSim3Decoupled(
    const Eigen::MatrixBase<Derived1> &phi,
    const Eigen::MatrixBase<Derived2> &J_phi)
{
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 7);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 7, 7);

    using Scalar = typename Derived1::Scalar;

    Eigen::MatrixBase<Derived2> &J =
        const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

    J.setZero();

    Eigen::Vector4<Scalar> omega = phi.template tail<4>();
    rightJacobianInvSO3(omega.template head<3>(), J.template block<3, 3>(3, 3));
    J.template topLeftCorner<3, 3>() =
        Sophus::RxSO3<Scalar>::exp(omega).matrix();
    J(6, 6) = 1;
}

} // namespace Sophus
