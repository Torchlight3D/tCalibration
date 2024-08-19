#pragma once

#include <Eigen/Core>

namespace tl {

// see Förstner et al, PCV, page 521
// Js(x)
template <typename T>
void jacobian_3_vec(const Eigen::Vector3<T>& vec, Eigen::Matrix3<T>& jac)
{
    jac = (Eigen::Matrix3<T>::Identity() -
           (vec * vec.transpose()) / (vec.transpose() * vec)) /
          vec.norm();
}

// have a look at
// W. Foerstner, PCV, Page 370
// and S.Urban, MLPnP paper
template <typename T>
void get_information_for_bearing(const T& variance,
                                 const Eigen::Matrix3<T>& inv_cam_mat,
                                 const Eigen::Matrix3<T>& bearing_jac,
                                 const Eigen::Matrix<T, 3, 2>& bearing_ns,
                                 Eigen::Matrix2<T>& information)
{
    const Eigen::Matrix3<T> Exx =
        Eigen::Vector3<T>(variance, variance, 0).asDiagonal();
    const Eigen::Matrix3<T> proj_Exx =
        inv_cam_mat * Exx * inv_cam_mat.transpose();
    const Eigen::Matrix3<T> Evv =
        bearing_jac * proj_Exx * bearing_jac.transpose();
    const Eigen::Matrix2<T> Ers = bearing_ns.transpose() * Evv * bearing_ns;
    information = Ers.inverse();
}

/**
 * compute the nullspace of a 3-vector efficiently
 * without QR see W.Foerstner PCV, Page 778, eq. A.120)
 *
 * @param vector  Eigen::Matrix<T, 3, 1>
 *
 * @return      nullspace 3x2
 */
template <typename T>
void nullS_3x2_templated(const Eigen::Vector3<T>& vector,
                         Eigen::Matrix<T, 3, 2>& nullspace)
{
    const T x_n = vector(2);
    const Eigen::Vector2<T> x_0(vector(0), vector(1));
    const Eigen::Matrix2<T> I_2 = Eigen::Matrix2<T>::Identity();

    const Eigen::Matrix2<T> outer_prod = x_0 * x_0.transpose();
    if (x_n > T(0)) {
        const Eigen::Matrix2<T> tmp = I_2 - outer_prod / (T(1) + x_n);
        nullspace.row(0) = tmp.row(0);
        nullspace.row(1) = tmp.row(1);
        nullspace.row(2) = -x_0.transpose();
    }
    else {
        const Eigen::Matrix2<T> tmp = I_2 - outer_prod / (T(1) - x_n);
        nullspace.row(0) = tmp.row(0);
        nullspace.row(1) = tmp.row(1);
        nullspace.row(2) = x_0.transpose();
    }
}

/**
 * compute the nullspace of a 4-vector efficiently
 * without QR, see W.Foerstner PCV, Page 778, eq. A.120)
 *
 * @param vector  Eigen::Matrix<T, 4, 1>
 *
 * @return      nullspace 4x3
 */
template <typename T>
void nullS_3x4_templated(const Eigen::Vector4<T>& vector,
                         Eigen::Matrix<T, 4, 3>& nullspace)
{
    const T x_n = vector(3);
    const Eigen::Vector3<T> x_0(vector(0), vector(1), vector(2));
    const Eigen::Matrix3<T> I_3 = Eigen::Matrix3<T>::Identity();

    const Eigen::Matrix3<T> outer_prod = x_0 * x_0.transpose();
    if (x_n > T(0)) {
        const Eigen::Matrix3<T> tmp = I_3 - outer_prod / (T(1) + x_n);
        nullspace.row(0) = tmp.row(0);
        nullspace.row(1) = tmp.row(1);
        nullspace.row(2) = tmp.row(2);
        nullspace.row(3) = -x_0.transpose();
    }
    else {
        const Eigen::Matrix3<T> tmp = I_3 - outer_prod / (T(1) - x_n);
        nullspace.row(0) = tmp.row(0);
        nullspace.row(1) = tmp.row(1);
        nullspace.row(2) = tmp.row(2);
        nullspace.row(3) = x_0.transpose();
    }
}

} // namespace tl
