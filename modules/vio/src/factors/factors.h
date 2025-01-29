#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

template <typename T>
inline void QuaternionInverse(const T q[4], T q_inverse[4])
{
    q_inverse[0] = q[0];
    q_inverse[1] = -q[1];
    q_inverse[2] = -q[2];
    q_inverse[3] = -q[3];
};

struct TError
{
    double t_x, t_y, t_z, var;

    TError(double t_x, double t_y, double t_z, double var)
        : t_x(t_x), t_y(t_y), t_z(t_z), var(var)
    {
    }

    template <typename T>
    bool operator()(const T* tj, T* residuals) const
    {
        using Vec3 = Eigen::Vector3<T>;

        Eigen::Map<Vec3>{residuals} =
            (Eigen::Map<const Vec3>{tj} - Vec3(t_x, t_y, t_z)) / T(var);

        return true;
    }

    static ceres::CostFunction* Create(double t_x, double t_y, double t_z,
                                       double var)
    {
        return new ceres::AutoDiffCostFunction<TError, 3, 3>(
            new TError(t_x, t_y, t_z, var));
    }
};

struct RelativeRTError
{
    double t_x, t_y, t_z, t_norm;
    double q_w, q_x, q_y, q_z;
    double t_var, q_var;

    RelativeRTError(double t_x, double t_y, double t_z, double q_w, double q_x,
                    double q_y, double q_z, double t_var, double q_var)
        : t_x(t_x),
          t_y(t_y),
          t_z(t_z),
          q_w(q_w),
          q_x(q_x),
          q_y(q_y),
          q_z(q_z),
          t_var(t_var),
          q_var(q_var)
    {
    }

    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j,
                    const T* tj, T* residuals) const
    {
        using Quat = Eigen::Quaternion<T>;
        using Vec3 = Eigen::Vector3<T>;

        const Vec3 t_w_ij =
            Eigen::Map<const Vec3>{tj} - Eigen::Map<const Vec3>{ti};

        T i_q_w[4];
        QuaternionInverse(w_q_i, i_q_w);

        Vec3 t_i_ij;
        ceres::QuaternionRotatePoint(i_q_w, t_w_ij.data(), t_i_ij.data());

        Eigen::Map<Vec3>{residuals} = (t_i_ij - Vec3(t_x, t_y, t_z)) / T(t_var);

        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = T(q_x);
        relative_q[2] = T(q_y);
        relative_q[3] = T(q_z);

        T q_i_j[4];
        ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

        T relative_q_inv[4];
        QuaternionInverse(relative_q, relative_q_inv);

        T error_q[4];
        ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

        residuals[3] = T(2) * error_q[1] / T(q_var);
        residuals[4] = T(2) * error_q[2] / T(q_var);
        residuals[5] = T(2) * error_q[3] / T(q_var);

        return true;
    }

    static ceres::CostFunction* Create(double t_x, double t_y, double t_z,
                                       double q_w, double q_x, double q_y,
                                       double q_z, double t_var, double q_var)
    {
        return new ceres::AutoDiffCostFunction<RelativeRTError, 6, 4, 3, 4, 3>(
            new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var,
                                q_var));
    }
};
