#include "imufactor.h"

#include <Eigen/Core>
#include <glog/logging.h>

#include <tMath/Eigen/Utils>
#include "imupreintegration.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Vector15d = Eigen::Matrix<double, 15, 1>;

IMUFactor::IMUFactor(IntegrationBase *_pre_integration)
    : pre_integration(_pre_integration)
{
}

bool IMUFactor::Evaluate(double const *const *parameters, double *residuals,
                         double **jacobians) const
{
    Eigen::Map<const Vector3d> Pi{&parameters[0][0]};
    Eigen::Map<const Quaterniond> Qi{&parameters[0][3]};
    Eigen::Map<const Vector3d> Vi{&parameters[1][0]};
    Eigen::Map<const Vector3d> Bai{&parameters[1][3]};
    Eigen::Map<const Vector3d> Bgi{&parameters[1][6]};
    Eigen::Map<const Vector3d> Pj{&parameters[2][0]};
    Eigen::Map<const Quaterniond> Qj{&parameters[2][3]};
    Eigen::Map<const Vector3d> Vj{&parameters[3][0]};
    Eigen::Map<const Vector3d> Baj{&parameters[3][3]};
    Eigen::Map<const Vector3d> Bgj{&parameters[3][6]};

    // Matrix<double, 15, 15> Fd;
    // Matrix<double, 15, 12> Gd;

    // Vector3d pPj =
    //     Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
    // Quaterniond pQj = Qi * delta_q;
    // Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
    // Vector3d pBaj = Bai;
    // Vector3d pBgj = Bgi;

    // Vi + Qi * delta_v - g * sum_dt = Vj;
    // Qi * delta_q = Qj;

    // delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
    // delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
    // delta_q = Qi.inverse() * Qj;

#if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

    auto sqrt_info =
        Eigen::LLT<Matrix15d>{pre_integration->covariance.inverse()}
            .matrixL()
            .transpose();
    // sqrt_info.setIdentity();

    Eigen::Map<Vector15d>{residuals} =
        sqrt_info *
        pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);

    if (jacobians) {
        if (pre_integration->jacobian.maxCoeff() > 1e8 ||
            pre_integration->jacobian.minCoeff() < -1e8) {
            LOG(FATAL) << "numerical unstable in preintegration";
        }

        const auto &sum_dt = pre_integration->sum_dt;
        Matrix3d dp_dba = pre_integration->jacobian.block<3, 3>(O_P, O_BA);
        Matrix3d dp_dbg = pre_integration->jacobian.block<3, 3>(O_P, O_BG);
        Matrix3d dq_dbg = pre_integration->jacobian.block<3, 3>(O_R, O_BG);
        Matrix3d dv_dba = pre_integration->jacobian.block<3, 3>(O_V, O_BA);
        Matrix3d dv_dbg = pre_integration->jacobian.block<3, 3>(O_V, O_BG);

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>
                jacobian_pose_i{jacobians[0]};
            jacobian_pose_i.setZero();
            jacobian_pose_i.block<3, 3>(O_P, O_P) =
                -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(O_P, O_R) =
                math::Skew(Qi.inverse() *
                           (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
            Quaterniond corrected_delta_q =
                pre_integration->delta_q *
                Utility::deltaQ(
                    Vector3d{dq_dbg * (Bgi - pre_integration->linearized_bg)});
            jacobian_pose_i.block<3, 3>(O_R, O_R) =
                -(Utility::Qleft(Qj.inverse() * Qi) *
                  Utility::Qright(corrected_delta_q))
                     .bottomRightCorner<3, 3>();
#endif
            jacobian_pose_i.block<3, 3>(O_V, O_R) =
                math::Skew(Qi.inverse() * (G * sum_dt + Vj - Vi));

            jacobian_pose_i = sqrt_info * jacobian_pose_i;

            if (jacobian_pose_i.maxCoeff() > 1e8 ||
                jacobian_pose_i.minCoeff() < -1e8) {
                LOG(FATAL) << "numerical unstable in preintegration";
            }
        }

        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
                jacobian_speedbias_i(jacobians[1]);
            jacobian_speedbias_i.setZero();
            jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) =
                -Qi.inverse().toRotationMatrix() * sum_dt;
            jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
            jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
            // Quaterniond corrected_delta_q =
            // pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi -
            // pre_integration->linearized_bg));
            // jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
            // -Utility::Qleft(Qj.inverse() * Qi *
            // corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
                -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q)
                     .bottomRightCorner<3, 3>() *
                dq_dbg;
#endif
            jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) =
                -Qi.inverse().toRotationMatrix();
            jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
            jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;
            jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) =
                -Matrix3d::Identity();
            jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) =
                -Matrix3d::Identity();

            jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

            // ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
            // ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
        }

        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>
                jacobian_pose_j(jacobians[2]);
            jacobian_pose_j.setZero();
            jacobian_pose_j.block<3, 3>(O_P, O_P) =
                Qi.inverse().toRotationMatrix();

#if 0
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Matrix3d::Identity();
#else
            Quaterniond corrected_delta_q =
                pre_integration->delta_q *
                Utility::deltaQ(
                    Vector3d{dq_dbg * (Bgi - pre_integration->linearized_bg)});
            jacobian_pose_j.block<3, 3>(O_R, O_R) =
                Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj)
                    .bottomRightCorner<3, 3>();
#endif

            jacobian_pose_j = sqrt_info * jacobian_pose_j;

            // ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
            // ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
        }

        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>
                jacobian_speedbias_j(jacobians[3]);
            jacobian_speedbias_j.setZero();
            jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) =
                Qi.inverse().toRotationMatrix();
            jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) =
                Matrix3d::Identity();
            jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) =
                Matrix3d::Identity();

            jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

            // ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
            // ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
        }
    }

    return true;
}

} // namespace tl
