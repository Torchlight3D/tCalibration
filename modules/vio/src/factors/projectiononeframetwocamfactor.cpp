#include "projectiononeframetwocamfactor.h"

#include <Eigen/Geometry>

#include <tMath/Eigen/Utils>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

using Matrix23d = Eigen::Matrix<double, 2, 3>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;

Eigen::Matrix2d ProjectionFactor::sqrt_info;

ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &point1,
                                   const Eigen::Vector3d &point2)
    : _point1(point1), _point2(point2)
{
#ifdef UNIT_SPHERE_ERROR
    Vector3d b1, b2;
    Vector3d a = pts_j.normalized();
    Vector3d tmp(0, 0, 1);
    if (a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

bool ProjectionFactor::Evaluate(const double *const *parameters,
                                double *residuals, double **jacobians) const
{
    // Left camera
    // const auto &params_i = parameters[0];
    // Eigen::Map<const Vector3d> t_i{params_i};
    // Eigen::Map<const Quaterniond> q_i{params_i + 3};
    Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                   parameters[0][5]);

    Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
                   parameters[1][5]);

    Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
                    parameters[2][5]);

    double inv_dep_i = parameters[3][0];

    Vector3d pts_camera_i = _point1 / inv_dep_i;
    Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Vector3d pts_w = Qi * pts_imu_i + Pi;
    Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    Eigen::Map<Vector2d> residual(residuals);

#ifdef UNIT_SPHERE_ERROR
    residual = tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - _point2.head<2>();
#endif
    residual = sqrt_info * residual;

    if (jacobians) {
        Matrix3d Ri = Qi.toRotationMatrix();
        Matrix3d Rj = Qj.toRotationMatrix();
        Matrix3d ric = qic.toRotationMatrix();

        Eigen::Matrix<double, 2, 3> reduce(2, 3);
#ifdef UNIT_SPHERE_ERROR
        const auto &x1 = pts_camera_j(0);
        const auto &x2 = pts_camera_j(1);
        const auto &x3 = pts_camera_j(2);
        const auto norm = pts_camera_j.norm();
        const auto norm_3 = std::pow(norm, 3);
        Matrix3d jaco_norm;
        // clang-format off
        jaco_norm << 1. / norm - x1 * x1 / norm_3, -x1 * x2 / norm_3, -x1 * x3 / norm_3,
            -x1 * x2 / norm_3, 1. / norm - x2 * x2 / norm_3, -x2 * x3 / norm_3,
            -x1 * x3 / norm_3, -x2 * x3 / norm_3, 1. / norm - x3 * x3 / norm_3;
        // clang-format on
        reduce = tangent_base * jaco_norm;
#else
        // clang-format off
        reduce << 1. / dep_j, 0., -pts_camera_j(0) / (dep_j * dep_j),
            0., 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
        // clang-format on
#endif
        reduce = sqrt_info * reduce;

        if (jacobians[0]) {
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() =
                ric.transpose() * Rj.transpose() * Ri * -math::Skew(pts_imu_i);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_pose_i(jacobians[0]);
            jaco_pose_i.leftCols<6>() = reduce * jaco_i;
            jaco_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1]) {
            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * math::Skew(pts_imu_j);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_pose_j(jacobians[1]);
            jaco_pose_j.leftCols<6>() = reduce * jaco_j;
            jaco_pose_j.rightCols<1>().setZero();
        }

        if (jacobians[2]) {
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() =
                ric.transpose() * (Rj.transpose() * Ri - Matrix3d::Identity());
            Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() =
                -tmp_r * math::Skew(pts_camera_i) +
                math::Skew(Vector3d(tmp_r * pts_camera_i)) +
                math::Skew(
                    Vector3d(ric.transpose() *
                             (Rj.transpose() * (Ri * tic + Pi - Pj) - tic)));

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_ex_pose(jacobians[2]);
            jaco_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jaco_ex_pose.rightCols<1>().setZero();
        }

        if (jacobians[3]) {
            Eigen::Map<Vector2d> jaco_feature(jacobians[3]);
#if 1
            jaco_feature = reduce * ric.transpose() * Rj.transpose() * Ri *
                           ric * _point1 * -1.0 / (inv_dep_i * inv_dep_i);
#else
            jaco_feature =
                reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
#endif
        }
    }

    return true;
}

namespace {
constexpr auto kRow{640};
constexpr auto TR{320};
} // namespace

Eigen::Matrix2d ProjectionTdFactor::sqrt_info;

ProjectionTdFactor::ProjectionTdFactor(const Eigen::Vector3d &point1,
                                       const Eigen::Vector3d &point2,
                                       const Eigen::Vector2d &velocity1,
                                       const Eigen::Vector2d &velocity2,
                                       double td1, double td2, double row1,
                                       double row2)
    : _point1(point1),
      _point2(point2),
      _velocity1(velocity1.x(), velocity1.y(), 0.),
      _velocity2(velocity2.x(), velocity2.y(), 0.),
      _td1(td1),
      _td2(td2),
      _row1(row1 - kRow / 2),
      _row2(row2 - kRow / 2)
{
#ifdef UNIT_SPHERE_ERROR
    Vector3d b1, b2;
    Vector3d a = pts_j.normalized();
    Vector3d tmp(0, 0, 1);
    if (a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

bool ProjectionTdFactor::Evaluate(const double *const *parameters,
                                  double *residuals, double **jacobians) const
{
    // Retrive data
    Eigen::Map<const Vector3d> Pi{&parameters[0][0]};
    Eigen::Map<const Quaterniond> Qi{&parameters[0][3]};
    Eigen::Map<const Vector3d> Pj{&parameters[1][0]};
    Eigen::Map<const Quaterniond> Qj{&parameters[1][3]};
    Eigen::Map<const Vector3d> tic{&parameters[2][0]};
    Eigen::Map<const Quaterniond> qic{&parameters[2][3]};
    const auto &inv_dep_i = parameters[3][0];
    const auto &td = parameters[4][0];

    Vector3d pts_i_td = _point1 - (td - _td1 + TR / kRow * _row1) * _velocity1;
    Vector3d pts_j_td = _point2 - (td - _td2 + TR / kRow * _row2) * _velocity2;
    Vector3d pts_camera_i = pts_i_td / inv_dep_i;
    Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Vector3d pts_w = Qi * pts_imu_i + Pi;
    Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    Eigen::Map<Vector2d> residual(residuals);
#ifdef UNIT_SPHERE_ERROR
    residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    const auto &dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif
    residual = sqrt_info * residual;

    if (jacobians) {
        Matrix3d Ri = Qi.toRotationMatrix();
        Matrix3d Rj = Qj.toRotationMatrix();
        Matrix3d ric = qic.toRotationMatrix();
        Matrix23d reduce;
#ifdef UNIT_SPHERE_ERROR
        double norm = pts_camera_j.norm();
        Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = pts_camera_j(0);
        x2 = pts_camera_j(1);
        x3 = pts_camera_j(2);
        norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3),
            -x1 * x2 / pow(norm, 3), -x1 * x3 / pow(norm, 3),
            -x1 * x2 / pow(norm, 3), 1.0 / norm - x2 * x2 / pow(norm, 3),
            -x2 * x3 / pow(norm, 3), -x1 * x3 / pow(norm, 3),
            -x2 * x3 / pow(norm, 3), 1.0 / norm - x3 * x3 / pow(norm, 3);
        reduce = tangent_base * norm_jaco;
#else
        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j), 0,
            1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
#endif
        reduce = sqrt_info * reduce;

        if (jacobians[0]) {
            Matrix36d jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() =
                ric.transpose() * Rj.transpose() * Ri * -math::Skew(pts_imu_i);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1]) {
            Matrix36d jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * math::Skew(pts_imu_j);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jacobian_pose_j(jacobians[1]);
            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }

        if (jacobians[2]) {
            Matrix36d jaco_ex;
            jaco_ex.leftCols<3>() =
                ric.transpose() * (Rj.transpose() * Ri - Matrix3d::Identity());
            Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() =
                -tmp_r * math::Skew(pts_camera_i) +
                math::Skew(Vector3d{tmp_r * pts_camera_i}) +
                math::Skew(
                    Vector3d{ric.transpose() *
                             (Rj.transpose() * (Ri * tic + Pi - Pj) - tic)});

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jacobian_ex_pose(jacobians[2]);
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }

        if (jacobians[3]) { // feature
            Eigen::Map<Vector2d>{jacobians[3]} =
                reduce * ric.transpose() * Rj.transpose() * Ri * ric *
                pts_i_td * -1. / (inv_dep_i * inv_dep_i);
        }

        if (jacobians[4]) { // td
            Eigen::Map<Vector2d>{jacobians[4]} =
                reduce * ric.transpose() * Rj.transpose() * Ri * ric *
                    _velocity1 / inv_dep_i * -1. +
                sqrt_info * _velocity2.head(2);
        }
    }

    return true;
}

Eigen::Matrix2d ProjectionOneFrameTwoCamFactor::sqrt_info;

ProjectionOneFrameTwoCamFactor::ProjectionOneFrameTwoCamFactor(
    const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
    const Eigen::Vector2d &velocity1, const Eigen::Vector2d &velocity2,
    double td1, double td2)
    : _point1(point1),
      _point2(point2),
      _velocity1(velocity1.x(), velocity1.y(), 0.),
      _velocity2(velocity2.x(), velocity2.y(), 0.),
      _td1(td1),
      _td2(td2)
{
#ifdef UNIT_SPHERE_ERROR
    Vector3d b1, b2;
    Vector3d a = pts_j.normalized();
    Vector3d tmp(0, 0, 1);
    if (a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

bool ProjectionOneFrameTwoCamFactor::Evaluate(const double *const *parameters,
                                              double *residuals,
                                              double **jacobians) const
{
    Eigen::Map<const Vector3d> t_ic1{parameters[0]};
    Eigen::Map<const Quaterniond> q_ic1{parameters[0] + 3};

    Eigen::Map<const Vector3d> t_ic2{parameters[1]};
    Eigen::Map<const Quaterniond> q_ic2{parameters[1] + 3};

    const double &invdepth1 = parameters[2][0];

    const double &td = parameters[3][0];

    Vector3d pts_i_td = _point1 - (td - _td1) * _velocity1;
    Vector3d pts_j_td = _point2 - (td - _td2) * _velocity2;

    Vector3d pts_camera_i = pts_i_td / invdepth1;
    Vector3d pts_imu_i = q_ic1 * pts_camera_i + t_ic1;
    Vector3d pts_imu_j = pts_imu_i;
    Vector3d pts_camera_j = q_ic2.inverse() * (pts_imu_j - t_ic2);

    Eigen::Map<Vector2d> residual(residuals);
#ifdef UNIT_SPHERE_ERROR
    residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif
    residual = sqrt_info * residual;

    if (jacobians) {
        Matrix3d ric = q_ic1.toRotationMatrix();
        Matrix3d ric2 = q_ic2.toRotationMatrix();

        Matrix23d reduce;
#ifdef UNIT_SPHERE_ERROR
        const auto norm = pts_camera_j.norm();
        const auto norm3 = std::pow(norm, 3);
        const auto x1 = pts_camera_j(0);
        const auto x2 = pts_camera_j(1);
        const auto x3 = pts_camera_j(2);

        Matrix3d norm_jaco;
        // clang-format off
        norm_jaco << 1. / norm - x1 * x1 / norm3, -x1 * x2 / norm3, -x1 * x3 / norm3,
                     -x1 * x2 / norm3, 1. / norm - x2 * x2 / norm3, -x2 * x3 / norm3,
                     -x1 * x3 / norm3, -x2 * x3 / norm3, 1. / norm - x3 * x3 / norm3;
        // clang-format on
        reduce = tangent_base * norm_jaco;
#else
        // clang-format off
        reduce << 1. / dep_j, 0., -pts_camera_j(0) / (dep_j * dep_j),
                  0., 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
        // clang-format on
#endif
        reduce = sqrt_info * reduce;

        if (jacobians[0]) {
            Matrix36d jaco_ex;
            jaco_ex.leftCols<3>() = ric2.transpose();
            jaco_ex.rightCols<3>() =
                ric2.transpose() * ric * -math::Skew(pts_camera_i);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_ex_pose(jacobians[0]);
            jaco_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jaco_ex_pose.rightCols<1>().setZero();
        }

        if (jacobians[1]) {
            Matrix36d jaco_ex;
            jaco_ex.leftCols<3>() = -ric2.transpose();
            jaco_ex.rightCols<3>() = math::Skew(pts_camera_j);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_ex_pose1(jacobians[1]);
            jaco_ex_pose1.leftCols<6>() = reduce * jaco_ex;
            jaco_ex_pose1.rightCols<1>().setZero();
        }

        if (jacobians[2]) { // feature
#if 1
            Eigen::Map<Vector2d>{jacobians[2]} = reduce * ric2.transpose() *
                                                 ric * _point1 * -1. /
                                                 (invdepth1 * invdepth1);
#else
            Eigen::Map<Vector2d>{jacobians[2]} =
                reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
#endif
        }

        if (jacobians[3]) { // td
            Eigen::Map<Vector2d>{jacobians[3]} = reduce * ric2.transpose() *
                                                     ric * _velocity1 /
                                                     invdepth1 * -1.0 +
                                                 sqrt_info * _velocity2.head(2);
        }
    }

    return true;
}

Eigen::Matrix2d ProjectionTwoFrameOneCamFactor::sqrt_info;

ProjectionTwoFrameOneCamFactor::ProjectionTwoFrameOneCamFactor(
    const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
    const Eigen::Vector2d &velocity1, const Eigen::Vector2d &velocity2,
    double td1, double td2)
    : _point1(point1),
      _point2(point2),
      _velocity1(velocity1.x(), velocity1.y(), 0.),
      _velocity2(velocity2.x(), velocity2.y(), 0.),
      _td1(td1),
      _td2(td2)
{
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if (a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

bool ProjectionTwoFrameOneCamFactor::Evaluate(double const *const *parameters,
                                              double *residuals,
                                              double **jacobians) const
{
    // Retrive data
    Eigen::Map<const Vector3d> Pi{&parameters[0][0]};
    Eigen::Map<const Quaterniond> Qi{&parameters[0][3]};
    Eigen::Map<const Vector3d> Pj{&parameters[1][0]};
    Eigen::Map<const Quaterniond> Qj{&parameters[1][3]};
    Eigen::Map<const Vector3d> tic{&parameters[2][0]};
    Eigen::Map<const Quaterniond> qic{&parameters[2][3]};
    const auto &inv_dep_i = parameters[3][0];
    const auto &td = parameters[4][0];

    Vector3d pts_i_td = _point1 - (td - _td1) * _velocity1;
    Vector3d pts_j_td = _point2 - (td - _td2) * _velocity2;
    Vector3d pts_camera_i = pts_i_td / inv_dep_i;
    Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Vector3d pts_w = Qi * pts_imu_i + Pi;
    Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    Eigen::Map<Vector2d> residual(residuals);
#ifdef UNIT_SPHERE_ERROR
    residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    const auto &dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif
    residual = sqrt_info * residual;

    if (jacobians) {
        Matrix3d Ri = Qi.toRotationMatrix();
        Matrix3d Rj = Qj.toRotationMatrix();
        Matrix3d ric = qic.toRotationMatrix();
        Matrix23d reduce;
#ifdef UNIT_SPHERE_ERROR
        double norm = pts_camera_j.norm();
        Eigen::Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = pts_camera_j(0);
        x2 = pts_camera_j(1);
        x3 = pts_camera_j(2);
        norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3),
            -x1 * x2 / pow(norm, 3), -x1 * x3 / pow(norm, 3),
            -x1 * x2 / pow(norm, 3), 1.0 / norm - x2 * x2 / pow(norm, 3),
            -x2 * x3 / pow(norm, 3), -x1 * x3 / pow(norm, 3),
            -x2 * x3 / pow(norm, 3), 1.0 / norm - x3 * x3 / pow(norm, 3);
        reduce = tangent_base * norm_jaco;
#else
        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j), 0,
            1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
#endif
        reduce = sqrt_info * reduce;

        if (jacobians[0]) { // Left pose
            Matrix36d jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() =
                ric.transpose() * Rj.transpose() * Ri * -math::Skew(pts_imu_i);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_pose_i(jacobians[0]);
            jaco_pose_i.leftCols<6>() = reduce * jaco_i;
            jaco_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1]) { // Right pose
            Matrix36d jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * math::Skew(pts_imu_j);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_pose_j(jacobians[1]);
            jaco_pose_j.leftCols<6>() = reduce * jaco_j;
            jaco_pose_j.rightCols<1>().setZero();
        }

        if (jacobians[2]) { // Extrinsic
            Matrix36d jaco_ex;
            jaco_ex.leftCols<3>() =
                ric.transpose() * (Rj.transpose() * Ri - Matrix3d::Identity());
            Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() =
                -tmp_r * math::Skew(pts_camera_i) +
                math::Skew((tmp_r * pts_camera_i).eval()) +
                math::Skew((ric.transpose() *
                            (Rj.transpose() * (Ri * tic + Pi - Pj) - tic))
                               .eval());

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_ex_pose(jacobians[2]);
            jaco_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jaco_ex_pose.rightCols<1>().setZero();
        }

        if (jacobians[3]) { // feature
            Eigen::Map<Vector2d>{jacobians[3]} =
                reduce * ric.transpose() * Rj.transpose() * Ri * ric *
                pts_i_td * -1. / (inv_dep_i * inv_dep_i);
        }

        if (jacobians[4]) { // td
            Eigen::Map<Vector2d>{jacobians[4]} = reduce * ric.transpose() *
                                                     Rj.transpose() * Ri * ric *
                                                     _velocity1 / inv_dep_i * -1. +
                                                 sqrt_info * _velocity2.head(2);
        }
    }

    return true;
}

Eigen::Matrix2d ProjectionTwoFrameTwoCamFactor::sqrt_info;

ProjectionTwoFrameTwoCamFactor::ProjectionTwoFrameTwoCamFactor(
    const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
    const Eigen::Vector2d &velocity1, const Eigen::Vector2d &velocity2,
    double td1, double td2)
    : _point1(point1),
      _point2(point2),
      _velocity1(velocity1.x(), velocity1.y(), 0.),
      _velocity2(velocity2.x(), velocity2.y(), 0.),
      _td1(td1),
      _td2(td2)
{
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if (a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

bool ProjectionTwoFrameTwoCamFactor::Evaluate(const double *const *parameters,
                                              double *residuals,
                                              double **jacobians) const
{
    // Retrive data
    Eigen::Map<const Vector3d> t_c1{&parameters[0][0]};
    Eigen::Map<const Quaterniond> q_c1{&parameters[0][3]};
    Eigen::Map<const Vector3d> t_c2{&parameters[1][0]};
    Eigen::Map<const Quaterniond> q_c2{&parameters[1][3]};
    Eigen::Map<const Vector3d> t_ic1{&parameters[2][0]};
    Eigen::Map<const Quaterniond> q_ic1{&parameters[2][3]};
    Eigen::Map<const Vector3d> t_ic2{&parameters[3][0]};
    Eigen::Map<const Quaterniond> q_ic2{&parameters[3][3]};
    const auto &inv_dep_i = parameters[4][0];
    const auto &td = parameters[5][0];

    Vector3d point_c1_td = _point1 - (td - _td1) * _velocity1;
    Vector3d point_c2_td = _point2 - (td - _td2) * _velocity2;
    Vector3d pts_camera_i = point_c1_td / inv_dep_i;
    Vector3d pts_imu_i = q_ic1 * pts_camera_i + t_ic1;
    Vector3d pts_w = q_c1 * pts_imu_i + t_c1;
    Vector3d pts_imu_j = q_c2.inverse() * (pts_w - t_c2);
    Vector3d pts_camera_j = q_ic2.inverse() * (pts_imu_j - t_ic2);

    Eigen::Map<Vector2d> residual(residuals);
#ifdef UNIT_SPHERE_ERROR
    residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    const auto &dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - point_c2_td.head<2>();
#endif
    residual = sqrt_info * residual;

    if (jacobians) {
        Matrix3d Ri = q_c1.toRotationMatrix();
        Matrix3d Rj = q_c2.toRotationMatrix();
        Matrix3d ric = q_ic1.toRotationMatrix();
        Matrix3d ric2 = q_ic2.toRotationMatrix();
        Matrix23d reduce;
#ifdef UNIT_SPHERE_ERROR
        double norm = pts_camera_j.norm();
        Eigen::Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = pts_camera_j(0);
        x2 = pts_camera_j(1);
        x3 = pts_camera_j(2);
        norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3),
            -x1 * x2 / pow(norm, 3), -x1 * x3 / pow(norm, 3),
            -x1 * x2 / pow(norm, 3), 1.0 / norm - x2 * x2 / pow(norm, 3),
            -x2 * x3 / pow(norm, 3), -x1 * x3 / pow(norm, 3),
            -x2 * x3 / pow(norm, 3), 1.0 / norm - x3 * x3 / pow(norm, 3);
        reduce = tangent_base * norm_jaco;
#else
        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j), 0,
            1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
#endif
        reduce = sqrt_info * reduce;

        if (jacobians[0]) { // Left pose
            Matrix36d jaco_i;
            jaco_i.leftCols<3>() = ric2.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() =
                ric2.transpose() * Rj.transpose() * Ri * -math::Skew(pts_imu_i);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_pose_i(jacobians[0]);
            jaco_pose_i.leftCols<6>() = reduce * jaco_i;
            jaco_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1]) { // Right pose
            Matrix36d jaco_j;
            jaco_j.leftCols<3>() = ric2.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric2.transpose() * math::Skew(pts_imu_j);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_pose_j(jacobians[1]);
            jaco_pose_j.leftCols<6>() = reduce * jaco_j;
            jaco_pose_j.rightCols<1>().setZero();
        }

        if (jacobians[2]) {
            Matrix36d jaco_ex;
            jaco_ex.leftCols<3>() = ric2.transpose() * Rj.transpose() * Ri;
            jaco_ex.rightCols<3>() = ric2.transpose() * Rj.transpose() * Ri *
                                     ric * -math::Skew(pts_camera_i);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jaco_ex_pose(jacobians[2]);
            jaco_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jaco_ex_pose.rightCols<1>().setZero();
        }

        if (jacobians[3]) {
            Matrix36d jaco_ex;
            jaco_ex.leftCols<3>() = -ric2.transpose();
            jaco_ex.rightCols<3>() = math::Skew(pts_camera_j);

            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
                jacobian_ex_pose1(jacobians[3]);
            jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose1.rightCols<1>().setZero();
        }

        if (jacobians[4]) { // feature
#if 1
            Eigen::Map<Vector2d>{jacobians[4]} =
                reduce * ric2.transpose() * Rj.transpose() * Ri * ric *
                point_c1_td * -1. / (inv_dep_i * inv_dep_i);
#else
            Eigen::Map<Vector2d>{jacobians[4]} =
                reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
#endif
        }
        if (jacobians[5]) { // td
            Eigen::Map<Eigen::Vector2d>{jacobians[5]} =
                reduce * ric2.transpose() * Rj.transpose() * Ri * ric *
                    _velocity1 / inv_dep_i * -1. +
                sqrt_info * _velocity2.head(2);
        }
    }

    return true;
}

} // namespace tl
