#pragma once

#include <Eigen/Geometry>

namespace tl {

using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Matrix18d = Eigen::Matrix<double, 18, 18>;
using Vector15d = Eigen::Vector<double, 15>;

class IntegrationBase
{
public:
    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0,
                    const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba,
                    const Eigen::Vector3d &_linearized_bg);

    void push_back(double dt, const Eigen::Vector3d &acc,
                   const Eigen::Vector3d &gyr);

    Vector15d evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi,
                       const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai,
                       const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                       const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
                       const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj);

    void propagate(double _dt, const Eigen::Vector3d &_acc_1,
                   const Eigen::Vector3d &_gyr_1);
    void repropagate(const Eigen::Vector3d &_linearized_ba,
                     const Eigen::Vector3d &_linearized_bg);

    void midPointIntegration(
        double _dt, const Eigen::Vector3d &_acc_0,
        const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_acc_1,
        const Eigen::Vector3d &_gyr_1, const Eigen::Vector3d &delta_p,
        const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
        const Eigen::Vector3d &linearized_ba,
        const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
        Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
        Eigen::Vector3d &result_linearized_ba,
        Eigen::Vector3d &result_linearized_bg, bool update_jacobian);

    void eulerIntegration(
        double _dt, const Eigen::Vector3d &_acc_0,
        const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_acc_1,
        const Eigen::Vector3d &_gyr_1, const Eigen::Vector3d &delta_p,
        const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
        const Eigen::Vector3d &linearized_ba,
        const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
        Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
        Eigen::Vector3d &result_linearized_ba,
        Eigen::Vector3d &result_linearized_bg, bool update_jacobian);

public:
    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;

    Matrix15d jacobian, covariance;
    Matrix15d step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Matrix18d _noise;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
};

} // namespace tl
