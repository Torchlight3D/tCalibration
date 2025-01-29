#include "marginalizationfactor.h"

#include <thread>

#include <glog/logging.h>

namespace tl {

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::VectorXd;

void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost->num_residuals());

    std::vector<int> block_sizes = cost->parameter_block_sizes();
    raw_jacobians = new double *[block_sizes.size()];
    jacobians.resize(block_sizes.size());

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
        jacobians[i].resize(cost->num_residuals(), block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();
        // dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
    cost->Evaluate(parameter_blocks.data(), residuals.data(),
                            raw_jacobians);

    // std::vector<int> tmp_idx(block_sizes.size());
    // Eigen::MatrixXd tmp(dim, dim);
    // for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //     int size_i = localSize(block_sizes[i]);
    //     Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //     for (int j = 0, sub_idx = 0; j <
    //     static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j]
    //     == 7 ? 6 : block_sizes[j], j++)
    //     {
    //         int size_j = localSize(block_sizes[j]);
    //         Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //         tmp_idx[j] = sub_idx;
    //         tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) =
    //         jacobian_i.transpose() * jacobian_j;
    //     }
    // }
    // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    // std::cout << saes.eigenvalues() << std::endl;
    // ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);

    if (loss) {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss->Evaluate(sq_norm, rho);
        // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm,
        // rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            jacobians[i] =
                sqrt_rho1_ *
                (jacobians[i] - alpha_sq_norm_ * residuals *
                                    (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo()
{
    for (auto it = parameter_block_data.begin();
         it != parameter_block_data.end(); ++it) {
        delete[] it->second;
    }

    for (int i = 0; i < (int)factors.size(); i++) {
        delete[] factors[i]->raw_jacobians;
        delete factors[i]->cost;
        delete factors[i];
    }
}

void MarginalizationInfo::addResidualBlockInfo(
    ResidualBlockInfo *residual_block_info)
{
    factors.emplace_back(residual_block_info);

    std::vector<double *> &parameter_blocks =
        residual_block_info->parameter_blocks;
    std::vector<int> parameter_block_sizes =
        residual_block_info->cost->parameter_block_sizes();

    for (int i = 0;
         i < static_cast<int>(residual_block_info->parameter_blocks.size());
         i++) {
        double *addr = parameter_blocks[i];
        int size = parameter_block_sizes[i];
        parameter_block_size[reinterpret_cast<long>(addr)] = size;
    }

    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size());
         i++) {
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}

void MarginalizationInfo::preMarginalize()
{
    for (auto it : factors) {
        it->Evaluate();

        std::vector<int> block_sizes =
            it->cost->parameter_block_sizes();
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
            int size = block_sizes[i];
            if (parameter_block_data.find(addr) == parameter_block_data.end()) {
                double *data = new double[size];
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
                parameter_block_data[addr] = data;
            }
        }
    }
}

int MarginalizationInfo::localSize(int size) const
{
    return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const
{
    return size == 6 ? 7 : size;
}

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; // global size
    std::unordered_map<long, int> parameter_block_idx;  // local size
};

void *ThreadsConstructA(void *threadsstruct)
{
    ThreadsStruct *p = ((ThreadsStruct *)threadsstruct);
    for (auto it : p->sub_factors) {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size());
             i++) {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(
                it->parameter_blocks[i])];
            int size_i = p->parameter_block_size[reinterpret_cast<long>(
                it->parameter_blocks[i])];
            if (size_i == 7)
                size_i = 6;
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size());
                 j++) {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(
                    it->parameter_blocks[j])];
                int size_j = p->parameter_block_size[reinterpret_cast<long>(
                    it->parameter_blocks[j])];
                if (size_j == 7)
                    size_j = 6;
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) +=
                        jacobian_i.transpose() * jacobian_j;
                else {
                    p->A.block(idx_i, idx_j, size_i, size_j) +=
                        jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) =
                        p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            p->b.segment(idx_i, size_i) +=
                jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

void MarginalizationInfo::marginalize()
{
    int pos = 0;
    for (auto &[index, size] : parameter_block_idx) {
        size = pos;
        pos += localSize(parameter_block_size[index]);
    }

    m = pos;

    for (const auto &[index, size] : parameter_block_size) {
        if (!parameter_block_idx.contains(index)) {
            parameter_block_idx[index] = pos;
            pos += localSize(size);
        }
    }

    n = pos - m;

    MatrixXd A(pos, pos);
    VectorXd b(pos);
    A.setZero();
    b.setZero();

    // for (auto it : factors) {
    //     for (int i = 0; i < static_cast<int>(it->parameter_blocks.size());
    //          i++) {
    //         int idx_i = parameter_block_idx[reinterpret_cast<long>(
    //             it->parameter_blocks[i])];
    //         int size_i =
    //         localSize(parameter_block_size[reinterpret_cast<long>(
    //             it->parameter_blocks[i])]);
    //         MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
    //         for (int j = i; j <
    //         static_cast<int>(it->parameter_blocks.size());
    //              j++) {
    //             int idx_j = parameter_block_idx[reinterpret_cast<long>(
    //                 it->parameter_blocks[j])];
    //             int size_j =
    //                 localSize(parameter_block_size[reinterpret_cast<long>(
    //                     it->parameter_blocks[j])]);
    //             MatrixXd jacobian_j =
    //             it->jacobians[j].leftCols(size_j); if (i == j)
    //                 A.block(idx_i, idx_j, size_i, size_j) +=
    //                     jacobian_i.transpose() * jacobian_j;
    //             else {
    //                 A.block(idx_i, idx_j, size_i, size_j) +=
    //                     jacobian_i.transpose() * jacobian_j;
    //                 A.block(idx_j, idx_i, size_j, size_i) =
    //                     A.block(idx_i, idx_j, size_i, size_j).transpose();
    //             }
    //         }
    //         b.segment(idx_i, size_i) += jacobian_i.transpose() *
    //         it->residuals;
    //     }
    // }

    // multi thread
    // FIXME: Double check here, weird implementation
    std::thread tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];
    int i = 0;
    for (auto it : factors) {
        threadsstruct[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        threadsstruct[i].A = MatrixXd::Zero(pos, pos);
        threadsstruct[i].b = VectorXd::Zero(pos);
        threadsstruct[i].parameter_block_size = parameter_block_size;
        threadsstruct[i].parameter_block_idx = parameter_block_idx;
        tids[i] = std::thread{ThreadsConstructA, &threadsstruct[i]};
    }
    for (int i = NUM_THREADS - 1; i >= 0; i--) {
        tids[i].join();
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }

    // TODO
    MatrixXd Amm =
        0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
    Eigen::SelfAdjointEigenSolver<MatrixXd> saes(Amm);

    // ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue
    // %f", saes.eigenvalues().minCoeff());

    MatrixXd Amm_inv =
        saes.eigenvectors() *
        VectorXd((saes.eigenvalues().array() > eps)
                     .select(saes.eigenvalues().array().inverse(), 0))
            .asDiagonal() *
        saes.eigenvectors().transpose();

    VectorXd bmm = b.segment(0, m);
    MatrixXd Amr = A.block(0, m, m, n);
    MatrixXd Arm = A.block(m, 0, n, m);
    MatrixXd Arr = A.block(m, m, n, n);
    VectorXd brr = b.segment(m, n);
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;

    Eigen::SelfAdjointEigenSolver<MatrixXd> saes2(A);
    VectorXd S((saes2.eigenvalues().array() > eps)
                   .select(saes2.eigenvalues().array(), 0));
    VectorXd S_inv((saes2.eigenvalues().array() > eps)
                       .select(saes2.eigenvalues().array().inverse(), 0));

    VectorXd S_sqrt = S.cwiseSqrt();
    VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    linearized_jacobians =
        S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals =
        S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
    // std::cout << A << std::endl
    //           << std::endl;
    // std::cout << linearized_jacobians << std::endl;
    // printf("error2: %f %f\n", (linearized_jacobians.transpose() *
    // linearized_jacobians - A).sum(),
    //       (linearized_jacobians.transpose() * linearized_residuals -
    //       b).sum());
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(
    std::unordered_map<long, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const auto &[index, size] : parameter_block_idx) {
        if (size >= m) {
            keep_block_size.push_back(parameter_block_size[index]);
            keep_block_idx.push_back(parameter_block_idx[index]);
            keep_block_data.push_back(parameter_block_data[index]);
            keep_block_addr.push_back(addr_shift[index]);
        }
    }
    sum_block_size = std::accumulate(std::begin(keep_block_size),
                                     std::end(keep_block_size), 0);

    return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(
    MarginalizationInfo *_marginalization_info)
    : marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size) {
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;
    }

    set_num_residuals(marginalization_info->n);
};

bool MarginalizationFactor::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const
{
    // printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(),
    // num_residuals()); for (int i = 0; i <
    // static_cast<int>(keep_block_size.size()); i++)
    //{
    //     //printf("unsigned %x\n", reinterpret_cast<unsigned
    //     long>(parameters[i]));
    //     //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
    // printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    // printf("residual %x\n", reinterpret_cast<long>(residuals));
    // }
    int n = marginalization_info->n;
    int m = marginalization_info->m;
    VectorXd dx(n);
    for (size_t i = 0; i < marginalization_info->keep_block_size.size(); i++) {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;

        Eigen::Map<const VectorXd> x{parameters[i], size};
        Eigen::Map<const VectorXd> x0{marginalization_info->keep_block_data[i],
                                      size};
        if (size != 7) {
            dx.segment(idx, size) = x - x0;
        }
        else {
            const Quaterniond p0{x0(6), x0(3), x0(4), x0(5)};
            const Quaterniond p{x(6), x(3), x(4), x(5)};

            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            dx.segment<3>(idx + 3) =
                2. * Utility::positify(p0.inverse() * p).vec();
            if (!((p0.inverse() * p).w() >= 0)) {
                dx.segment<3>(idx + 3) =
                    2. * -Utility::positify(p0.inverse() * p).vec();
            }
        }
    }

    Eigen::Map<VectorXd>(residuals, n) =
        marginalization_info->linearized_residuals +
        marginalization_info->linearized_jacobians * dx;

    if (jacobians) {
        for (size_t i{0}; i < marginalization_info->keep_block_size.size();
             i++) {
            if (jacobians[i]) {
                const auto size = marginalization_info->keep_block_size[i];
                const auto local_size = marginalization_info->localSize(size);
                const auto idx = marginalization_info->keep_block_idx[i] - m;

                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                         Eigen::RowMajor>>
                    jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local_size) =
                    marginalization_info->linearized_jacobians.middleCols(
                        idx, local_size);
            }
        }
    }
    return true;
}

} // namespace tl
