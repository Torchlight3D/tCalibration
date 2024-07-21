#include "alignrotations.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {

// A cost function whose error is the difference in rotations after the current
// alignemnt is applied. That is,
//         error = unaligned_rotation * rotation_alignment - gt_rotation.
struct RotationAlignmentError
{
    Eigen::Vector3d _gt_rotation;
    Eigen::Matrix3d _rotation;

    RotationAlignmentError(const Eigen::Vector3d& gt_rotation,
                           const Eigen::Vector3d& rotation)
        : _gt_rotation(gt_rotation)
    {
        ceres::AngleAxisToRotationMatrix(
            rotation.data(), ceres::ColumnMajorAdapter3x3(_rotation.data()));
    }

    // Compute the alignment error of the two rotations after applying the
    // rvec transformation.
    template <typename T>
    bool operator()(const T* rvec, T* residuals) const
    {
        using Mat3 = Eigen::Matrix3<T>;
        using Vec3 = Eigen::Vector3<T>;

        Mat3 R;
        ceres::AngleAxisToRotationMatrix(
            rvec, ceres::ColumnMajorAdapter3x3(R.data()));

        const Mat3 R_aligned = _rotation.cast<T>() * R;

        Vec3 aa_aligned;
        ceres::RotationMatrixToAngleAxis(
            ceres::ColumnMajorAdapter3x3(R_aligned.data()), aa_aligned.data());

        Eigen::Map<Vec3>{residuals} = _gt_rotation.cast<T>() - aa_aligned;
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d& gt_rotation,
                                       const Eigen::Vector3d& rotation)
    {
        return new ceres::AutoDiffCostFunction<RotationAlignmentError,
                                               Vector3d::SizeAtCompileTime,
                                               Vector3d::SizeAtCompileTime>(
            new RotationAlignmentError(gt_rotation, rotation));
    }
};

void rotateRotations(const Eigen::Vector3d& alignment,
                     std::vector<Eigen::Vector3d>* rotations)
{
    Matrix3d R;
    ceres::AngleAxisToRotationMatrix(alignment.data(),
                                     ceres::ColumnMajorAdapter3x3(R.data()));

    for (auto& rotation : *rotations) {
        Matrix3d R_i;
        ceres::AngleAxisToRotationMatrix(
            rotation.data(), ceres::ColumnMajorAdapter3x3(R_i.data()));

        const Matrix3d R_i_align = R_i * R;
        ceres::RotationMatrixToAngleAxis(
            ceres::ColumnMajorAdapter3x3(R_i_align.data()), rotation.data());
    }
}

} // namespace

void AlignRotations(const std::vector<Eigen::Vector3d>& gt_rotations,
                    std::vector<Eigen::Vector3d>* rotations)
{
    CHECK_EQ(gt_rotations.size(), rotations->size());

    Vector3d alignment = Vector3d::Zero();
    ceres::Problem problem;
    for (size_t i{0}; i < gt_rotations.size(); ++i) {
        problem.AddResidualBlock(
            RotationAlignmentError::create(gt_rotations[i], rotations->at(i)),
            nullptr, alignment.data());
    }

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.function_tolerance = 0.;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);

    VLOG(2) << summary.FullReport();

    rotateRotations(alignment, rotations);
}

} // namespace tl
