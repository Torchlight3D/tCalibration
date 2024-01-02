#pragma once

#include <vector>
#include <Eigen/Core>

namespace thoht {

using Matrix24d = Eigen::Matrix<double, 2, 4>;

// Helper method to the P4Pf algorithm that computes the grobner basis that
// allows for a solution to the problem.
bool FourPointFocalLengthHelper(double glab, double glac, double glad,
                                double glbc, double glbd, double glcd,
                                const Matrix24d& features_normalized,
                                std::vector<double>* f,
                                std::vector<Eigen::Vector3d>* depths);

} // namespace thoht
