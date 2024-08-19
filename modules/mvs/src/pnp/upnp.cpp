#include "upnp.h"
#include "impl/upnp.hpp"

#include <Eigen/Eigenvalues>
#include <glog/logging.h>

#include <tCore/Math>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

using Matrix3x10d = Eigen::Matrix<double, 3, 10>;
using Matrix8d = Eigen::Matrix<double, 8, 8>;
using Matrix8cd = Eigen::Matrix<std::complex<double>, 8, 8>;
using Matrix10d = Eigen::Matrix<double, 10, 10>;
using Matrix16d = Eigen::Matrix<double, 16, 16>;
using Matrix16cd = Eigen::Matrix<std::complex<double>, 16, 16>;
using Vector10d = Eigen::Matrix<double, 10, 1>;

namespace {

constexpr int kNumMaxRotations = 16;
constexpr int kNumMaxRotationsExploitingSymmetry = 8;
constexpr int kNumMinCorrespondences = 4;

struct InputDatum
{
    const std::vector<Eigen::Vector3d>& ray_origins;
    const std::vector<Eigen::Vector3d>& ray_directions;
    const std::vector<Eigen::Vector3d>& world_points;
};

// Computes the H Matrix (see Eq. (6)) and the outer products of the ray
// directions, since these are used to compute matrix V (Eq. (5)).
inline Eigen::Matrix3d ComputeHMatrixAndRayDirectionsOuterProducts(
    const InputDatum& input_datum, std::vector<Eigen::Matrix3d>* outer_products)
{
    const auto& ray_directions = input_datum.ray_directions;
    CHECK_NOTNULL(outer_products)->reserve(ray_directions.size());
    Matrix3d h_inverse;
    h_inverse.setZero();
    for (const auto& ray : ray_directions) {
        outer_products->emplace_back(ray * ray.transpose());
        h_inverse -= outer_products->back();
    }
    h_inverse += ray_directions.size() * Matrix3d::Identity();
    return h_inverse.inverse();
}

inline Matrix3x10d LeftMultiply(const Eigen::Vector3d& point)
{
    Matrix3x10d phi_mat;
    // Row 0.
    phi_mat(0, 0) = point.x();
    phi_mat(0, 1) = point.x();
    phi_mat(0, 2) = -point.x();
    phi_mat(0, 3) = -point.x();
    phi_mat(0, 4) = 0.0;
    phi_mat(0, 5) = 2 * point.z();
    phi_mat(0, 6) = -2 * point.y();
    phi_mat(0, 7) = 2 * point.y();
    phi_mat(0, 8) = 2 * point.z();
    phi_mat(0, 9) = 0.0;

    // Row 1.
    phi_mat(1, 0) = point.y();
    phi_mat(1, 1) = -point.y();
    phi_mat(1, 2) = point.y();
    phi_mat(1, 3) = -point.y();
    phi_mat(1, 4) = -2.0 * point.z();
    phi_mat(1, 5) = 0.0;
    phi_mat(1, 6) = 2 * point.x();
    phi_mat(1, 7) = 2 * point.x();
    phi_mat(1, 8) = 0.0;
    phi_mat(1, 9) = 2 * point.z();

    // Row 3.
    phi_mat(2, 0) = point.z();
    phi_mat(2, 1) = -point.z();
    phi_mat(2, 2) = -point.z();
    phi_mat(2, 3) = point.z();
    phi_mat(2, 4) = 2.0 * point.y();
    phi_mat(2, 5) = -2.0 * point.x();
    phi_mat(2, 6) = 0.0;
    phi_mat(2, 7) = 0.0;
    phi_mat(2, 8) = 2.0 * point.x();
    phi_mat(2, 9) = 2.0 * point.y();
    return phi_mat;
}

inline std::vector<Eigen::Matrix3d> ComputeHelperMatrices(
    const InputDatum& input_datum,
    const std::vector<Eigen::Matrix3d>& outer_products,
    const Eigen::Matrix3d& h_matrix, Matrix3x10d* g_matrix,
    Eigen::Vector3d* j_matrix)
{
    const auto& world_points = input_datum.world_points;
    const auto& ray_origins = input_datum.ray_origins;
    CHECK_EQ(ray_origins.size(), outer_products.size());
    CHECK_NOTNULL(g_matrix)->setZero();
    CHECK_NOTNULL(j_matrix)->setZero();
    const Matrix3d identity = Matrix3d::Identity();
    std::vector<Matrix3d> v_matrices(world_points.size(), Matrix3d::Zero());
    for (int i = 0; i < ray_origins.size(); ++i) {
        const auto& outer_product = outer_products[i];
        // Computation following Eq. (5).
        v_matrices[i] = h_matrix * (outer_product - identity);
        const auto& v_matrix = v_matrices[i];
        // Compute the left multiplication matrix or Phi matrix in the paper.
        const Matrix3x10d left_multiply_mat = LeftMultiply(world_points[i]);
        *j_matrix += v_matrix * ray_origins[i];
        *g_matrix += v_matrix * left_multiply_mat;
    }
    return v_matrices;
}

// Computes the block matrices that compose the M matrix in Eq. 17. These
// blocks are:
// quadratic_penalty_matrix = \sum A_i^T * A_i,
// linear_penalty_vector = \sum A_i^T * b_i ,
// gamma = \sum b_i^T * b_i.
void ComputeCostParameters(const InputDatum& input_datum,
                           const std::vector<Eigen::Matrix3d>& outer_products,
                           const Matrix3x10d& g_matrix,
                           const Eigen::Vector3d& j_matrix,
                           Upnp::CostParameters* cost_params)
{
    const Matrix3d identity = Matrix3d::Identity();
    const std::vector<Vector3d>& world_points = input_datum.world_points;
    const std::vector<Vector3d>& ray_origins = input_datum.ray_origins;
    Matrix10d& a_matrix = CHECK_NOTNULL(cost_params)->quadratic_penalty_mat;
    Vector10d& b_vector = cost_params->linear_penalty_vector;
    double& gamma = cost_params->gamma;

    // Gamma is the sum of the dot products of b_matrices.
    for (int i = 0; i < world_points.size(); ++i) {
        // Compute the left multiplication matrix or Phi matrix in the paper.
        const Matrix3x10d left_multiply_mat = LeftMultiply(world_points[i]);
        const Matrix3d outer_prod_minus_identity = outer_products[i] - identity;

        // Compute the i-th a_matrix.
        const Matrix3x10d temp_a_mat =
            outer_prod_minus_identity * (left_multiply_mat + g_matrix);
        a_matrix += temp_a_mat.transpose() * temp_a_mat;

        // Compute the i-th b_vector.
        const Vector3d temp_b_mat =
            -outer_prod_minus_identity * (ray_origins[i] + j_matrix);
        b_vector += temp_a_mat.transpose() * temp_b_mat;

        // Compute the i-th gamma.
        gamma += temp_b_mat.squaredNorm();
    }
}

// Constructs the vector s as indicated in Eq. 12.
inline Vector10d ComputeRotationVector(const Eigen::Quaterniond& rotation)
{
    Vector10d rotation_vector;
    // Set the values of the rotation vector.
    rotation_vector[0] = rotation.w() * rotation.w();
    rotation_vector[1] = rotation.x() * rotation.x();
    rotation_vector[2] = rotation.y() * rotation.y();
    rotation_vector[3] = rotation.z() * rotation.z();
    rotation_vector[4] = rotation.w() * rotation.x();
    rotation_vector[5] = rotation.w() * rotation.y();
    rotation_vector[6] = rotation.w() * rotation.z();
    rotation_vector[7] = rotation.x() * rotation.y();
    rotation_vector[8] = rotation.x() * rotation.z();
    rotation_vector[9] = rotation.y() * rotation.z();
    return rotation_vector;
}

Eigen::Vector3d ComputeTranslation(
    const InputDatum& input_datum, const Eigen::Quaterniond& rotation,
    const std::vector<Eigen::Matrix3d>& v_matrices)
{
    Vector3d translation = Vector3d::Zero();
    for (int i = 0; i < input_datum.world_points.size(); ++i) {
        translation += v_matrices[i] * (rotation * input_datum.world_points[i] -
                                        input_datum.ray_origins[i]);
    }
    return translation;
}

std::vector<Eigen::Vector3d> ComputeTranslations(
    const InputDatum& input_datum,
    const std::vector<Eigen::Quaterniond>& rotations,
    const std::vector<Eigen::Matrix3d>& v_matrices)
{
    std::vector<Vector3d> translations(rotations.size());
    const int num_points = input_datum.world_points.size();
    const int num_rotations = rotations.size();
    for (int i = 0; i < num_rotations; ++i) {
        translations[i] =
            ComputeTranslation(input_datum, rotations[i], v_matrices);
    }
    return translations;
}

std::vector<double> ComputeCostsAndRankSolutions(
    const Upnp::CostParameters& cost_params,
    std::vector<Eigen::Quaterniond>* solution_rotations,
    std::vector<Eigen::Vector3d>* solution_translations)
{
    std::vector<double> costs(solution_rotations->size(), 0.0);
    std::vector<std::pair<int, double>> indexes_and_costs;
    indexes_and_costs.reserve(costs.size());
    // 1. Compute the costs of the solutions.
    for (int i = 0; i < costs.size(); ++i) {
        costs[i] = Upnp::EvaluateCost(cost_params, solution_rotations->at(i));
        indexes_and_costs.emplace_back(i, costs[i]);
    }

    // 2. Sort the costs such that the best rotation (i.e., lowest error) is the
    // first solution.
    std::sort(indexes_and_costs.begin(), indexes_and_costs.end(),
              [](const std::pair<int, double>& lhs,
                 const std::pair<int, double>& rhs) {
                  return lhs.second < rhs.second;
              });

    // 3. Rank the solutions.
    std::vector<Quaterniond> ranked_rotations(costs.size());
    std::vector<Vector3d> ranked_translations(costs.size());
    for (int i = 0; i < costs.size(); ++i) {
        costs[i] = indexes_and_costs[i].second;
        ranked_rotations[i] =
            solution_rotations->at(indexes_and_costs[i].first);
        ranked_translations[i] =
            solution_translations->at(indexes_and_costs[i].first);
    }

    *solution_rotations = std::move(ranked_rotations);
    *solution_translations = std::move(ranked_translations);

    return costs;
}

void DiscardBadSolutions(const InputDatum& input_datum,
                         std::vector<Eigen::Quaterniond>* solution_rotations,
                         std::vector<Eigen::Vector3d>* solution_translations)
{
    CHECK_EQ(CHECK_NOTNULL(solution_rotations)->size(),
             CHECK_NOTNULL(solution_translations)->size());
    std::vector<Quaterniond> final_rotations;
    std::vector<Vector3d> final_translations;
    final_rotations.reserve(solution_rotations->size());
    final_translations.reserve(solution_translations->size());

    /// Alias
    const auto& world_points = input_datum.world_points;
    const auto& ray_origins = input_datum.ray_origins;
    const auto& ray_directions = input_datum.ray_directions;

    // For every computed solution, check that points are in front of camera.
    for (int i = 0; i < solution_rotations->size(); ++i) {
        const auto& soln_rotation = solution_rotations->at(i);
        const auto& soln_translation = solution_translations->at(i);

        // Check that all points are in front of the camera. Discard the
        // solution if this is not the case.
        bool all_points_in_front_of_camera = true;

        for (int j = 0; j < world_points.size(); ++j) {
            const Vector3d transformed_point = soln_rotation * world_points[j] +
                                               soln_translation -
                                               ray_origins[j];

            // Find the rotation that puts the image ray at [0, 0, 1] i.e.
            // looking straightforward from the camera.
            const Quaterniond unrot = Quaterniond::FromTwoVectors(
                ray_directions[j], Vector3d::UnitZ());

            // Rotate the transformed point and check if the z coordinate is
            // negative. This will indicate if the point is projected behind the
            // camera.
            const Vector3d rotated_projection = unrot * transformed_point;
            if (rotated_projection.z() < 0) {
                all_points_in_front_of_camera = false;
                break;
            }
        }

        if (all_points_in_front_of_camera) {
            final_rotations.emplace_back(soln_rotation);
            final_translations.emplace_back(soln_translation);
        }
    }

    // Set the final solutions.
    std::swap(*solution_rotations, final_rotations);
    std::swap(*solution_translations, final_translations);
}

// The observed pattern is that duplicate rotations appear consequtively in the
// vector, i.e., rotation[i] == rotation[i + 1] is common.
std::vector<Eigen::Quaterniond> RemoveDuplicateRotations(
    const std::vector<Eigen::Quaterniond>& candidate_rotations)
{
    // If no rotations then return empty vector.
    if (candidate_rotations.empty()) {
        return {};
    }

    constexpr double kAngleThreshold = math::degToRad(0.1);

    std::vector<Quaterniond> rotations;
    rotations.reserve(candidate_rotations.size());
    for (int i = 0; i < candidate_rotations.size(); ++i) {
        bool duplicate_rotation = false;
        const auto& candidate_rotation = candidate_rotations[i];
        for (int j = rotations.size() - 1; j >= 0; --j) {
            if (candidate_rotation.angularDistance(rotations[j]) <
                kAngleThreshold) {
                duplicate_rotation = true;
                break;
            }
        }
        if (!duplicate_rotation) {
            rotations.push_back(candidate_rotation);
        }
    }
    return rotations;
}

} // namespace

inline std::vector<Eigen::Quaterniond> Upnp::ComputeRotations(
    const int num_correspondences)
{
    // Build the action matrix.
    std::vector<Quaterniond> candidate_rotations;
    if (use_minimal_template_ &&
        num_correspondences <= kNumMinCorrespondences) {
        candidate_rotations = SolveForRotationsFromMinimalSample();
    }
    candidate_rotations = SolveForRotationsFromNonMinimalSample();
    // Remove duplicate solutions.
    return RemoveDuplicateRotations(candidate_rotations);
}

double Upnp::EvaluateCost(const Upnp::CostParameters& parameters,
                          const Eigen::Quaterniond& rotation)
{
    // Compute the quaternion vector.
    const Vector10d rotation_vector = ComputeRotationVector(rotation);
    return (rotation_vector.transpose() * parameters.quadratic_penalty_mat *
                rotation_vector +
            2.0 * parameters.linear_penalty_vector.transpose() *
                rotation_vector)(0, 0) +
           parameters.gamma;
}

double Upnp::ComputeResidual(const Eigen::Vector3d& ray_origin,
                             const Eigen::Vector3d& ray_direction,
                             const Eigen::Vector3d& world_point,
                             const Eigen::Quaterniond& rotation,
                             const Eigen::Vector3d& translation)
{
    const Quaterniond unrot =
        Quaterniond::FromTwoVectors(ray_direction, Vector3d::UnitZ());
    const Vector3d reprojected_point =
        rotation * world_point + translation - ray_origin;
    const Vector3d unrot_reprojected_point = unrot * reprojected_point;
    const Vector3d unrot_ray_direction = unrot * ray_direction;
    return (unrot_reprojected_point.hnormalized() -
            unrot_ray_direction.hnormalized())
        .norm();
}

std::vector<Eigen::Matrix3d> Upnp::ComputeCostParameters(
    const std::vector<Eigen::Vector3d>& ray_origins,
    const std::vector<Eigen::Vector3d>& ray_directions,
    const std::vector<Eigen::Vector3d>& world_points)
{
    const InputDatum input_datum(ray_origins, ray_directions, world_points);
    // 1. Compute the H matrix and the outer products of the ray directions.
    std::vector<Matrix3d> outer_products;
    const Matrix3d h_matrix = ComputeHMatrixAndRayDirectionsOuterProducts(
        input_datum, &outer_products);

    // 2. Compute matrices J and G from page 132 or 6-th page in the paper.
    Matrix3x10d g_matrix;
    Vector3d j_matrix;
    const std::vector<Matrix3d> v_matrices = ComputeHelperMatrices(
        input_datum, outer_products, h_matrix, &g_matrix, &j_matrix);

    // 3. Compute matrix the block-matrix of matrix M from Eq. 17.
    ::tl::ComputeCostParameters(input_datum, outer_products, g_matrix, j_matrix,
                                &cost_params_);

    return v_matrices;
}

std::vector<Eigen::Quaterniond> Upnp::SolveForRotationsFromNonMinimalSample()
{
    std::vector<Quaterniond> rotations(kNumMaxRotationsExploitingSymmetry);
    // Build action matrix.
    const Matrix8d action_matrix = BuildActionMatrixUsingSymmetry(
        cost_params_.quadratic_penalty_mat, cost_params_.linear_penalty_vector,
        &non_minimal_sample_template_matrix_);

    const Eigen::EigenSolver<Matrix8d> eigen_solver(action_matrix);
    const Matrix8cd eigen_vectors = eigen_solver.eigenvectors();

    for (int i = 0; i < rotations.size(); ++i) {
        // According to the original implementation, the complex solutions
        // can be good, in particular when the number of correspondences is
        // really low. The solutions simply ignore the imaginary part.
        rotations[i] =
            Quaterniond(eigen_vectors(4, i).real(), eigen_vectors(5, i).real(),
                        eigen_vectors(6, i).real(), eigen_vectors(7, i).real())
                .normalized();
    }

    return rotations;
}

std::vector<Eigen::Quaterniond> Upnp::SolveForRotationsFromMinimalSample()
{
    std::vector<Quaterniond> rotations(kNumMaxRotations);
    // Build action matrix.
    const Matrix16d action_matrix = BuildActionMatrix(
        cost_params_.quadratic_penalty_mat, cost_params_.linear_penalty_vector,
        &minimal_sample_template_matrix_);

    const Eigen::EigenSolver<Matrix16d> eigen_solver(action_matrix, true);
    const Matrix16cd eigen_vectors = eigen_solver.eigenvectors();

    for (int i = 0; i < rotations.size(); ++i) {
        // According to the original implementation, the complex solutions
        // can be good, in particular when the number of correspondences is
        // really low. The solutions simply ignore the imaginary part.
        Vector4d quaternion(
            eigen_vectors(11, i).real(), eigen_vectors(12, i).real(),
            eigen_vectors(13, i).real(), eigen_vectors(14, i).real());

        if (quaternion[0] < 0.0) {
            quaternion *= -1.0;
        }

        rotations[i] = Eigen::Quaterniond(quaternion[0], quaternion[1],
                                          quaternion[2], quaternion[3])
                           .normalized();
    }

    return rotations;
}

bool Upnp::EstimatePose(const std::vector<Eigen::Vector3d>& ray_origins,
                        const std::vector<Eigen::Vector3d>& ray_directions,
                        const std::vector<Eigen::Vector3d>& world_points,
                        std::vector<Eigen::Quaterniond>* solution_rotations,
                        std::vector<Eigen::Vector3d>* solution_translations,
                        std::vector<double>* solution_costs)
{
    CHECK_NOTNULL(solution_rotations)->clear();
    CHECK_NOTNULL(solution_translations)->clear();
    CHECK_EQ(ray_origins.size(), ray_directions.size());
    CHECK_EQ(world_points.size(), ray_directions.size());

    // Compute Upnp cost parameters.
    const InputDatum input_datum(ray_origins, ray_directions, world_points);
    const std::vector<Matrix3d> v_matrices =
        ComputeCostParameters(ray_origins, ray_directions, world_points);

    // Compute rotations.
    *solution_rotations = ComputeRotations(world_points.size());

    // Compute translation.
    *solution_translations =
        ComputeTranslations(input_datum, *solution_rotations, v_matrices);

    // Discard solutions that have points behind the camera.
    DiscardBadSolutions(input_datum, solution_rotations, solution_translations);

    if (solution_costs) {
        *solution_costs = ComputeCostsAndRankSolutions(
            cost_params_, solution_rotations, solution_translations);
    }

    return !solution_rotations->empty();
}

Upnp::CostParameters Upnp(const std::vector<Eigen::Vector3d>& ray_origins,
                          const std::vector<Eigen::Vector3d>& ray_directions,
                          const std::vector<Eigen::Vector3d>& world_points,
                          std::vector<Eigen::Quaterniond>* solution_rotations,
                          std::vector<Eigen::Vector3d>* solution_translations)
{
    class Upnp estimator;
    CHECK(estimator.EstimatePose(ray_origins, ray_directions, world_points,
                                 solution_rotations, solution_translations))
        << "Could not estimate pose";
    return estimator.cost_params();
}

Upnp::CostParameters Upnp(const std::vector<Eigen::Vector2d>& normalized_pixels,
                          const std::vector<Eigen::Vector3d>& world_points,
                          std::vector<Eigen::Quaterniond>* solution_rotations,
                          std::vector<Eigen::Vector3d>* solution_translations)
{
    CHECK_EQ(normalized_pixels.size(), world_points.size());
    std::vector<Vector3d> ray_directions(world_points.size());
    std::vector<Vector3d> ray_origins(world_points.size());

    // Compute the ray directions and origins from the normalized pixels.
    for (int i = 0; i < world_points.size(); ++i) {
        ray_directions[i] = normalized_pixels[i].homogeneous().normalized();
        ray_origins[i].setZero();
    }

    return Upnp(ray_origins, ray_directions, world_points, solution_rotations,
                solution_translations);
}

} // namespace tl
