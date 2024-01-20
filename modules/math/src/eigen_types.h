#pragma once

#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

// static assertion failing if the type \a TYPE is not a vector type of the
// given size
#define EIGEN_STATIC_ASSERT_VECTOR_SIZE(TYPE, SIZE)                      \
    EIGEN_STATIC_ASSERT(TYPE::IsVectorAtCompileTime &&                   \
                            (TYPE::SizeAtCompileTime == SIZE ||          \
                             TYPE::SizeAtCompileTime == Eigen::Dynamic), \
                        THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)

// static assertion failing if the type \a TYPE is not a vector type of the
// given size
#define EIGEN_STATIC_ASSERT_MATRIX_SIZE(TYPE, ROWS, COLS)                \
    EIGEN_STATIC_ASSERT((TYPE::RowsAtCompileTime == ROWS ||              \
                         TYPE::RowsAtCompileTime == Eigen::Dynamic) &&   \
                            (TYPE::ColsAtCompileTime == COLS ||          \
                             TYPE::RowsAtCompileTime == Eigen::Dynamic), \
                        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)

namespace tl {

template <typename Eigen_t>
using eigen_vector = std::vector<Eigen_t, Eigen::aligned_allocator<Eigen_t>>;

template <typename Key, typename Eigen_t>
using eigen_map =
    std::map<Key, Eigen_t, std::less<Key>,
             Eigen::aligned_allocator<std::pair<const Key, Eigen_t>>>;

using Vector2dList = eigen_vector<Eigen::Vector2d>;
using Vector3dList = eigen_vector<Eigen::Vector3d>;
using QuaterniondList = eigen_vector<Eigen::Quaterniond>;
using Matrix3dList = eigen_vector<Eigen::Matrix3d>;
using Matrix4dList = eigen_vector<Eigen::Matrix4d>;

using StampedVector3dList = eigen_map<double, Eigen::Vector3d>;
using StampedQuaterniondList = eigen_map<double, Eigen::Quaterniond>;

using Transform = Eigen::Isometry3d;
using TransformList = eigen_vector<Transform>;

using Matrix34d = Eigen::Matrix<double, 3, 4>;
using Matrix34f = Eigen::Matrix<float, 3, 4>; // for 3D viz
using ProjectionMatrix = Matrix34d;

} // namespace tl
