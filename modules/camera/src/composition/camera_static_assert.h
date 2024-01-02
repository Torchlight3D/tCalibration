#pragma once

#include <string_view>
#include <Eigen/Dense>

namespace thoht {

template <class Derived>
struct EvalOrReference
{
    using Type = typename Eigen::internal::eval<Derived>::type;
};

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows,
          int _MaxCols, int MapOptions, typename StrideType>
struct EvalOrReference<Eigen::Map<
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>,
    MapOptions, StrideType>>
{
    using Type = const Eigen::MatrixBase<Eigen::Map<
        Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>,
        MapOptions, StrideType>> &;
};

template <class DerivedPoint3D, class DerivedPoint2D, class DerivedJ3DPtr,
          class DerivedJparamPtr, int N>
constexpr inline void checkProjectionDerivedTypes()
{
    EIGEN_STATIC_ASSERT(DerivedPoint3D::IsVectorAtCompileTime &&
                            (DerivedPoint3D::SizeAtCompileTime == 3 ||
                             DerivedPoint3D::SizeAtCompileTime == 4),
                        THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedPoint2D, 2);

    if constexpr (!std::is_same_v<DerivedJ3DPtr, std::nullptr_t>) {
        static_assert(std::is_pointer_v<DerivedJ3DPtr>);
        using DerivedJ3D = typename std::remove_pointer<DerivedJ3DPtr>::type;

        EIGEN_STATIC_ASSERT(DerivedJ3D::RowsAtCompileTime == 2 &&
                                int(DerivedJ3D::ColsAtCompileTime) ==
                                    int(DerivedPoint3D::SizeAtCompileTime),
                            THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
    }

    if constexpr (!std::is_same_v<DerivedJparamPtr, std::nullptr_t>) {
        static_assert(std::is_pointer_v<DerivedJparamPtr>);
        using DerivedJparam =
            typename std::remove_pointer<DerivedJparamPtr>::type;
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedJparam, 2, N);
    }
}

/// @brief Helper function to check that 3D points are 3 or 4 dimensional and
/// the Jacobians have the appropriate shape in the unproject function
template <class DerivedPoint2D, class DerivedPoint3D, class DerivedJ2DPtr,
          class DerivedJparamPtr, int N>
constexpr inline void checkUnprojectionDerivedTypes()
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedPoint2D, 2);

    EIGEN_STATIC_ASSERT(DerivedPoint3D::IsVectorAtCompileTime &&
                            (DerivedPoint3D::SizeAtCompileTime == 3 ||
                             DerivedPoint3D::SizeAtCompileTime == 4),
                        THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)

    if constexpr (!std::is_same_v<DerivedJ2DPtr, std::nullptr_t>) {
        static_assert(std::is_pointer_v<DerivedJ2DPtr>);
        using DerivedJ2D = typename std::remove_pointer<DerivedJ2DPtr>::type;
        EIGEN_STATIC_ASSERT(DerivedJ2D::ColsAtCompileTime == 2 &&
                                int(DerivedJ2D::RowsAtCompileTime) ==
                                    int(DerivedPoint3D::SizeAtCompileTime),
                            THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
    }

    if constexpr (!std::is_same_v<DerivedJparamPtr, std::nullptr_t>) {
        static_assert(std::is_pointer_v<DerivedJparamPtr>);
        using DerivedJparam =
            typename std::remove_pointer<DerivedJparamPtr>::type;
        EIGEN_STATIC_ASSERT(DerivedJparam::ColsAtCompileTime == N &&
                                int(DerivedJparam::RowsAtCompileTime) ==
                                    int(DerivedPoint3D::SizeAtCompileTime),
                            THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
    }
}

} // namespace thoht
