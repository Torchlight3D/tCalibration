#pragma once

#include "DistortionInterface.h"
#include "../core/eigen_types.h"

namespace tl {

class NoDistortion
{
public:
    enum
    {
        IntrinsicsDimension = -1
    };
    enum
    {
        DesignVariableDimension = 0
    };

    NoDistortion() {}
    virtual ~NoDistortion() {}

    // parameters
    void setParameters(const Eigen::MatrixXd &params) {}
    void parameters(Eigen::MatrixXd &params) const { params.resize(0, 0); }
    Eigen::Vector2i parameterSize() const { return {0, 0}; }

    void clear() {}

    // point mapping
    template <typename Derived_p>
    void distort(const Eigen::MatrixBase<Derived_p> &normalizedPoint) const;

    template <typename Derived_p, typename Derived_j>
    void distort(const Eigen::MatrixBase<Derived_p> &normalizedPoint,
                 const Eigen::MatrixBase<Derived_j> &distortionJacobian) const;

    template <typename Derived_p>
    void undistort(const Eigen::MatrixBase<Derived_p> &normalizedPoint) const;

    template <typename Derived_p, typename Derived_j>
    void undistort(
        const Eigen::MatrixBase<Derived_p> &y,
        const Eigen::MatrixBase<Derived_j> &undistortionJacobian) const;

    template <typename Derived_p, typename Derived_j>
    void distortParameterJacobian(
        const Eigen::MatrixBase<Derived_p> &point,
        const Eigen::MatrixBase<Derived_j> &distortionJacobian) const;

    // slam backend compatibility
    void update(const double *v) {}

    int minimalDimensions() const { return 0; }
};

template <typename DERIVED_Y>
void NoDistortion::distort(const Eigen::MatrixBase<DERIVED_Y> & /* y */) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED_Y>, 2);
}

template <typename DERIVED_Y, typename DERIVED_JY>
void NoDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & /* y */,
    const Eigen::MatrixBase<DERIVED_JY> &outJyConst) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED_Y>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<DERIVED_JY>, 2, 2);

    auto outJy = const_cast<Eigen::MatrixBase<DERIVED_JY> &>(outJyConst);
    outJy.derived().resize(2, 2);
    outJy.setIdentity();
}

template <typename DERIVED>
void NoDistortion::undistort(const Eigen::MatrixBase<DERIVED> & /* y */) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED>, 2);
}

template <typename DERIVED, typename DERIVED_JY>
void NoDistortion::undistort(
    const Eigen::MatrixBase<DERIVED> & /* y */,
    const Eigen::MatrixBase<DERIVED_JY> &outJyConst) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<DERIVED_JY>, 2, 2);

    auto outJy = const_cast<Eigen::MatrixBase<DERIVED_JY> &>(outJyConst);
    outJy.derived().resize(2, 2);
    outJy.setIdentity();
}

template <typename DERIVED_Y, typename DERIVED_JD>
void NoDistortion::distortParameterJacobian(
    const Eigen::MatrixBase<DERIVED_Y> & /* imageY */,
    const Eigen::MatrixBase<DERIVED_JD> &outJdConst) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<DERIVED_Y>, 2);
    // EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(Eigen::MatrixBase<DERIVED_JD>,
    // 2, 2);

    auto outJd = const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJdConst);
    outJd.derived().resize(2, 0);
    // outJd.setIdentity();
}

} // namespace tl
