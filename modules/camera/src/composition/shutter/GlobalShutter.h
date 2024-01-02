#pragma once

#include <Eigen/Core>

#include "../../../core/Duration.h"

namespace thoht {

class GlobalShutter
{
public:
    enum
    {
        DesignVariableDimension = 0
    };

    GlobalShutter();
    ~GlobalShutter();

    template <typename K>
    Duration temporalOffset(const K & /* k */) const
    {
        return Duration(0);
    }

    template <typename DERIVED_K, typename DERIVED_J>
    void temporalOffsetIntrinsicsJacobian(
        const Eigen::MatrixBase<DERIVED_K> & /* k */,
        const Eigen::MatrixBase<DERIVED_J> &outJ) const
    {
        auto J = const_cast<Eigen::MatrixBase<DERIVED_J> &>(outJ);
        J.resize(0, 0);
    }

    // slam backend compatibility
    void update(const double *v);
    int minimalDimensions() const;

    void getParameters(Eigen::MatrixXd &P) const;
    void setParameters(const Eigen::MatrixXd &P);
    Eigen::Vector2i parameterSize() const;
};

} // namespace thoht
