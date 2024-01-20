#pragma once

#include <Eigen/Core>

#include "../../../core/Duration.h"

namespace tl {

class RollingShutter
{
public:
    enum
    {
        DesignVariableDimension = 0
    };

    RollingShutter();
    explicit RollingShutter(double lineDelay);
    ~RollingShutter();

    // properties
    double lineDelay() const { return _lineDelay; };

    template <typename DERIVED_K>
    Duration temporalOffset(const Eigen::MatrixBase<DERIVED_K> &k) const
    {
        return Duration(_lineDelay * k[1]);
    }

    /// The resulting jacobian assumes the output of "temporal offset" is in
    /// seconds.
    template <typename DERIVED_K, typename DERIVED_J>
    void temporalOffsetIntrinsicsJacobian(
        const Eigen::MatrixBase<DERIVED_K> &k,
        const Eigen::MatrixBase<DERIVED_J> &outJ) const
    {
        Eigen::MatrixBase<DERIVED_J> &J =
            const_cast<Eigen::MatrixBase<DERIVED_J> &>(outJ);
        J.resize(1, 1);
        J(0, 0) = k[1];
    }

    // aslam::backend compatibility
    void update(const double *v);
    int minimalDimensions() const;

    void getParameters(Eigen::MatrixXd &P) const;
    void setParameters(const Eigen::MatrixXd &P);
    Eigen::Vector2i parameterSize() const;

private:
    // RS camera time between start of integration of two consecutive lines in
    // seconds
    double _lineDelay;
};

} // namespace tl
