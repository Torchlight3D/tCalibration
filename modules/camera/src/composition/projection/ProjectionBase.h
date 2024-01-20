#pragma once

#include <Eigen/Core>

namespace tl {

template <typename Distortion_t>
class ProjectionBase
{
public:
    ProjectionBase();
    ProjectionBase(double fu, double fv, double cu, double cv, int ru, int rv);
    explicit ProjectionBase(double fu, double fv, double cu, double cv, int ru,
                            int rv, Distortion_t distortion);
    virtual ~ProjectionBase();

    // properties
    void setDistortion(const Distortion_t& distortion);
    Distortion_t& distortion();
    const Distortion_t& distortion() const;

    virtual int keypointDim() const = 0;
    virtual bool isProjectionInvertible() const = 0;

    // intrinsic
    double fu() const { return _fu; }
    double fv() const { return _fv; }
    double cu() const { return _cu; }
    double cv() const { return _cv; }
    int ru() const { return _ru; }
    int rv() const { return _rv; }

    inline double focalLengthCol() const { return fu(); }
    inline double focalLengthRow() const { return fv(); }
    inline double opticalCenterCol() const { return cu(); }
    inline double opticalCenterRow() const { return cv(); }
    inline int width() const { return ru(); }
    inline int height() const { return rv(); }

    Eigen::Matrix3d cameraMatrix() const;

    virtual void resizeIntrinsics(double scale) = 0;

    // point mapping, TODO

    // serialization, TODO

protected:
    Distortion_t _distortion;
    double _fu, _fv, _cu, _cv;
    int _ru, _rv;
};

template <typename Distortion_t>
inline ProjectionBase<Distortion_t>::ProjectionBase()
    : ProjectionBase(0., 0., 0., 0., 1, 1)
{
}

template <typename Distortion_t>
inline ProjectionBase<Distortion_t>::ProjectionBase(double fu, double fv,
                                                    double cu, double cv,
                                                    int ru, int rv)
    : ProjectionBase(fu, fv, cu, cv, ru, rv, Distortion_t())
{
}

template <typename Distortion_t>
ProjectionBase<Distortion_t>::ProjectionBase(double fu, double fv, double cu,
                                             double cv, int ru, int rv,
                                             Distortion_t distortion)
    : _fu(fu),
      _fv(fv),
      _cu(cu),
      _cv(cv),
      _ru(ru),
      _rv(rv),
      _distortion(distortion)
{
}

template <typename Distortion_t>
ProjectionBase<Distortion_t>::~ProjectionBase()
{
}

template <typename Distortion_t>
inline Eigen::Matrix3d ProjectionBase<Distortion_t>::cameraMatrix() const
{
    Eigen::Matrix3d K;
    K << _fu, 0.0, _cu, 0.0, _fv, _cv, 0.0, 0.0, 1.0;
    return K;
}

} // namespace tl
