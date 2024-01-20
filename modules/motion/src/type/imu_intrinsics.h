#pragma once

#include <Eigen/Core>

namespace tl {

namespace imu {
inline static constexpr int kMisalignmentSize{6};
inline static constexpr int kScaleSize{3};
inline static constexpr int kBiasSize{3};
inline static constexpr int kParameterSize =
    kMisalignmentSize + kScaleSize + kBiasSize;
} // namespace imu

template <typename T>
class ImuIntrinsics_
{
public:
    ImuIntrinsics_(T mis_yz = T(0), T mis_zy = T(0), T mis_zx = T(0),
                   T mis_xz = T(0), T mis_xy = T(0), T mis_yx = T(0),
                   T s_x = T(1), T s_y = T(1), T s_z = T(1), T b_x = T(0),
                   T b_y = T(0), T b_z = T(0))
    {
        // clang-format off
        T_ <<    T(1), -mis_yz,  mis_zy,
               mis_xz,    T(1), -mis_zx,
              -mis_xy,  mis_yx,    T(1);

        K_ <<   s_x, T(0), T(0),
               T(0),  s_y, T(0),
               T(0), T(0),  s_z;
        // clang-format on

        bias_ << b_x, b_y, b_z;

        update();
    }

    inline T misYZ() const { return -T_(0, 1); }
    inline T misZY() const { return T_(0, 2); }
    inline T misZX() const { return -T_(1, 2); }
    inline T misXZ() const { return T_(1, 0); }
    inline T misXY() const { return -T_(2, 0); }
    inline T misYX() const { return T_(2, 1); }

    inline T scaleX() const { return K_(0, 0); }
    inline T scaleY() const { return K_(1, 1); }
    inline T scaleZ() const { return K_(2, 2); }

    inline T biasX() const { return bias_(0); }
    inline T biasY() const { return bias_(1); }
    inline T biasZ() const { return bias_(2); }

    inline void setMisalignmentMatrix(const Eigen::Matrix3<T>& mat)
    {
        T_ = mat;
        update();
    }
    inline const Eigen::Matrix3<T>& misalignmentMatrix() const { return T_; }

    inline void setScale(const Eigen::Vector3<T>& scale)
    {
        K_(0, 0) = scale(0);
        K_(1, 1) = scale(1);
        K_(2, 2) = scale(2);
        update();
    }
    inline const Eigen::Matrix3<T>& scaleMatrix() const { return K_; }

    inline void setBias(const Eigen::Vector3<T>& bias)
    {
        bias_ = bias;
        update();
    }
    inline const Eigen::Vector3<T>& bias() const { return bias_; }

    inline std::vector<T> toVector() const
    {
        return {misYZ(),  misZY(),  misZX(),  misXZ(),  misXY(),  misYX(),
                scaleX(), scaleY(), scaleZ(), bias_(0), bias_(1), bias_(2)};
    }

    inline Eigen::Matrix3<T> toCompactMatrix() const
    {
        return T_ - Eigen::Matrix3<T>::Identity() + K_;
    }

    inline Eigen::Vector3<T> normalize(const Eigen::Vector3<T>& data) const
    {
        return TK_ * data;
    }

    inline Eigen::Vector3<T> unbias(const Eigen::Vector3<T>& data) const
    {
        return data - bias_;
    }

    inline Eigen::Vector3<T> unbiasNormalize(
        const Eigen::Vector3<T>& data) const
    {
        return TK_ * (data - bias_);
    }

private:
    inline void update() { TK_ = T_ * K_; }

private:
    Eigen::Matrix3<T> T_;
    Eigen::Matrix3<T> K_;
    Eigen::Vector3<T> bias_;
    Eigen::Matrix3<T> TK_; // cache
};

using ImuIntrinsics = ImuIntrinsics_<double>;

} // namespace tl
