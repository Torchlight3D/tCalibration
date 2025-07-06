#pragma once

#include <Eigen/Core>

#include <nlohmann/json.hpp>

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

template <typename _Scalar>
class _ImuIntrinsics : public Eigen::Matrix<_Scalar, 3, 4>
{
    using Matrix3 = Eigen::Matrix3<_Scalar>;
    using Vector3 = Eigen::Vector3<_Scalar>;

public:
    using Base = Eigen::Matrix<_Scalar, 3, 4>;
    using Scalar = _Scalar;

    enum
    {
        ScaleSize = 3,
        BiasSize = 3,
    };

    inline _ImuIntrinsics() : Base()
    {
        this->setZero();
        this->diagonal().setOnes();
    }

    inline void setMisalignment(const Eigen::Matrix3<Scalar>& mat)
    {
        const Vector3 scale = this->scale();
        this->block<3, 3>(0, 0) = mat;
        this->scale() = scale;
    }
    inline const auto misalignment() const
    {
        Matrix3 mat = this->template block<3, 3>(0, 0);
        mat.diagonal().setOnes();
        return mat;
    }

    inline const auto scale() const { return this->diagonal(); }
    inline auto scale() { return this->diagonal(); }

    inline const auto bias() const { return this->col(3); }
    inline auto bias() { return this->col(3); }

    inline auto normalize(const Eigen::Vector3<Scalar>& data) const
    {
        return this->template block<3, 3>(0, 0) * data;
    }

    inline auto unbias(const Eigen::Vector3<Scalar>& data) const
    {
        return data - bias();
    }

    inline auto unbiasNormalize(const Eigen::Vector3<Scalar>& data) const
    {
        return normalize(unbias(data));
    }
};

template <typename _Scalar>
std::ostream& operator<<(std::ostream& os, const _ImuIntrinsics<_Scalar>& intri)
{
    os << "Imu intrinsics:"
          "\n"
          "Misalignment: "
          "\n"
       << intri.misalignment()
       << "\n"
          "Scale: "
       << intri.scale().transpose()
       << "\n"
          "Bias: "
       << intri.bias().transpose() << "\n";
    return os;
}

/// @brief Static calibration for accelerometer.
///
/// Calibrates axis scaling and misalignment and has 9 parameters \f$ [b_x,
/// b_y, b_z, s_1, s_2, s_3, s_4, s_5, s_6]^T \f$.
/// \f[
/// a_c = \begin{bmatrix} s_1 + 1 & 0 & 0 \\ s_2 & s_4 + 1 & 0 \\ s_3 & s_5 &
/// s_6 + 1 \\  \end{bmatrix} a_r -  \begin{bmatrix} b_x \\ b_y \\ b_z
/// \end{bmatrix}
/// \f] where  \f$ a_c \f$ is a calibrated measurement and \f$ a_r \f$ is a
/// raw measurement. When all elements are zero applying calibration results in
/// Identity mapping.
template <typename Scalar>
class CalibAccelBias
{
public:
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat33 = Eigen::Matrix<Scalar, 3, 3>;

    /// @brief Default constructor with zero initialization.
    inline CalibAccelBias() { accel_bias_full_.setZero(); }

    /// @brief  Set calibration to random values (used in unit-tests).
    inline void setRandom()
    {
        accel_bias_full_.setRandom();
        accel_bias_full_.template head<3>() /= 10;
        accel_bias_full_.template tail<6>() /= 100;
    }

    /// @brief Return const vector of parameters.
    /// See detailed description in \ref CalibAccelBias.
    inline const Eigen::Matrix<Scalar, 9, 1>& getParam() const
    {
        return accel_bias_full_;
    }

    /// @brief Return vector of parameters. See detailed description in \ref
    /// CalibAccelBias.
    inline Eigen::Matrix<Scalar, 9, 1>& getParam() { return accel_bias_full_; }

    /// @brief Increment the calibration vector
    ///
    /// @param inc increment vector
    inline void operator+=(const Eigen::Matrix<Scalar, 9, 1>& inc)
    {
        accel_bias_full_ += inc;
    }

    /// @brief Return bias vector and scale matrix. See detailed description in
    /// \ref CalibAccelBias.
    inline void getBiasAndScale(Vec3& accel_bias, Mat33& accel_scale) const
    {
        accel_bias = accel_bias_full_.template head<3>();

        accel_scale.setZero();
        accel_scale.col(0) = accel_bias_full_.template segment<3>(3);
        accel_scale(1, 1) = accel_bias_full_(6);
        accel_scale(2, 1) = accel_bias_full_(7);
        accel_scale(2, 2) = accel_bias_full_(8);
    }

    /// @brief Calibrate the measurement. See detailed description in
    /// \ref CalibAccelBias.
    ///
    /// @param raw_measurement
    /// @return calibrated measurement
    inline Vec3 getCalibrated(const Vec3& raw_measurement) const
    {
        Vec3 accel_bias;
        Mat33 accel_scale;

        getBiasAndScale(accel_bias, accel_scale);

        return (raw_measurement + accel_scale * raw_measurement - accel_bias);
    }

    /// @brief Invert calibration (used in unit-tests).
    ///
    /// @param calibrated_measurement
    /// @return raw measurement
    inline Vec3 invertCalibration(const Vec3& calibrated_measurement) const
    {
        Vec3 accel_bias;
        Mat33 accel_scale;

        getBiasAndScale(accel_bias, accel_scale);

        Mat33 accel_scale_inv =
            (Eigen::Matrix3d::Identity() + accel_scale).inverse();

        return accel_scale_inv * (calibrated_measurement + accel_bias);
    }

private:
    Eigen::Matrix<Scalar, 9, 1> accel_bias_full_;
};

/// @brief Static calibration for gyroscope.
///
/// Calibrates rotation, axis scaling and misalignment and has 12 parameters \f$
/// [b_x, b_y, b_z, s_1, s_2, s_3, s_4, s_5, s_6, s_7, s_8, s_9]^T \f$. \f[
/// \omega_c = \begin{bmatrix} s_1 + 1 & s_4 & s_7 \\ s_2 & s_5 + 1 & s_8 \\ s_3
/// & s_6 & s_9 +1 \\  \end{bmatrix} \omega_r -  \begin{bmatrix} b_x \\ b_y
/// \\ b_z \end{bmatrix} \f] where  \f$ \omega_c \f$ is a calibrated measurement
/// and \f$ \omega_r \f$ is a raw measurement. When all elements are zero
/// applying calibration results in Identity mapping.
template <typename Scalar>
class CalibGyroBias
{
public:
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat33 = Eigen::Matrix<Scalar, 3, 3>;

    /// @brief Default constructor with zero initialization.
    inline CalibGyroBias() { gyro_bias_full_.setZero(); }

    /// @brief  Set calibration to random values (used in unit-tests).
    inline void setRandom()
    {
        gyro_bias_full_.setRandom();
        gyro_bias_full_.template head<3>() /= 10;
        gyro_bias_full_.template tail<9>() /= 100;
    }

    /// @brief Return const vector of parameters.
    /// See detailed description in \ref CalibGyroBias.
    inline const Eigen::Matrix<Scalar, 12, 1>& getParam() const
    {
        return gyro_bias_full_;
    }

    /// @brief Return vector of parameters.
    /// See detailed description in \ref CalibGyroBias.
    inline Eigen::Matrix<Scalar, 12, 1>& getParam() { return gyro_bias_full_; }

    /// @brief Increment the calibration vector
    ///
    /// @param inc increment vector
    inline void operator+=(const Eigen::Matrix<Scalar, 12, 1>& inc)
    {
        gyro_bias_full_ += inc;
    }

    /// @brief Return bias vector and scale matrix. See detailed description in
    /// \ref CalibGyroBias.
    inline void getBiasAndScale(Vec3& gyro_bias, Mat33& gyro_scale) const
    {
        gyro_bias = gyro_bias_full_.template head<3>();
        gyro_scale.col(0) = gyro_bias_full_.template segment<3>(3);
        gyro_scale.col(1) = gyro_bias_full_.template segment<3>(6);
        gyro_scale.col(2) = gyro_bias_full_.template segment<3>(9);
    }

    /// @brief Calibrate the measurement. See detailed description in
    /// \ref CalibGyroBias.
    ///
    /// @param raw_measurement
    /// @return calibrated measurement
    inline Vec3 getCalibrated(const Vec3& raw_measurement) const
    {
        Vec3 gyro_bias;
        Mat33 gyro_scale;

        getBiasAndScale(gyro_bias, gyro_scale);

        return (raw_measurement + gyro_scale * raw_measurement - gyro_bias);
    }

    /// @brief Invert calibration (used in unit-tests).
    ///
    /// @param calibrated_measurement
    /// @return raw measurement
    inline Vec3 invertCalibration(const Vec3& calibrated_measurement) const
    {
        Vec3 gyro_bias;
        Mat33 gyro_scale;

        getBiasAndScale(gyro_bias, gyro_scale);

        Mat33 gyro_scale_inv =
            (Eigen::Matrix3d::Identity() + gyro_scale).inverse();

        return gyro_scale_inv * (calibrated_measurement + gyro_bias);
    }

private:
    Eigen::Matrix<Scalar, 12, 1> gyro_bias_full_;
};

} // namespace tl

namespace nlohmann {

template <typename T>
void to_json(json& j, const tl::_ImuIntrinsics<T>& intrinsics)
{
    // TODO:
}

template <typename T>
void from_json(const json& j, tl::_ImuIntrinsics<T>& intrinsics)
{
    // TODO:
}

} // namespace nlohmann
