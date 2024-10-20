#pragma once

#include <Eigen/Dense>

#define KALMAN_VECTOR(NAME, T, N)                                 \
    using Base = kalman::Vector<T, N>;                            \
    using typename Base::Scalar;                                  \
    using Base::RowsAtCompileTime;                                \
    using Base::ColsAtCompileTime;                                \
    using Base::SizeAtCompileTime;                                \
                                                                  \
    NAME() : kalman::Vector<T, N>() {}                            \
                                                                  \
    template <typename OtherDerived>                              \
    NAME(const Eigen::MatrixBase<OtherDerived>& other)            \
        : kalman::Vector<T, N>(other)                             \
    {                                                             \
    }                                                             \
                                                                  \
    template <typename OtherDerived>                              \
    NAME& operator=(const Eigen::MatrixBase<OtherDerived>& other) \
    {                                                             \
        this->Base::operator=(other);                             \
        return *this;                                             \
    }

namespace tl {
namespace kalman {

inline constexpr auto Dynamic = Eigen::Dynamic;

template <typename T, int rows, int cols>
using Matrix = Eigen::Matrix<T, rows, cols>;

template <typename T, int N>
using Vector = Eigen::Vector<T, N>;

/**
 * @brief Cholesky square root decomposition of a symmetric positive-definite
 * matrix
 * @param _MatrixType The matrix type
 * @param _UpLo Square root form (Eigen::Lower or Eigen::Upper)
 */
template <typename _MatrixType, int _UpLo = Eigen::Lower>
class Cholesky : public Eigen::LLT<_MatrixType, _UpLo>
{
public:
    Cholesky() : Eigen::LLT<_MatrixType, _UpLo>() {}

    /**
     * @brief Construct cholesky square root decomposition from matrix
     * @param m The matrix to be decomposed
     */
    Cholesky(const _MatrixType& m) : Eigen::LLT<_MatrixType, _UpLo>(m) {}

    /**
     * @brief Set decomposition to identity
     */
    Cholesky& setIdentity()
    {
        this->m_matrix.setIdentity();
        this->m_isInitialized = true;
        return *this;
    }

    /**
     * @brief Check whether the decomposed matrix is the identity matrix
     */
    bool isIdentity() const
    {
        eigen_assert(this->m_isInitialized && "LLT is not initialized.");
        return this->m_matrix.isIdentity();
    }

    /**
     * @brief Set lower triangular part of the decomposition
     * @param matrix The lower part stored in a full matrix
     */
    template <typename Derived>
    Cholesky& setL(const Eigen::MatrixBase<Derived>& matrix)
    {
        this->m_matrix = matrix.template triangularView<Eigen::Lower>();
        this->m_isInitialized = true;
        return *this;
    }

    /**
     * @brief Set upper triangular part of the decomposition
     * @param matrix The upper part stored in a full matrix
     */
    template <typename Derived>
    Cholesky& setU(const Eigen::MatrixBase<Derived>& matrix)
    {
        this->m_matrix =
            matrix.template triangularView<Eigen::Upper>().adjoint();
        this->m_isInitialized = true;
        return *this;
    }
};

template <typename T, int N>
using SquareMatrix = Matrix<T, N, N>;

/**
 * @class kalman::Covariance
 * @brief Template type for covariance matrices
 * @param Type The vector type for which to generate a covariance (usually a
 * state or measurement type)
 */
template <class Type>
using Covariance = SquareMatrix<typename Type::Scalar, Type::RowsAtCompileTime>;

/**
 * @class kalman::CovarianceSquareRoot
 * @brief Template type for covariance square roots
 * @param Type The vector type for which to generate a covariance (usually a
 * state or measurement type)
 */
template <class Type>
using CovarianceSquareRoot = Cholesky<Covariance<Type>>;

/**
 * @class kalman::KalmanGain
 * @brief Template type of Kalman Gain
 * @param State The system state type
 * @param Measurement The measurement type
 */
template <class State, class Measurement>
using KalmanGain = Matrix<typename State::Scalar, State::RowsAtCompileTime,
                          Measurement::RowsAtCompileTime>;

// Jacobian of A w.r.t. B
template <class A, class B>
using Jacobian =
    Matrix<typename A::Scalar, A::RowsAtCompileTime, B::RowsAtCompileTime>;

} // namespace kalman
} // namespace tl
