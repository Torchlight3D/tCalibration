#include "fft.h"

#include <fftw3.h>

namespace thoht {

static_assert(sizeof(size_t) == sizeof(uint64_t),
              "Uncompatible compiler: sizeof(size_t) != 8 !");
static_assert(sizeof(double) == 8,
              "Uncompatible compiler: sizeof(double) != 8 !");
static_assert(sizeof(float) == 4,
              "Uncompatible compiler: sizeof( float) != 4 !");
static_assert(sizeof(int) == 4, "Uncompatible compiler: sizeof(   int) != 4 !");
static_assert(sizeof(std::complex<double>) == 16,
              "Uncompatible compiler: sizeof(complex<double>) != 16 !");
static_assert(sizeof(std::complex<float>) == 8,
              "Uncompatible compiler: sizeof(complex< float>) !=  8 !");

using Eigen::VectorXd;
using Eigen::VectorXcd;

VectorXcd fft_1d(const VectorXcd &vec)
{
    VectorXcd out(vec.size());
    auto plan = fftw_plan_dft_1d(vec.size(), (fftw_complex *)vec.data(),
                                 (fftw_complex *)out.data(), FFTW_FORWARD,
                                 FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    return out;
}

VectorXcd fft_1d(const VectorXd &vec)
{
    VectorXcd tmp = vec.cast<std::complex<double>>();
    return fft_1d(tmp);
}

VectorXcd ifft_1d(const VectorXcd &vec)
{
    VectorXcd out(vec.size());
    auto plan = fftw_plan_dft_1d(vec.size(), (fftw_complex *)vec.data(),
                                 (fftw_complex *)out.data(), FFTW_BACKWARD,
                                 FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    out /= out.size();
    return out;
}

VectorXcd ifft_1d(const VectorXd &vec)
{
    VectorXcd tmp = vec.cast<std::complex<double>>();
    return ifft_1d(tmp);
}

using Eigen::MatrixXd;
using Eigen::MatrixXcd;

using RowMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXcd = Eigen::Matrix<std::complex<double>, Eigen::Dynamic,
                                   Eigen::Dynamic, Eigen::RowMajor>;

MatrixXcd fft_2d(const MatrixXcd &mat)
{
    RowMatrixXcd row_mat{mat};
    RowMatrixXcd out(mat.rows(), mat.cols());
    auto plan = fftw_plan_dft_2d(
        mat.rows(), mat.cols(), (fftw_complex *)row_mat.data(),
        (fftw_complex *)out.data(), FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    return MatrixXcd{out};
}

MatrixXcd ifft_2d(const MatrixXcd &mat)
{
    RowMatrixXcd row_mat{mat};
    RowMatrixXcd out(mat.rows(), mat.cols());
    auto plan = fftw_plan_dft_2d(
        mat.rows(), mat.cols(), (fftw_complex *)row_mat.data(),
        (fftw_complex *)out.data(), FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    out /= out.size();
    return MatrixXcd{out};
}

MatrixXcd fft_2d(const MatrixXd &mat)
{
    MatrixXcd tmp = mat.cast<std::complex<double>>();
    return fft_2d(tmp);
}

MatrixXcd ifft_2d(const MatrixXd &mat)
{
    MatrixXcd tmp = mat.cast<std::complex<double>>();
    return ifft_2d(tmp);
}

// For real to complex FFT, out array only need to have a
// size of [in.size() / 2 + 1].
//  in array shape: m
// out array shape: m/2 + 1
VectorXcd rfft_1d(const VectorXd &vec)
{
    VectorXcd out(vec.size() / 2 + 1);
    auto plan = fftw_plan_dft_r2c_1d(vec.size(), (double *)vec.data(),
                                     (fftw_complex *)out.data(), FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    return out;
}

// fftw_c2r destroyes the original vector, so in this function
// you cannot pass `const T& in`
//  in array shape:  m
// out array shape:  (m-1) * 2
VectorXd irfft_1d(VectorXcd vec)
{
    // if original vector length is odd, size = in.size() * 2 - 1,
    //                             else, size = in.size() * 2 - 2;
    int size = fabs(vec.tail<1>()(0).imag()) > 1e-5 ? vec.size() * 2 - 1
                                                    : vec.size() * 2 - 2;
    VectorXd out(size);
    auto plan = fftw_plan_dft_c2r_1d(out.size(), (fftw_complex *)vec.data(),
                                     (double *)out.data(), FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    out /= out.size();
    return out;
}

// For real to comlex 2D-FFT:
//  in array shape:  m x n,
// out array shape:  m x (n/2 + 1);
MatrixXcd rfft_2d(const MatrixXd &mat)
{
    RowMatrixXd row_mat{mat};
    RowMatrixXcd out(mat.rows(), mat.cols() / 2 + 1);
    auto plan =
        fftw_plan_dft_r2c_2d(mat.rows(), mat.cols(), (double *)row_mat.data(),
                             (fftw_complex *)out.data(), FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    return MatrixXcd{out};
}

// fftw_c2r destroyes the original matrix, you cannot pass
// `const T& in`
//  in array shape: m x n
// out array shape: m x ((n-1) * 2)
MatrixXd irfft_2d(MatrixXcd mat)
{
    int cols = fabs(mat.rightCols<1>().head<1>()(0).imag()) > 1e-5
                   ? mat.cols() * 2 - 1
                   : mat.cols() * 2 - 2;

    RowMatrixXcd row_mat{mat};
    RowMatrixXd out(mat.rows(), cols);
    auto plan = fftw_plan_dft_c2r_2d(out.rows(), out.cols(),
                                     (fftw_complex *)row_mat.data(),
                                     (double *)out.data(), FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
    out /= out.size();
    return MatrixXd{out};
}

Eigen::VectorXd fftfreq(int len, double d)
{
    const auto val = 1. / (len * d);
    const auto head = (len - 1) / 2 + 1;
    const auto tail = len - head;

    Eigen::VectorXd res{len};
    res.head(head) = Eigen::VectorXi::LinSpaced(head, 0, head).cast<double>();
    res.tail(tail) = Eigen::VectorXi::LinSpaced(tail, -tail, -1).cast<double>();
    return res * val;
}

} // namespace thoht
