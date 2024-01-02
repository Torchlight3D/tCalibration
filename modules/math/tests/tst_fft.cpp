#include <gtest/gtest.h>
#include <glog/logging.h>

#include <AxMath/FFT>

using namespace thoht;

using Eigen::VectorXd;
using Eigen::VectorXcd;
using Eigen::MatrixXd;
using Eigen::MatrixXcd;
using cd = std::complex<double>;

TEST(FFT, FFTFreq)
{
    constexpr int len{8};
    constexpr double step{0.1};

    auto freq = fftfreq(len, step);
    VectorXd freq_gt{len};
    freq_gt << 0., 1.25, 2.5, 3.75, -5., -3.75, -2.5, -1.25;
    EXPECT_TRUE(freq.isApprox(freq_gt));
}

TEST(FFT, FFT1D)
{
    constexpr int len{24};

    VectorXd vec{len};
    for (long i{0}; i < vec.size(); ++i) {
        vec(i) = i + 0.;
    }

    // Could be higher, depends on how the ground truths are logged.
    constexpr double kPrecision{1e-5};

    auto vec_fft = fft_1d(vec);
    const VectorXcd vec_fft_gt{
        {cd{276., 0.},      cd{-12., 91.149},  cd{-12., 44.785},
         cd{-12., 28.971},  cd{-12., 20.785},  cd{-12., 15.639},
         cd{-12., 12},      cd{-12., 9.2079},  cd{-12., 6.9282},
         cd{-12., 4.9706},  cd{-12., 3.2154},  cd{-12., 1.5798},
         cd{-12., 0.},      cd{-12., -1.5798}, cd{-12., -3.2154},
         cd{-12., -4.9706}, cd{-12., -6.9282}, cd{-12., -9.2079},
         cd{-12., -12},     cd{-12., -15.639}, cd{-12., -20.785},
         cd{-12., -28.971}, cd{-12., -44.785}, cd{-12., -91.149}}};
    EXPECT_TRUE(vec_fft.isApprox(vec_fft_gt, kPrecision));

    auto vec_ifft = ifft_1d(vec_fft);
    EXPECT_TRUE(vec_ifft.real().isApprox(vec));

    auto vec_rfft = rfft_1d(vec);
    const VectorXcd vec_rfft_gt{
        {cd{276., 0.}, cd{-12., 91.149}, cd{-12., 44.785}, cd{-12., 28.971},
         cd{-12., 20.785}, cd{-12., 15.639}, cd{-12., 12.}, cd{-12., 9.2079},
         cd{-12., 6.9282}, cd{-12., 4.9706}, cd{-12., 3.2154}, cd{-12., 1.5798},
         cd{-12., 0.}}};
    EXPECT_TRUE(vec_rfft.isApprox(vec_rfft_gt, kPrecision));

    auto vec_irfft = irfft_1d(vec_rfft);
    EXPECT_TRUE(vec_irfft.isApprox(vec));
}

TEST(FFT, FFT2D)
{
    MatrixXd mat{6, 5};
    for (long r{0}; r < mat.rows(); ++r) {
        for (long c{0}; c < mat.cols(); ++c) {
            mat(r, c) = r * mat.cols() + c;
        }
    }

    auto mat_fft = fft_2d(mat);
    const MatrixXcd mat_fft_gt{
        {cd{435., 0.}, cd{-15., 20.6457288070676}, cd{-15., 4.8737954434936},
         cd{-15., -4.8737954434936}, cd{-15., -20.6457288070676}},
        {cd{-75., 129.903810567666}, cd{0., 0.}, cd{0., 0.}, cd{0., 0.},
         cd{0., 0.}},
        {cd{-75., 43.3012701892219}, cd{0., 0.}, cd{0., 0.}, cd{0., 0.},
         cd{0., 0.}},
        {cd{-75., 0.}, cd{0., 0.}, cd{0., 0.}, cd{0., 0.}, cd{0., 0.}},
        {cd{-75., -43.3012701892219}, cd{0., 0.}, cd{0., 0.}, cd{0., 0.},
         cd{0., 0.}},
        {cd{-75., -129.903810567666}, cd{0., 0.}, cd{0., 0.}, cd{0., 0.},
         cd{0., 0.}}};
    EXPECT_TRUE(mat_fft.isApprox(mat_fft_gt));

    auto mat_rfft = rfft_2d(mat);
    const MatrixXcd mat_rfft_gt{
        {cd{435., 0.}, cd{-15., 20.6457288070676}, cd{-15., 4.8737954434936}},
        {cd{-75., 129.903810567666}, cd{0., 0.}, cd{0., 0.}},
        {cd{-75., 43.3012701892219}, cd{0., 0.}, cd{0., 0.}},
        {cd{-75., 0.}, cd{0., 0.}, cd{0., 0.}},
        {cd{-75., -43.3012701892219}, cd{0., 0.}, cd{0., 0.}},
        {cd{-75., -129.903810567666}, cd{0., 0.}, cd{0., 0.}}};
    EXPECT_TRUE(mat_rfft.isApprox(mat_rfft_gt));

    auto mat_ifft = ifft_2d(mat_fft);
    EXPECT_TRUE(mat_ifft.real().isApprox(mat));

    auto mat_irfft = irfft_2d(mat_rfft);
    EXPECT_TRUE(mat_irfft.isApprox(mat));
}
