#pragma once

#include <Eigen/Dense>

namespace tl {

///------- Complex to complex
Eigen::VectorXcd fft_1d(const Eigen::VectorXcd& vec);
Eigen::VectorXcd ifft_1d(const Eigen::VectorXcd& vec);

///------- Real to Complex
Eigen::VectorXcd fft_1d(const Eigen::VectorXd& vec);
Eigen::VectorXcd ifft_1d(const Eigen::VectorXd& vec);

Eigen::MatrixXcd fft_2d(const Eigen::MatrixXcd& mat);
Eigen::MatrixXcd ifft_2d(const Eigen::MatrixXcd& mat);

Eigen::MatrixXcd fft_2d(const Eigen::MatrixXd& mat);
Eigen::MatrixXcd ifft_2d(const Eigen::MatrixXd& mat);

///------- Real
Eigen::VectorXcd rfft_1d(const Eigen::VectorXd& vec);
Eigen::VectorXd irfft_1d(Eigen::VectorXcd vec);

Eigen::MatrixXcd rfft_2d(const Eigen::MatrixXd& mat);
Eigen::MatrixXd irfft_2d(Eigen::MatrixXcd mat);

///------- Utils
Eigen::VectorXd fftfreq(int length, double spacing = 1.);

} // namespace tl
