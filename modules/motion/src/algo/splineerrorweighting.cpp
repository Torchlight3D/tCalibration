#include "splineerrorweighting.h"

#include <unsupported/Eigen/FFT>
#include <glog/logging.h>

#include <tCore/Math>
#include <tMath/Solvers/BrentSolver>

namespace tl {

using Eigen::VectorXd;
using Eigen::MatrixX3cd;

namespace {

VectorXd fftfreq(int len, double d)
{
    const auto val = 1. / (len * d);
    const auto head = (len - 1) / 2 + 1;
    const auto tail = len - head;

    Eigen::VectorXd res{len};
    res.head(head) = Eigen::VectorXi::LinSpaced(head, 0, head).cast<double>();
    res.tail(tail) = Eigen::VectorXi::LinSpaced(tail, -tail, -1).cast<double>();
    return res * val;
}

inline double calcSignalEnergy(const VectorXd& signal)
{
    return signal.cwiseAbs().array().pow(2).sum() / signal.size();
}

// omega is angular velocity
VectorXd bspline_interp_freq_func(const VectorXd& omega, double spacing = 1.)
{
    auto calcH = [](const VectorXd& w) {
        auto sinc = [](double x) {
            if (x == 0.) {
                return 1.;
            }
            auto a = x * pi;
            return std::sin(a) / a;
        };

        auto a = 3 * w.unaryExpr([&sinc](double x) {
                          return sinc(x / two_pi_<double>);
                      })
                         .array()
                         .pow(4);
        auto b = 2. + w.array().cos();
        return a.cwiseQuotient(b);
    };

    return spacing * calcH(omega * spacing);
}

VectorXd spline_interpolation_response(const VectorXd& freqs, double spacing)
{
    auto H = bspline_interp_freq_func(freqs * two_pi_<double>, spacing);
    return H / H(0);
}

double dt_to_variance_spectrum(const VectorXd& spectrum, const VectorXd& freqs,
                               double spacing)
{
    VectorXd H = spline_interpolation_response(freqs, spacing);
    VectorXd ones{H.size()};
    ones.setOnes();
    auto energy = calcSignalEnergy((-H + ones).cwiseProduct(spectrum));
    return energy / spectrum.size();
}

VectorXd makeReferenceSpectrum(const ImuReadings& data)
{
    // Shape: N x 3
    const auto mat = data.toMatrix();

    Eigen::FFT<double> fft;
    MatrixX3cd mat_fft{mat.rows(), mat.cols()};
    for (int c{0}; c < mat.cols(); ++c) {
        Eigen::VectorXcd out = mat_fft.col(c);
        const Eigen::VectorXd in = mat.col(c);
        fft.fwd(out, in);
    }

    // Remove DC component
    mat_fft.row(0) = Eigen::RowVector3cd::Zero();

    auto x_hat = mat_fft.rowwise().norm() * std::sqrt(1. / 3.);
    return x_hat;
}

double findUniformKnotSpacingSpectrum(const VectorXd& xhat, const Timeline& t,
                                      double quality, double minSpacing = -1.,
                                      double maxSpacing = -1.)
{
    auto sampleRate = calcSampleRate(t);
    auto freqs = fftfreq(t.size(), 1. / sampleRate);
    auto max_remove = calcSignalEnergy(xhat) * (1. - quality);

    auto quality_func = [&xhat, &freqs, &max_remove](double spacing) -> double {
        VectorXd H = spline_interpolation_response(freqs, spacing);
        VectorXd ones{H.size()};
        ones.setOnes();
        auto removed = calcSignalEnergy((-H + ones).cwiseProduct(xhat));
        return max_remove / removed;
    };

    if (minSpacing < 0.) {
        minSpacing = 1 / sampleRate;
    }
    if (maxSpacing < 0.) {
        maxSpacing = t.size() * 0.25 / sampleRate;
    }

    // find_max_quality_dt
    // This could be in Options
    double minQuality = 1.0;

    auto dt = maxSpacing;
    auto q = quality_func(dt);
    if (q >= minQuality) {
        return dt;
    }

    // Backtrack
    auto step = maxSpacing * 0.5;
    double maxQuality{0.};
    double maxQualityDt{-1.};
    while (true) {
        dt -= step;
        dt = std::max(dt, minSpacing);

        q = quality_func(dt);

        if (q > minQuality) {
            auto func = [&quality_func, &minQuality](double dt) -> double {
                return quality_func(dt) - minQuality;
            };

            BrentSolver brent;
            auto brent_dt = brent.solve(func, dt, maxSpacing);
            return brent_dt;
        }
        else {
            step *= 0.5;
            if (q > maxQuality) {
                maxQuality = q;
                maxQualityDt = dt;
            }

            if (dt <= minSpacing) {
                // dt too small, return current best one
                return maxQualityDt;
            }
        }
    }
}
} // namespace

void SplineErrorWeighting::estKnotSpacingAndVariance(
    const ImuReadings& data, double quality, double minSpacing,
    double maxSpacing, double& spacing, double& var)
{
    const auto x_hat = makeReferenceSpectrum(data);

    const auto times = data.timeline();
    spacing = findUniformKnotSpacingSpectrum(x_hat, times, quality, minSpacing,
                                             maxSpacing);

    auto sample_rate = calcSampleRate(times);
    auto freqs = fftfreq(x_hat.size(), 1. / sample_rate);
    var = dt_to_variance_spectrum(x_hat, freqs, spacing);
}

} // namespace tl
