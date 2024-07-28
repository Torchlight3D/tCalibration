#include "snr.h"

#include <cmath>

Snr::Snr(double dark_mean, double dark_sd, double bright_mean, double bright_sd)
    : dark_mean(dark_mean),
      dark_sd(dark_sd),
      bright_mean(bright_mean),
      bright_sd(bright_sd),
      contrast_val(bright_mean - dark_mean),
      oversampling_val(0)
{
}

double Snr::mean_cnr() const
{
    double denom =
        std::sqrt(0.5 * dark_sd * dark_sd + 0.5 * bright_sd * bright_sd);
    return std::min(10000.0, contrast_val / std::max(0.01, denom));
}

double Snr::dark_cnr() const
{
    return std::min(10000.0, contrast_val / std::max(0.01, dark_sd));
}

double Snr::bright_cnr() const
{
    return std::min(10000.0, contrast_val / std::max(0.01, bright_sd));
}

double Snr::dark_snr() const
{
    return std::min(10000.0, dark_mean / std::max(0.01, dark_sd));
}

double Snr::bright_snr() const
{
    return std::min(10000.0, bright_mean / std::max(0.01, bright_sd));
}

double Snr::contrast() const { return contrast_val; }

double Snr::oversampling() const { return oversampling_val; }

void Snr::set_oversampling(double val) { oversampling_val = val; }
