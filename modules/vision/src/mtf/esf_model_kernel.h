#pragma once

#include "esf_model.h"

class EsfModelKernel : public EsfModel
{
public:
    explicit EsfModelKernel(double in_alpha = 13) { set_alpha(in_alpha); }

    int build_esf(std::vector<Ordered_point>& ordered, double* sampled,
                  int fft_size, double max_distance_from_edge,
                  std::vector<double>& esf, Snr& snr,
                  bool allow_peak_shift = false) override;

    void compute_mtf_corrections() override;

    double kernel(double x) const;
};
