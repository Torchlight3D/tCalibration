#pragma once

#include "esf_model.h"

class EsfModelLoss : public EsfModel
{
public:
    EsfModelLoss(double in_alpha = 5.5, [[maybe_unused]] double ridge = 5e-8)
    {
        set_alpha(in_alpha);
    }

    int build_esf(std::vector<Ordered_point>& ordered, double* sampled,
                  int fft_size, double max_distance_from_edge,
                  std::vector<double>& esf, Snr& snr,
                  bool allow_peak_shift = false) override;
};
