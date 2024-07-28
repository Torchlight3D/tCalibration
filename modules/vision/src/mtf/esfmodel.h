#pragma once

#include <array>
#include <string>
#include <vector>

#include "orderedpoint.h"
#include "snr.h"

// Brief:
// Edge Spread Function

class EsfModel
{
public:
    explicit EsfModel(double alpha = 13.7);
    virtual ~EsfModel();

    virtual int build_esf(std::vector<Ordered_point>& ordered, double* sampled,
                          int fft_size, double max_distance_from_edge,
                          std::vector<double>& esf, Snr& snr,
                          bool allow_peak_shift = false) = 0;

    virtual void set_alpha(double a);

    virtual void compute_mtf_corrections();

    void moving_average_smoother(std::vector<double>& smoothed, double* sampled,
                                 int fft_size, int fft_left, int fft_right,
                                 int left_trans, int right_trans,
                                 int width = 16);

    int estimate_esf_clipping(std::vector<Ordered_point>& ordered,
                              double* sampled, int fft_size,
                              bool allow_peak_shift, int effective_maxdot,
                              std::vector<double>& mean,
                              std::vector<double>& weights, int& fft_left,
                              int& fft_right, int& twidth, Snr& snr);

    const std::vector<double>& get_correction() const;

    void set_monotonic_filter(bool b);

public:
    const static std::array<std::string, 2> esf_model_names;

protected:
    double get_alpha() const;

    double sinc(double x);

protected:
    double alpha = 13.5;
    std::vector<double> w; // MTF correction weight
    static constexpr double missing = -1e7;
    bool apply_monotonic_filter = false;
};
