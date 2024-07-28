#include "esfsampler.h"

const std::array<std::string, 4> EsfSampler::esf_sampler_names = {
    {"line", "quadratic", "piecewise-quadratic", "deferred"}};

EsfSampler::EsfSampler(double max_dot, Bayer::cfa_mask_t cfa_mask,
                       double max_edge_length, double border_width)
    : max_dot(max_dot),
      default_cfa_mask(cfa_mask),
      max_edge_length(max_edge_length),
      border_width(border_width)
{
}

EsfSampler::esf_sampler_t EsfSampler::from_string(const std::string &s)
{
    for (size_t i = 0; i < esf_sampler_names.size(); i++) {
        if (s.compare(esf_sampler_names[i]) == 0) {
            return static_cast<esf_sampler_t>(i);
        }
    }
    return LINE; // fallback
}
