#pragma once

#include <vector>

#include "mtfprofilesample.h"
#include "mtfrenderer.h"

class Mtf_renderer_stats : public Mtf_renderer
{
public:
    Mtf_renderer_stats(bool lpmm_mode = false, double pixel_size = 1.0);

    void render(const std::vector<Block>& blocks) override;

    void render(const std::vector<Mtf_profile_sample>& samples);

    void print_stats(std::vector<double>& unfiltered,
                     std::vector<double>& filtered);

private:
    double quantile(const std::vector<double>& d, double q);

private:
    double pixel_size;
};
