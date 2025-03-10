#pragma once

#include "mtfrenderer.h"

class Mtf_renderer_sfr : public Mtf_renderer
{
public:
    Mtf_renderer_sfr(const std::string& fname, bool lpmm_mode = false,
                     double pixel_size = 1.0);

    void render(const std::vector<Block>& blocks) override;

public:
    std::string ofname;
    bool lpmm_mode;
    double pixel_size;
};
