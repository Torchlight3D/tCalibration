#pragma once

#include "mtfrenderer.h"

class Mtf_renderer_print : public Mtf_renderer
{
public:
    Mtf_renderer_print(const std::string& fname, bool filter = false,
                       double angle = 0., bool lpmm_mode = false,
                       double pixel_size = 1.);

    void render(const std::vector<Block>& blocks) override;

public:
    std::string ofname;
    bool filter;
    double angle;
    bool lpmm_mode;
    double pixel_size;
};
