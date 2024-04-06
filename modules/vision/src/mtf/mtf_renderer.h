#pragma once

#include "block.h"

class Mtf_renderer
{
public:
    Mtf_renderer(const std::string& img_filename = {})
        : img_filename(img_filename)
    {
    }
    virtual ~Mtf_renderer() {}

    virtual void render(const std::vector<Block>& blocks) = 0;

    std::string img_filename;
};
