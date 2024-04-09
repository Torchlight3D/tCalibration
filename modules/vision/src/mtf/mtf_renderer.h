#pragma once

#include "block.h"

class Mtf_renderer
{
public:
    explicit Mtf_renderer(const std::string& filename = {})
        : img_filename(filename)
    {
    }
    virtual ~Mtf_renderer() = default;

    virtual void render(const std::vector<Block>& blocks) = 0;

    std::string img_filename;
};
