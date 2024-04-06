#pragma once

#include "mtf_renderer.h"
#include "output_version.h"

class Mtf_renderer_esf : public Mtf_renderer
{
public:
    Mtf_renderer_esf(const std::string& fname_esf, const std::string& fname_lsf,
                     Output_version::type output_version);

    void render(const std::vector<Block>& blocks) override;

public:
    std::string ofname_esf;
    std::string ofname_lsf;
    Output_version::type output_version;
};
