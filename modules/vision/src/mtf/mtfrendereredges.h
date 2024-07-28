#pragma once

#include "mtfrenderer.h"
#include "outputversion.h"
#include "jobmetadata.h"

class Mtf_renderer_edges : public Mtf_renderer
{
public:
    Mtf_renderer_edges(const std::string& fname, const std::string& sfrname,
                       const std::string& devname,
                       const std::string& serialname,
                       Output_version::type output_version,
                       const Job_metadata& metadata, bool lpmm_mode = false,
                       double pixel_size = 1.0);

    void render(const std::vector<Block>& blocks) override;

private:
    std::string ofname;
    std::string sfrname;
    std::string devname;
    std::string serialname;
    bool lpmm_mode;
    double pixel_size;
    Output_version::type output_version;
    Job_metadata metadata;
};
