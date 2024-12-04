#include "mtfrenderersfr.h"

#include "utils.h"

Mtf_renderer_sfr::Mtf_renderer_sfr(const std::string& fname, bool lpmm_mode,
                                   double pixel_size)
    : ofname(fname), lpmm_mode(lpmm_mode), pixel_size(pixel_size)
{
}

void Mtf_renderer_sfr::render(const std::vector<Block>& blocks)
{
    FILE* fout = fopen(ofname.c_str(), "wt");
    for (size_t i = 0; i < blocks.size(); i++) {
        for (size_t k = 0; k < 4; k++) {
            const std::vector<double>& sfr = blocks[i].get_sfr(k);
            fprintf(fout, "%lf ",
                    tl::angle_reduce(std::atan2(-blocks[i].get_normal(k).x,
                                                blocks[i].get_normal(k).y)));
            for (size_t j = 0; j < sfr.size(); j++) {
                fprintf(fout, "%lf ", sfr[j]);
            }
            fprintf(fout, "\n");
        }
    }
    fclose(fout);
}
