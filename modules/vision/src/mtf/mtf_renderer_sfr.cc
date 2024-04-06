#include "mtf_renderer_sfr.h"

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
                    angle_reduce(atan2(-blocks[i].get_normal(k).x,
                                       blocks[i].get_normal(k).y)));
            for (size_t j = 0; j < sfr.size(); j++) {
                fprintf(fout, "%lf ", sfr[j]);
            }
            fprintf(fout, "\n");
        }
    }
    fclose(fout);
}

double Mtf_renderer_sfr::angle_reduce(double x)
{
    double quad1 = fabs(fmod(x, M_PI / 2.0));
    if (quad1 > M_PI / 4.0) {
        quad1 = M_PI / 2.0 - quad1;
    }
    quad1 = quad1 / M_PI * 180;
    return quad1;
}
