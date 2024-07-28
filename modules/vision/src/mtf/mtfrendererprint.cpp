#include "mtfrendererprint.h"

Mtf_renderer_print::Mtf_renderer_print(const std::string& fname, bool filter,
                                       double angle, bool lpmm_mode,
                                       double pixel_size)
    : ofname(fname),
      filter(filter),
      angle(angle),
      lpmm_mode(lpmm_mode),
      pixel_size(pixel_size)
{
}

void Mtf_renderer_print::render(const std::vector<Block>& blocks)
{
    FILE* fout = fopen(ofname.c_str(), "wt");
    for (const auto& block : blocks) {
        if (!block.valid) {
            continue;
        }

        for (size_t k = 0; k < 4; k++) {
            double val = block.get_mtf50_value(k);
            if (filter) {
                double ba = block.get_edge_angle(k);
                double ad = acos(cos(angle) * cos(ba) + sin(angle) * sin(ba));
                if (fabs(ad) < 5.0 / 180.0 * M_PI ||
                    fabs(ad - M_PI) < 5.0 / 180.0 * M_PI) {
                    fprintf(fout, "%lf ", val);
                }
            }
            else {
                fprintf(fout, "%lf ", val);
            }
        }
        fprintf(fout, "\n");
    }
    fclose(fout);
}
