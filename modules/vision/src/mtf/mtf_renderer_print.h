#pragma once

#include "mtf_renderer.h"

class Mtf_renderer_print : public Mtf_renderer
{
public:
    Mtf_renderer_print(const std::string& fname, bool filter = false,
                       double angle = 0, bool lpmm_mode = false,
                       double pixel_size = 1.0)
        : ofname(fname),
          filter(filter),
          angle(angle),
          lpmm_mode(lpmm_mode),
          pixel_size(pixel_size)
    {
    }

    void render(const std::vector<Block>& blocks) override
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
                    double ad =
                        acos(cos(angle) * cos(ba) + sin(angle) * sin(ba));
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

public:
    std::string ofname;
    bool filter;
    double angle;
    bool lpmm_mode;
    double pixel_size;
};
