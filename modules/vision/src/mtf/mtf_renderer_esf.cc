#include "mtf_renderer_esf.h"

Mtf_renderer_esf::Mtf_renderer_esf(const std::string& fname_esf,
                                   const std::string& fname_lsf,
                                   Output_version::type output_version)
    : ofname_esf(fname_esf),
      ofname_lsf(fname_lsf),
      output_version(output_version)
{
}

void Mtf_renderer_esf::render(const std::vector<Block>& blocks)
{
    FILE* fout_esf = fopen(ofname_esf.c_str(), "wt");
    FILE* fout_lsf = fopen(ofname_lsf.c_str(), "wt");

    printf("output version = %d", (int)output_version);

    if (output_version >= Output_version::V2) {
        fprintf(fout_esf, "# MTF Mapper ESF, output format version %d \n",
                int(output_version));
        fprintf(fout_esf, "# column  1: block_id\n");
        fprintf(fout_esf, "# column  2: edge centroid x (pixels)\n");
        fprintf(fout_esf, "# column  3: edge centroid y (pixels)\n");
        fprintf(fout_esf, "# column  4: ESF sample spacing (pixels)\n");
        fprintf(fout_esf, "# column  5: number of ESF samples, N\n");
        fprintf(fout_esf, "# next N columns: ESF samples\n");

        fprintf(fout_lsf, "# MTF Mapper LSF, output format version %d \n",
                int(output_version));
        fprintf(fout_lsf, "# column  1: block_id\n");
        fprintf(fout_lsf, "# column  2: edge centroid x (pixels)\n");
        fprintf(fout_lsf, "# column  3: edge centroid y (pixels)\n");
        fprintf(fout_lsf, "# column  4: LSF sample spacing (pixels)\n");
        fprintf(fout_lsf, "# column  5: number of LSF samples, N\n");
        fprintf(fout_lsf, "# next N columns: LSF samples\n");
    }

    for (size_t i = 0; i < blocks.size(); i++) {
        for (size_t k = 0; k < 4; k++) {
            const std::vector<double>& esf = blocks[i].get_esf(k);

            if (output_version >= Output_version::V2) {
                fprintf(fout_esf, "%d %lf %lf 0.125 %d ", int(i),
                        blocks[i].get_edge_centroid(k).x,
                        blocks[i].get_edge_centroid(k).y, int(esf.size()));
            }

            for (size_t j = 0; j < esf.size(); j++) {
                fprintf(fout_esf, "%lf ", esf[j]);
            }
            fprintf(fout_esf, "\n");

            double sum = 0;
            for (size_t j = 1; j < esf.size() - 1; j++) {
                sum += (esf[j + 1] - esf[j - 1]) * 0.5;
            }

            if (output_version >= Output_version::V2) {
                fprintf(fout_lsf, "%d %lf %lf 0.125 %d ", int(i),
                        blocks[i].get_edge_centroid(k).x,
                        blocks[i].get_edge_centroid(k).y, int(esf.size()));
            }

            double sign = sum < 0 ? -1 : 1;
            fprintf(fout_lsf, "%lf ", 0.0);
            for (size_t j = 1; j < esf.size() - 1; j++) {
                fprintf(fout_lsf, "%lf ",
                        sign * (esf[j + 1] - esf[j - 1]) * 0.5);
            }
            fprintf(fout_lsf, " %lf\n", 0.0);
        }
    }
    fclose(fout_esf);
    fclose(fout_lsf);
}
