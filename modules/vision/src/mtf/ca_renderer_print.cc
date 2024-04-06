#include "ca_renderer_print.h"
#include "edge_info.h"

Ca_renderer_print::Ca_renderer_print(const std::string& fname,
                                     const cv::Mat& img, bool allow_all_edges)
    : ofname(fname),
      img_centre(img.cols / 2, img.rows / 2),
      allow_all_edges(allow_all_edges)
{
}

void Ca_renderer_print::render(const std::vector<Block>& blocks)
{
    FILE* fout = fopen(ofname.c_str(), "wt");

    // Only Meridional direction, for now
    fprintf(fout, "# MTF Mapper CA, output format version 2\n");
    fprintf(fout, "# column  1: block_id\n");
    fprintf(fout, "# column  2: edge centroid x (pixels)\n");
    fprintf(fout, "# column  3: edge centroid y (pixels)\n");
    fprintf(
        fout,
        "# column  4: Red channel shift relative to Green channel (pixels)\n");
    fprintf(
        fout,
        "# column  5: Blue channel shift relative to Green channel (pixels)\n");
    fprintf(fout, "# column  6: Red channel shift relative to Green channel "
                  "(%% of radial distance)\n");
    fprintf(fout, "# column  7: Blue channel shift relative to Green channel "
                  "(%% of radial distance)\n");

    for (size_t i = 0; i < blocks.size(); i++) {
        if (!blocks[i].valid)
            continue;

        for (size_t k = 0; k < 4; k++) {
            cv::Point2d cent = blocks[i].get_edge_centroid(k);
            cv::Point2d dir = cent - img_centre;
            double centre_dist = std::max(1.0, norm(dir));
            dir = dir * (1.0 / norm(dir));
            double delta = dir.dot(blocks[i].get_normal(k));

            if (!blocks[i].get_edge_valid(k))
                continue;
            if (fabs(blocks[i].get_ca(k).x - Edge_info::nodata) < 1e-6 ||
                fabs(blocks[i].get_ca(k).y - Edge_info::nodata) < 1e-6)
                continue;

            if (fabs(delta) >= cos(45.0 / 180 * M_PI) || blocks.size() == 1 ||
                allow_all_edges) {
                fprintf(fout, "%d %lf %lf %lf %lf %lf %lf\n", int(i),
                        blocks[i].get_edge_centroid(k).x,
                        blocks[i].get_edge_centroid(k).y, blocks[i].get_ca(k).x,
                        blocks[i].get_ca(k).y,
                        100 * blocks[i].get_ca(k).x / centre_dist,
                        100 * blocks[i].get_ca(k).y / centre_dist);
            }
        }
    }

    fclose(fout);
}
