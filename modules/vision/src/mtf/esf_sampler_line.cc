#include "esf_sampler_line.h"

EsfLineSampler::EsfLineSampler(double max_dot, Bayer::cfa_mask_t cfa_mask,
                               double border_width)
    : EsfSampler(max_dot, cfa_mask, 200, border_width)
{
}

void EsfLineSampler::sample(Edge_model &edge_model,
                            std::vector<Ordered_point> &local_ordered,
                            const std::map<int, scanline> &scanset,
                            double &edge_length, const cv::Mat &geom_img,
                            const cv::Mat &sampling_img,
                            Bayer::cfa_mask_t cfa_mask)
{
    cfa_mask = cfa_mask == Bayer::DEFAULT ? default_cfa_mask : cfa_mask;

    double max_along_edge = -1e50;
    double min_along_edge = 1e50;

    for (const auto &[y, scanline] : scanset) {
        if (y < border_width || y > geom_img.rows - 1 - border_width)
            continue;

        int rowcode = (y & 1) << 1;
        for (int x = scanline.start; x <= scanline.end; ++x) {
            if (x < border_width || x > geom_img.cols - 1 - border_width)
                continue;

            int code = 1 << ((rowcode | (x & 1)) ^ 3);
            if ((code & cfa_mask) == 0)
                continue;

            cv::Point2d d = cv::Point2d(x, y) - edge_model.get_centroid();
            double perp = d.ddot(edge_model.get_normal());
            double par = d.ddot(edge_model.get_direction());
            if (fabs(perp) < max_dot && fabs(par) < max_edge_length) {
                local_ordered.push_back(
                    Ordered_point(perp, sampling_img.at<uint16_t>(y, x)));
                max_along_edge = std::max(max_along_edge, par);
                min_along_edge = std::min(min_along_edge, par);
            }
        }
    }

    edge_length = max_along_edge - min_along_edge;
}
