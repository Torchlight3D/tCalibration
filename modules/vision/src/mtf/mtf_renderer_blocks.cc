#include "mtf_renderer_blocks.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

Mtf_renderer_blocks::Mtf_renderer_blocks(const cv::Mat& in_img,
                                         const std::string& fname)
    : img(in_img), ofname(fname)
{
    out_img = img.clone();
}

void Mtf_renderer_blocks::render(const std::vector<Block>& blocks)
{
    cv::Point* pts = new cv::Point[4];
    for (size_t i = 0; i < blocks.size(); i++) {
        int npts = 4;
        const Mrectangle& rect = blocks[i].rect;
        for (size_t k = 0; k < 4; k++) {
            pts[k] = cv::Point(rect.corners[k].x, rect.corners[k].y);
        }
        cv::polylines(out_img, (const cv::Point**)&pts, &npts, 1, true,
                      CV_RGB(65535, 65535, 65535), 1, cv::LINE_AA);
    }
    delete[] pts;

    cv::imwrite(ofname, out_img);
}
