#include "thresholding.h"

#include <math.h>
#include <stdint.h>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include "threadpool.h"

namespace {

int next_pow2(uint32_t x)
{
    uint32_t k = 1;
    while (k < 31 && (uint32_t(1) << k) < x) {
        k++;
    }
    return uint32_t(1) << k;
}

} // namespace

// Reference:
// D. Bradley, G. Roth. ACM Journal of Graphics Tools. 2007. Vol 12, No. 2:
// 13-21.
void bradley_adaptive_threshold(const cv::Mat& cvimg, cv::Mat& out,
                                double threshold, int S)
{
    out = cv::Mat(cvimg.rows, cvimg.cols, CV_8UC1);

    uint64_t* integralImg = 0;
    int i, j;
    int64_t sum = 0;
    int64_t count = 0;
    int index;
    int x1, y1, x2, y2;
    int s2 = S / 2;

    // create the integral image
    integralImg = new uint64_t[out.rows * out.cols];

    for (i = 0; i < out.cols; i++) {
        // reset this column sum
        sum = 0;

        for (j = 0; j < out.rows; j++) {
            index = j * out.cols + i;

            sum += int64_t(cvimg.at<uint16_t>(j, i));
            if (i == 0) {
                integralImg[index] = sum;
            }
            else {
                integralImg[index] = integralImg[index - 1] + sum;
            }
        }
    }

    // perform thresholding
    auto it = cvimg.begin<uint16_t>();
    for (j = 0; j < out.rows; j++) {
        for (i = 0; i < out.cols; i++) {
            index = j * out.cols + i;

            // set the SxS region
            x1 = i - s2;
            x2 = i + s2;
            y1 = j - s2;
            y2 = j + s2;

            // check the border
            x1 = std::max(0, x1);
            x2 = std::min(x2, out.cols - 1);
            y1 = std::max(0, y1);
            y2 = std::min(y2, out.rows - 1);

            count = (x2 - x1) * (y2 - y1);

            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = integralImg[y2 * out.cols + x2] -
                  integralImg[y1 * out.cols + x2] -
                  integralImg[y2 * out.cols + x1] +
                  integralImg[y1 * out.cols + x1];

            if (((int64_t)(*it) * count) < (int64_t)(sum * (1.0 - threshold))) {
                out.data[index] = 0;
            }
            else {
                out.data[index] = 255;
            }
            ++it;
        }
    }

    delete[] integralImg;
}

// Reference:
// Efficient Implementation of Local Adaptive Thresholding Techniques Using
// Integral Images, F. Shafait, D. Keysers, T.M. Breuel, ...
void sauvola_adaptive_threshold(const cv::Mat& cvimg, cv::Mat& out,
                                double threshold, int S)
{
    auto& tp = ThreadPool::instance();

    out = cv::Mat(cvimg.rows, cvimg.cols, CV_8UC1);

    uint64_t sum = 0;
    uint64_t sq_sum = 0;
    const int s2 = S / 2;

    // create the integral image
    uint64_t* integralImg = new uint64_t[out.rows * out.cols];
    uint64_t* sq_integralImg = new uint64_t[out.rows * out.cols];

    uint16_t scale = 0;

    const uint16_t* it1 = cvimg.ptr<uint16_t>(0);
    // complete first row, then move on to the rest
    sum = sq_sum = 0;
    for (int i = 0; i < out.cols; i++) {
        scale = std::max(scale, *it1);
        sum += uint64_t(*it1);
        sq_sum += uint64_t(*it1) * uint64_t(*it1);

        integralImg[i] = sum;
        sq_integralImg[i] = sq_sum;
        ++it1;
    }

    for (int j = 1; j < out.rows; j++) {
        // reset this row sum
        sum = sq_sum = 0;
        int index = j * out.cols;

        for (int i = 0; i < out.cols; i++) {
            scale = std::max(scale, *it1);
            sum += uint64_t(*it1);
            sq_sum += uint64_t(*it1) * uint64_t(*it1);

            integralImg[index] = integralImg[index - out.cols] + sum;
            sq_integralImg[index] = sq_integralImg[index - out.cols] + sq_sum;

            ++it1;
            index++;
        }
    }

    double dscale = 2.0 / double(scale);

    std::vector<std::future<void>> futures;
    for (size_t block = 0; block < tp.size(); block++) {
        futures.emplace_back(tp.enqueue([&, block] {
            for (int j = block; j < out.rows; j += tp.size()) {
                int index_base = j * out.cols;
                const uint16_t* rowptr = cvimg.ptr<uint16_t>(0);

                for (int i = 0; i < out.cols; i++) {
                    int x1 = i - s2;
                    int x2 = i + s2;
                    int y1 = j - s2;
                    int y2 = j + s2;

                    // check the border
                    x1 = std::max(0, x1);
                    x2 = std::min(x2, out.cols - 1);
                    y1 = std::max(0, y1);
                    y2 = std::min(y2, out.rows - 1);

                    uint64_t count = (x2 - x1) * (y2 - y1);

                    uint64_t bsum = integralImg[y2 * out.cols + x2] -
                                    integralImg[y1 * out.cols + x2] -
                                    integralImg[y2 * out.cols + x1] +
                                    integralImg[y1 * out.cols + x1];

                    uint64_t bsq_sum = sq_integralImg[y2 * out.cols + x2] -
                                       sq_integralImg[y1 * out.cols + x2] -
                                       sq_integralImg[y2 * out.cols + x1] +
                                       sq_integralImg[y1 * out.cols + x1];

                    double mean = double(bsum) / count;
                    double sigma = sqrt(double(bsq_sum) / count - mean * mean);
                    double t = mean * (1 + threshold * (sigma * dscale - 1));

                    out.data[index_base + i] =
                        (rowptr[index_base + i] < t) ? 0 : 255;
                }
            }
        }));
    }
    for (const auto& future : futures) {
        future.wait();
    }

    delete[] integralImg;
    delete[] sq_integralImg;
}

void invert(cv::Mat& img)
{
    if (img.depth() != CV_16U) {
        LOG(ERROR) << "Invert: expected a 16-bit image at this point";
        return;
    }

    double minval = 0;
    double maxval = 0;
    int minidx = 0;
    int maxidx = 0;
    cv::minMaxIdx(img, &minval, &maxval, &minidx, &maxidx);
    uint32_t maxval_i = int(maxval);

    maxval_i = std::max(0, next_pow2(maxval_i) - 1);

    uint16_t* ptr = (uint16_t*)img.data;
    uint16_t* sentinel = ptr + size_t(img.cols) * size_t(img.rows);

    while (ptr < sentinel) {
        *ptr = maxval_i - *ptr;
        ptr++;
    }
}
