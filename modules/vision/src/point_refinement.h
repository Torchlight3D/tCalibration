#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

namespace tl {

struct SaddlePoint
{
    double x, y;
    double a1, a2;
    double s, det;

    SaddlePoint() {}
    SaddlePoint(double x, double y) : x(x), y(y) {}
};

void initSaddlePointRefinement(int halfKernelSize, cv::Mat &saddleKernel,
                               cv::Mat &invAtAAt, cv::Mat &valid)
{
    const int windowSize = halfKernelSize * 2 + 1;
    saddleKernel.create(windowSize, windowSize, CV_64FC1);
    double maxVal = halfKernelSize + 1;
    double sum = 0;
    double *w = saddleKernel.ptr<double>(0);

    // circular kernel
    int cnt = 0;
    for (int y = -halfKernelSize; y <= halfKernelSize; y++) {
        for (int x = -halfKernelSize; x <= halfKernelSize; x++) {
            *w = maxVal - sqrt(x * x + y * y);
            if (*w > 0)
                cnt++;
            else
                *w = 0;
            sum += *w;
            w++;
        }
    }

    // scale kernel
    saddleKernel /= sum;

    cv::Mat A(cnt, 6, CV_64FC1);
    double *a = A.ptr<double>(0);
    w = saddleKernel.ptr<double>(0);
    valid.create(windowSize, windowSize, CV_8UC1);
    uint8_t *v = valid.ptr<uint8_t>(0);

    for (int y = -halfKernelSize; y <= halfKernelSize; y++)
        for (int x = -halfKernelSize; x <= halfKernelSize; x++) {
            if (*w > 0) {
                *v = 255;
                a[0] = x * x;
                a[1] = y * y;
                a[2] = x * y;
                a[3] = y;
                a[4] = x;
                a[5] = 1;
                a += 6;
            }
            else {
                *v = 0;
            }
            w++;
            v++;
        }

    // compute pseudoinverse of AtA
    cv::invert(A.t() * A, invAtAAt, cv::DECOMP_SVD);
    invAtAAt *= A.t();
}

template <typename PointType>
void saddleSubpixelRefinement(const cv::Mat &input,
                              const std::vector<PointType> &initial, cv::Mat &A,
                              cv::Mat &valid, std::vector<SaddlePoint> &refined,
                              int halfKernelSize = 3, int maxIterations = 3)
{
    cv::Mat b(A.cols, 1, CV_64FC1);

    refined.resize(initial.size());
    for (size_t idx{0}; idx < initial.size(); idx++) {
        refined[idx] = SaddlePoint(initial[idx].x, initial[idx].y);
    }

    // int diverged = 0, iterations = 0;
    int width = input.cols;
    int height = input.rows;
    for (size_t idx{0}; idx < refined.size(); idx++) {
        auto &pt = refined[idx];
        // printf("initial: %.3f, %.3f: ", pt.x, pt.y);

        cv::Mat p;
        for (int it = 0; it < maxIterations; it++) {
            if (pt.x > halfKernelSize + 1 &&
                pt.x < width - (halfKernelSize + 2) &&
                pt.y > halfKernelSize + 1 &&
                pt.y < height - (halfKernelSize + 2)) {
                int x0 = int(pt.x);
                int y0 = int(pt.y);
                double xw = pt.x - x0;
                double yw = pt.y - y0;

                // precompute bilinear interpolation weights
                double w00 = (1.0 - xw) * (1.0 - yw), w01 = xw * (1.0 - yw),
                       w10 = (1.0 - xw) * yw, w11 = xw * yw;

                // fit to local neighborhood = b vector...
                double *m = b.ptr<double>(0);
                uint8_t *v = valid.ptr<uint8_t>(0);

                for (int y = -halfKernelSize; y <= halfKernelSize; y++) {
                    const double *im00 = input.ptr<double>(y0 + y);
                    const double *im10 = input.ptr<double>(y0 + y + 1);
                    for (int x = -halfKernelSize; x <= halfKernelSize; x++) {
                        if (*v > 0) {
                            const int col0 = x0 + x;
                            const int col1 = col0 + 1;
                            *(m++) = im00[col0] * w00 + im00[col1] * w01 +
                                     im10[col0] * w10 + im10[col1] * w11;
                        }
                        v++;
                    }
                }
                // fit quadric to surface by solving LSQ
                p = A * b;

                // k5, k4, k3, k2, k1, k0
                // 0 , 1 , 2 , 3 , 4 , 5
                double *r = p.ptr<double>(0);
                pt.det =
                    4.0 * r[0] * r[1] - r[2] * r[2]; // 4.0 * k5 * k4 - k3 * k3
                                                     // compute the new location
                double dx = (-2 * r[1] * r[4] + r[2] * r[3]) /
                            pt.det; // - 2 * k4 * k1 +     k3 * k2
                double dy = (r[2] * r[4] - 2 * r[0] * r[3]) /
                            pt.det; //       k3 * k1 - 2 * k5 * k2
                pt.x += dx;
                pt.y += dy;
                dx = fabs(dx);
                dy = fabs(dy);
                // iterations++;

                constexpr float step_threshold{0.001};
                if (it == maxIterations ||
                    (step_threshold > dx && step_threshold > dy)) {
                    // converged
                    double k4mk5 = r[1] - r[0];
                    pt.s = sqrt(r[2] * r[2] + k4mk5 * k4mk5);
                    pt.a1 = atan2(-r[2], k4mk5) / 2.0;
                    pt.a2 = acos((r[1] + r[0]) / pt.s) / 2.0;
                    break;
                }
                else {
                    // check for divergence
                    if (pt.det > 0 ||
                        fabs(pt.x - initial[idx].x) > halfKernelSize ||
                        fabs(pt.y - initial[idx].y) > halfKernelSize) {
                        pt.x = pt.y = std::numeric_limits<double>::infinity();
                        // diverged++;
                        break;
                    }
                }
            }
            else {
                // too close to border...
                pt.x = pt.y = std::numeric_limits<double>::infinity();
                // diverged++;
                break;
            }
        }
    }
}

template <typename PointType>
void saddleSubpixelRefinement(const cv::Mat &input,
                              const std::vector<PointType> &initial,
                              std::vector<SaddlePoint> &refined,
                              int halfKernelSize = 2, int maxIterations = 20)
{
    cv::Mat weights, A, valid;
    initSaddlePointRefinement(halfKernelSize, weights, A, valid);

    cv::Mat smooth;
    cv::filter2D(input, smooth, CV_64FC1, weights);

    saddleSubpixelRefinement(smooth, initial, A, valid, refined, halfKernelSize,
                             maxIterations);
}

} // namespace tl
