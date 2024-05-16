#include "phasecorr.h"

#include <vector>

#include <opencv2/core.hpp>

namespace phasecorrelation {

void magSpectrums(cv::InputArray _src, cv::OutputArray _dst)
{
    cv::Mat src = _src.getMat();
    int depth = src.depth(), cn = src.channels(), type = src.type();
    int rows = src.rows, cols = src.cols;
    int j, k;

    CV_Assert(type == CV_32FC1 || type == CV_32FC2 || type == CV_64FC1 ||
              type == CV_64FC2);

    if (src.depth() == CV_32F)
        _dst.create(src.rows, src.cols, CV_32FC1);
    else
        _dst.create(src.rows, src.cols, CV_64FC1);

    cv::Mat dst = _dst.getMat();
    dst.setTo(0); // Mat elements are not equal to zero by default!

    bool is_1d =
        (rows == 1 || (cols == 1 && src.isContinuous() && dst.isContinuous()));

    if (is_1d)
        cols = cols + rows - 1, rows = 1;

    int ncols = cols * cn;
    int j0 = cn == 1;
    int j1 = ncols - (cols % 2 == 0 && cn == 1);

    if (depth == CV_32F) {
        const float* dataSrc = src.ptr<float>();
        float* dataDst = dst.ptr<float>();

        size_t stepSrc = src.step / sizeof(dataSrc[0]);
        size_t stepDst = dst.step / sizeof(dataDst[0]);

        if (!is_1d && cn == 1) {
            for (k = 0; k < (cols % 2 ? 1 : 2); k++) {
                if (k == 1)
                    dataSrc += cols - 1, dataDst += cols - 1;
                dataDst[0] = dataSrc[0] * dataSrc[0];
                if (rows % 2 == 0)
                    dataDst[(rows - 1) * stepDst] =
                        dataSrc[(rows - 1) * stepSrc] *
                        dataSrc[(rows - 1) * stepSrc];

                for (j = 1; j <= rows - 2; j += 2) {
                    dataDst[j * stepDst] = (float)std::sqrt(
                        (double)dataSrc[j * stepSrc] * dataSrc[j * stepSrc] +
                        (double)dataSrc[(j + 1) * stepSrc] *
                            dataSrc[(j + 1) * stepSrc]);
                }

                if (k == 1)
                    dataSrc -= cols - 1, dataDst -= cols - 1;
            }
        }

        for (; rows--; dataSrc += stepSrc, dataDst += stepDst) {
            if (is_1d && cn == 1) {
                dataDst[0] = dataSrc[0] * dataSrc[0];
                if (cols % 2 == 0)
                    dataDst[j1] = dataSrc[j1] * dataSrc[j1];
            }

            for (j = j0; j < j1; j += 2) {
                dataDst[j] =
                    (float)std::sqrt((double)dataSrc[j] * dataSrc[j] +
                                     (double)dataSrc[j + 1] * dataSrc[j + 1]);
            }
        }
    }
    else {
        const double* dataSrc = src.ptr<double>();
        double* dataDst = dst.ptr<double>();

        size_t stepSrc = src.step / sizeof(dataSrc[0]);
        size_t stepDst = dst.step / sizeof(dataDst[0]);

        if (!is_1d && cn == 1) {
            for (k = 0; k < (cols % 2 ? 1 : 2); k++) {
                if (k == 1)
                    dataSrc += cols - 1, dataDst += cols - 1;
                dataDst[0] = dataSrc[0] * dataSrc[0];
                if (rows % 2 == 0)
                    dataDst[(rows - 1) * stepDst] =
                        dataSrc[(rows - 1) * stepSrc] *
                        dataSrc[(rows - 1) * stepSrc];

                for (j = 1; j <= rows - 2; j += 2) {
                    dataDst[j * stepDst] =
                        std::sqrt(dataSrc[j * stepSrc] * dataSrc[j * stepSrc] +
                                  dataSrc[(j + 1) * stepSrc] *
                                      dataSrc[(j + 1) * stepSrc]);
                }

                if (k == 1)
                    dataSrc -= cols - 1, dataDst -= cols - 1;
            }
        }

        for (; rows--; dataSrc += stepSrc, dataDst += stepDst) {
            if (is_1d && cn == 1) {
                dataDst[0] = dataSrc[0] * dataSrc[0];
                if (cols % 2 == 0)
                    dataDst[j1] = dataSrc[j1] * dataSrc[j1];
            }

            for (j = j0; j < j1; j += 2) {
                dataDst[j] = std::sqrt(dataSrc[j] * dataSrc[j] +
                                       dataSrc[j + 1] * dataSrc[j + 1]);
            }
        }
    }
}

void divSpectrums(cv::InputArray _srcA, cv::InputArray _srcB,
                  cv::OutputArray _dst, int flags, bool conjB)
{
    cv::Mat srcA = _srcA.getMat(), srcB = _srcB.getMat();
    int depth = srcA.depth(), cn = srcA.channels(), type = srcA.type();
    int rows = srcA.rows, cols = srcA.cols;
    int j, k;

    CV_Assert(type == srcB.type() && srcA.size() == srcB.size());
    CV_Assert(type == CV_32FC1 || type == CV_32FC2 || type == CV_64FC1 ||
              type == CV_64FC2);

    _dst.create(srcA.rows, srcA.cols, type);
    cv::Mat dst = _dst.getMat();

    bool is_1d = (flags & cv::DFT_ROWS) ||
                 (rows == 1 || (cols == 1 && srcA.isContinuous() &&
                                srcB.isContinuous() && dst.isContinuous()));

    if (is_1d && !(flags & cv::DFT_ROWS))
        cols = cols + rows - 1, rows = 1;

    int ncols = cols * cn;
    int j0 = cn == 1;
    int j1 = ncols - (cols % 2 == 0 && cn == 1);

    if (depth == CV_32F) {
        const float* dataA = srcA.ptr<float>();
        const float* dataB = srcB.ptr<float>();
        float* dataC = dst.ptr<float>();
        float eps = FLT_EPSILON; // prevent div0 problems

        size_t stepA = srcA.step / sizeof(dataA[0]);
        size_t stepB = srcB.step / sizeof(dataB[0]);
        size_t stepC = dst.step / sizeof(dataC[0]);

        if (!is_1d && cn == 1) {
            for (k = 0; k < (cols % 2 ? 1 : 2); k++) {
                if (k == 1)
                    dataA += cols - 1, dataB += cols - 1, dataC += cols - 1;
                dataC[0] = dataA[0] / (dataB[0] + eps);
                if (rows % 2 == 0)
                    dataC[(rows - 1) * stepC] =
                        dataA[(rows - 1) * stepA] /
                        (dataB[(rows - 1) * stepB] + eps);
                if (!conjB)
                    for (j = 1; j <= rows - 2; j += 2) {
                        double denom =
                            (double)dataB[j * stepB] * dataB[j * stepB] +
                            (double)dataB[(j + 1) * stepB] *
                                dataB[(j + 1) * stepB] +
                            (double)eps;

                        double re =
                            (double)dataA[j * stepA] * dataB[j * stepB] +
                            (double)dataA[(j + 1) * stepA] *
                                dataB[(j + 1) * stepB];

                        double im =
                            (double)dataA[(j + 1) * stepA] * dataB[j * stepB] -
                            (double)dataA[j * stepA] * dataB[(j + 1) * stepB];

                        dataC[j * stepC] = (float)(re / denom);
                        dataC[(j + 1) * stepC] = (float)(im / denom);
                    }
                else
                    for (j = 1; j <= rows - 2; j += 2) {
                        double denom =
                            (double)dataB[j * stepB] * dataB[j * stepB] +
                            (double)dataB[(j + 1) * stepB] *
                                dataB[(j + 1) * stepB] +
                            (double)eps;

                        double re =
                            (double)dataA[j * stepA] * dataB[j * stepB] -
                            (double)dataA[(j + 1) * stepA] *
                                dataB[(j + 1) * stepB];

                        double im =
                            (double)dataA[(j + 1) * stepA] * dataB[j * stepB] +
                            (double)dataA[j * stepA] * dataB[(j + 1) * stepB];

                        dataC[j * stepC] = (float)(re / denom);
                        dataC[(j + 1) * stepC] = (float)(im / denom);
                    }
                if (k == 1)
                    dataA -= cols - 1, dataB -= cols - 1, dataC -= cols - 1;
            }
        }

        for (; rows--; dataA += stepA, dataB += stepB, dataC += stepC) {
            if (is_1d && cn == 1) {
                dataC[0] = dataA[0] / (dataB[0] + eps);
                if (cols % 2 == 0)
                    dataC[j1] = dataA[j1] / (dataB[j1] + eps);
            }

            if (!conjB)
                for (j = j0; j < j1; j += 2) {
                    double denom = (double)(dataB[j] * dataB[j] +
                                            dataB[j + 1] * dataB[j + 1] + eps);
                    double re = (double)(dataA[j] * dataB[j] +
                                         dataA[j + 1] * dataB[j + 1]);
                    double im = (double)(dataA[j + 1] * dataB[j] -
                                         dataA[j] * dataB[j + 1]);
                    dataC[j] = (float)(re / denom);
                    dataC[j + 1] = (float)(im / denom);
                }
            else
                for (j = j0; j < j1; j += 2) {
                    double denom = (double)(dataB[j] * dataB[j] +
                                            dataB[j + 1] * dataB[j + 1] + eps);
                    double re = (double)(dataA[j] * dataB[j] -
                                         dataA[j + 1] * dataB[j + 1]);
                    double im = (double)(dataA[j + 1] * dataB[j] +
                                         dataA[j] * dataB[j + 1]);
                    dataC[j] = (float)(re / denom);
                    dataC[j + 1] = (float)(im / denom);
                }
        }
    }
    else {
        const double* dataA = srcA.ptr<double>();
        const double* dataB = srcB.ptr<double>();
        double* dataC = dst.ptr<double>();
        double eps = DBL_EPSILON; // prevent div0 problems

        size_t stepA = srcA.step / sizeof(dataA[0]);
        size_t stepB = srcB.step / sizeof(dataB[0]);
        size_t stepC = dst.step / sizeof(dataC[0]);

        if (!is_1d && cn == 1) {
            for (k = 0; k < (cols % 2 ? 1 : 2); k++) {
                if (k == 1)
                    dataA += cols - 1, dataB += cols - 1, dataC += cols - 1;
                dataC[0] = dataA[0] / (dataB[0] + eps);
                if (rows % 2 == 0)
                    dataC[(rows - 1) * stepC] =
                        dataA[(rows - 1) * stepA] /
                        (dataB[(rows - 1) * stepB] + eps);
                if (!conjB)
                    for (j = 1; j <= rows - 2; j += 2) {
                        double denom =
                            dataB[j * stepB] * dataB[j * stepB] +
                            dataB[(j + 1) * stepB] * dataB[(j + 1) * stepB] +
                            eps;

                        double re =
                            dataA[j * stepA] * dataB[j * stepB] +
                            dataA[(j + 1) * stepA] * dataB[(j + 1) * stepB];

                        double im = dataA[(j + 1) * stepA] * dataB[j * stepB] -
                                    dataA[j * stepA] * dataB[(j + 1) * stepB];

                        dataC[j * stepC] = re / denom;
                        dataC[(j + 1) * stepC] = im / denom;
                    }
                else
                    for (j = 1; j <= rows - 2; j += 2) {
                        double denom =
                            dataB[j * stepB] * dataB[j * stepB] +
                            dataB[(j + 1) * stepB] * dataB[(j + 1) * stepB] +
                            eps;

                        double re =
                            dataA[j * stepA] * dataB[j * stepB] -
                            dataA[(j + 1) * stepA] * dataB[(j + 1) * stepB];

                        double im = dataA[(j + 1) * stepA] * dataB[j * stepB] +
                                    dataA[j * stepA] * dataB[(j + 1) * stepB];

                        dataC[j * stepC] = re / denom;
                        dataC[(j + 1) * stepC] = im / denom;
                    }
                if (k == 1)
                    dataA -= cols - 1, dataB -= cols - 1, dataC -= cols - 1;
            }
        }

        for (; rows--; dataA += stepA, dataB += stepB, dataC += stepC) {
            if (is_1d && cn == 1) {
                dataC[0] = dataA[0] / (dataB[0] + eps);
                if (cols % 2 == 0)
                    dataC[j1] = dataA[j1] / (dataB[j1] + eps);
            }

            if (!conjB)
                for (j = j0; j < j1; j += 2) {
                    double denom =
                        dataB[j] * dataB[j] + dataB[j + 1] * dataB[j + 1] + eps;
                    double re =
                        dataA[j] * dataB[j] + dataA[j + 1] * dataB[j + 1];
                    double im =
                        dataA[j + 1] * dataB[j] - dataA[j] * dataB[j + 1];
                    dataC[j] = re / denom;
                    dataC[j + 1] = im / denom;
                }
            else
                for (j = j0; j < j1; j += 2) {
                    double denom =
                        dataB[j] * dataB[j] + dataB[j + 1] * dataB[j + 1] + eps;
                    double re =
                        dataA[j] * dataB[j] - dataA[j + 1] * dataB[j + 1];
                    double im =
                        dataA[j + 1] * dataB[j] + dataA[j] * dataB[j + 1];
                    dataC[j] = re / denom;
                    dataC[j + 1] = im / denom;
                }
        }
    }
}

void fftShift(cv::InputOutputArray _out)
{
    cv::Mat out = _out.getMat();

    if (out.rows == 1 && out.cols == 1) {
        // trivially shifted.
        return;
    }

    std::vector<cv::Mat> planes;
    planes.push_back(out);

    int xMid = out.cols >> 1;
    int yMid = out.rows >> 1;

    bool is_1d = xMid == 0 || yMid == 0;

    if (is_1d) {
        xMid = xMid + yMid;

        for (size_t i = 0; i < planes.size(); i++) {
            cv::Mat tmp;
            cv::Mat half0(planes[i], cv::Rect(0, 0, xMid, 1));
            cv::Mat half1(planes[i], cv::Rect(xMid, 0, xMid, 1));

            half0.copyTo(tmp);
            half1.copyTo(half0);
            tmp.copyTo(half1);
        }
    }
    else {
        for (size_t i = 0; i < planes.size(); i++) {
            // perform quadrant swaps...
            cv::Mat tmp;
            cv::Mat q0(planes[i], cv::Rect(0, 0, xMid, yMid));
            cv::Mat q1(planes[i], cv::Rect(xMid, 0, xMid, yMid));
            cv::Mat q2(planes[i], cv::Rect(0, yMid, xMid, yMid));
            cv::Mat q3(planes[i], cv::Rect(xMid, yMid, xMid, yMid));

            q0.copyTo(tmp);
            q3.copyTo(q0);
            tmp.copyTo(q3);

            q1.copyTo(tmp);
            q2.copyTo(q1);
            tmp.copyTo(q2);
        }
    }

    cv::merge(planes, out);
}

cv::Point2d weightedCentroid(cv::InputArray _src, cv::Point peakLocation,
                             cv::Size weightBoxSize, double* response)
{
    cv::Mat src = _src.getMat();

    int type = src.type();
    CV_Assert(type == CV_32FC1 || type == CV_64FC1);

    int minr = peakLocation.y - (weightBoxSize.height >> 1);
    int maxr = peakLocation.y + (weightBoxSize.height >> 1);
    int minc = peakLocation.x - (weightBoxSize.width >> 1);
    int maxc = peakLocation.x + (weightBoxSize.width >> 1);

    cv::Point2d centroid;
    double sumIntensity = 0.0;

    // clamp the values to min and max if needed.
    if (minr < 0) {
        minr = 0;
    }

    if (minc < 0) {
        minc = 0;
    }

    if (maxr > src.rows - 1) {
        maxr = src.rows - 1;
    }

    if (maxc > src.cols - 1) {
        maxc = src.cols - 1;
    }

    if (type == CV_32FC1) {
        const float* dataIn = src.ptr<float>();
        dataIn += minr * src.cols;
        for (int y = minr; y <= maxr; y++) {
            for (int x = minc; x <= maxc; x++) {
                centroid.x += (double)x * dataIn[x];
                centroid.y += (double)y * dataIn[x];
                sumIntensity += (double)dataIn[x];
            }

            dataIn += src.cols;
        }
    }
    else {
        const double* dataIn = src.ptr<double>();
        dataIn += minr * src.cols;
        for (int y = minr; y <= maxr; y++) {
            for (int x = minc; x <= maxc; x++) {
                centroid.x += (double)x * dataIn[x];
                centroid.y += (double)y * dataIn[x];
                sumIntensity += dataIn[x];
            }

            dataIn += src.cols;
        }
    }

    if (response)
        *response = sumIntensity;

    sumIntensity += DBL_EPSILON; // prevent div0 problems...

    centroid.x /= sumIntensity;
    centroid.y /= sumIntensity;

    return centroid;
}

cv::Point2d phaseCorrelate(cv::InputArray _src1, cv::InputArray _src2,
                           cv::InputArray _window, double* response)
{
    cv::Mat src1 = _src1.getMat();
    cv::Mat src2 = _src2.getMat();
    cv::Mat window = _window.getMat();

    CV_Assert(src1.type() == src2.type());
    CV_Assert(src1.type() == CV_32FC1 || src1.type() == CV_64FC1);
    CV_Assert(src1.size == src2.size);

    if (!window.empty()) {
        CV_Assert(src1.type() == window.type());
        CV_Assert(src1.size == window.size);
    }

    int M = cv::getOptimalDFTSize(src1.rows);
    int N = cv::getOptimalDFTSize(src1.cols);

    cv::Mat padded1, padded2, paddedWin;

    if (M != src1.rows || N != src1.cols) {
        cv::copyMakeBorder(src1, padded1, 0, M - src1.rows, 0, N - src1.cols,
                           cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::copyMakeBorder(src2, padded2, 0, M - src2.rows, 0, N - src2.cols,
                           cv::BORDER_CONSTANT, cv::Scalar::all(0));

        if (!window.empty()) {
            cv::copyMakeBorder(window, paddedWin, 0, M - window.rows, 0,
                               N - window.cols, cv::BORDER_CONSTANT,
                               cv::Scalar::all(0));
        }
    }
    else {
        padded1 = src1;
        padded2 = src2;
        paddedWin = window;
    }

    // perform window multiplication if available
    if (!paddedWin.empty()) {
        // apply window to both images before proceeding...
        cv::multiply(paddedWin, padded1, padded1);
        cv::multiply(paddedWin, padded2, padded2);
    }

    // execute phase correlation equation
    // Reference: http://en.wikipedia.org/wiki/Phase_correlation
    cv::Mat FFT1, FFT2;
    dft(padded1, FFT1, cv::DFT_COMPLEX_OUTPUT);
    dft(padded2, FFT2, cv::DFT_COMPLEX_OUTPUT);

    //    // high-pass filter
    //    cv::Mat hpFilter = 1-paddedWin;
    //    phasecorrelation::fftShift(hpFilter);
    //    for(int i=0; i<paddedWin.rows; i++){
    //        for(int j=0; j<paddedWin.cols; j++){
    //            FFT1.at<cv::Vec2f>(i,j) *= hpFilter.at<float>(i,j);
    //            FFT2.at<cv::Vec2f>(i,j) *= hpFilter.at<float>(i,j);
    //        }
    //    }

    cv::Mat P;
    cv::mulSpectrums(FFT1, FFT2, P, cv::DFT_COMPLEX_OUTPUT, true);

    cv::Mat Pm(P.size(), CV_32F);
    // NOTE: memleak in magSpectrums when using it with complex output!
    // phasecorrelation::magSpectrums(P, Pm);
    for (int i = 0; i < P.rows; i++) {
        for (int j = 0; j < P.cols; j++) {
            cv::Vec2f e = P.at<cv::Vec2f>(i, j);
            Pm.at<float>(i, j) = cv::sqrt(e[0] * e[0] + e[1] * e[1]);
        }
    }

    // phasecorrelation::divSpectrums(P, Pm, C, 0, false); // FF* / |FF*| (phase
    // correlation equation completed here...)
    for (int i = 0; i < P.rows; i++) {
        for (int j = 0; j < P.cols; j++) {
            P.at<cv::Vec2f>(i, j) /= (Pm.at<float>(i, j) + DBL_EPSILON);
        }
    }

    cv::Mat C(P.size(), CV_32F);
    cv::dft(P, C, cv::DFT_INVERSE + cv::DFT_REAL_OUTPUT);

    phasecorrelation::fftShift(
        C); // shift the energy to the center of the frame.

    // cvtools::writeMat(C, "C.mat", "C");

    // locate the highest peak
    cv::Point peakLoc;
    minMaxLoc(C, NULL, NULL, NULL, &peakLoc);

    // get the phase shift with sub-pixel accuracy, 5x5 window seems about right
    // here...
    cv::Point2d t = phasecorrelation::weightedCentroid(
        C, peakLoc, cv::Size(3, 3), response);

    // max response is M*N (not exactly, might be slightly larger due to
    // rounding errors)
    if (response)
        *response /= M * N;

    // adjust shift relative to image center...
    cv::Point2d center((double)padded1.cols / 2.0, (double)padded1.rows / 2.0);

    return (center - t);
}

void createHanningWindow(cv::OutputArray _dst, cv::Size winSize, int type)
{
    CV_Assert(type == CV_32FC1 || type == CV_64FC1);

    _dst.create(winSize, type);
    cv::Mat dst = _dst.getMat();

    int rows = dst.rows, cols = dst.cols;

    cv::AutoBuffer<double> _wc(cols);
    double* const wc = (double*)_wc;

    double coeff0 = 2.0 * CV_PI / (double)(cols - 1),
           coeff1 = 2.0f * CV_PI / (double)(rows - 1);
    for (int j = 0; j < cols; j++) wc[j] = 0.5 * (1.0 - cos(coeff0 * j));

    if (dst.depth() == CV_32F) {
        for (int i = 0; i < rows; i++) {
            float* dstData = dst.ptr<float>(i);
            double wr = 0.5 * (1.0 - cos(coeff1 * i));
            for (int j = 0; j < cols; j++) dstData[j] = (float)(wr * wc[j]);
        }
    }
    else {
        for (int i = 0; i < rows; i++) {
            double* dstData = dst.ptr<double>(i);
            double wr = 0.5 * (1.0 - cos(coeff1 * i));
            for (int j = 0; j < cols; j++) dstData[j] = wr * wc[j];
        }
    }

    // perform batch sqrt for SSE performance gains
    cv::sqrt(dst, dst);
}

} // namespace phasecorrelation
