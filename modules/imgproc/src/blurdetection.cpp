#include "blurdetection.h"

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

namespace thoht {

// Refer to Kalibr
double DomainEntropyBlurDetection::evaluate(const cv::Mat& image)
{
    CV_Assert(image.channels() == 1 || image.channels() == 3);

    cv::Mat img;
    cv::cvtColor(image, img, cv::COLOR_RGBA2GRAY);
    img.convertTo(img, CV_32FC1);

    cv::dft(img, img, cv::DFT_COMPLEX_OUTPUT);

    Eigen::MatrixXcd gray{img.rows, img.rows};
    gray.setZero();
    cv::cv2eigen(img, gray);

    // TODO: fftshift to center on origin

    cv::Mat magnitude;
    cv::magnitude(img, img, magnitude);
    cv::normalize(magnitude, magnitude, 1, 0, cv::NORM_L1);

    cv::Mat fde;
    // fde = np.sum(magnitude_spectrum_normalized *
    // np.log(magnitude_spectrum_normalized))
    cv::log(magnitude, magnitude);

    // np.sum(fde)
    cv::sum(magnitude);

    return 0.;
}

double TenegradBlurDetection::evaluate(const cv::Mat& image)
{
    CV_Assert(image.channels() == 1 || image.channels() == 3);

    cv::Mat img;
    if (image.channels() == 3) {
        cv::cvtColor(image, img, cv::COLOR_RGBA2GRAY);
    }
    else {
        image.copyTo(img);
    }

    // Sobel
    constexpr int kKernelSize = 3;

    cv::Mat dx, dy;
    cv::Sobel(img, dx, CV_32F, 1, 0, kKernelSize);
    cv::Sobel(img, dy, CV_32F, 0, 1, kKernelSize);
    cv::Mat magnitude;
    cv::magnitude(dx, dy, magnitude);

    // Mean
    const double value = cv::mean(magnitude)[0];

    return value;
}

double LaplacianBlurDetection::evaluate(const cv::Mat& image)
{
    CV_Assert(image.channels() == 1 || image.channels() == 3);

    cv::Mat img;
    cv::cvtColor(image, img, cv::COLOR_RGBA2GRAY);

    // Laplacian
    cv::Laplacian(img, img, CV_8U);

    // Mean
    const double value = cv::mean(img)[0];

    return value;
}

double LaplacianModifiedBlurDetection::evaluate(const cv::Mat& image)
{
    CV_Assert(image.channels() == 1 || image.channels() == 3);

    cv::Mat img;
    cv::cvtColor(image, img, cv::COLOR_RGBA2GRAY);

    // Laplacian
    float kernelX[]{0, 0, 0, -1, 2, -1, 0, 0, 0};
    float kernelY[]{0, -1, 0, 0, 2, 0, 0, -1, 0};

    cv::Mat dx, dy;
    cv::filter2D(img, dx, -1, cv::Mat{3, 3, CV_32F, kernelX});
    cv::filter2D(img, dy, -1, cv::Mat{3, 3, CV_32F, kernelY});

    // Mean of absolute sum
    const double value = cv::mean(cv::abs(dx) + cv::abs(dy))[0];

    return value;
}

double VarianceBlurDetection::evaluate(const cv::Mat& image)
{
    CV_Assert(image.channels() == 1 || image.channels() == 3);

    cv::Mat img;
    cv::cvtColor(image, img, cv::COLOR_RGBA2GRAY);

    cv::Mat imgMean, imgStddev;
    cv::meanStdDev(img, imgMean, imgStddev);

    const double value = imgStddev.at<double>(0, 0);

    return value;
}

} // namespace thoht
