#include "skydetection.h"

#include <numeric>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

namespace tl {

namespace SkyAreaDetector {

namespace internal {

// Skyline also could be cv::Mat(1, Width, CV_8UC)
using Skyline = std::vector<int>;

void calcImageGradient(cv::InputArray src, cv::OutputArray gradient)
{
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    cv::Mat x_gradient;
    cv::Sobel(gray, x_gradient, CV_64F, 1, 0);
    cv::Mat y_gradient;
    cv::Sobel(gray, y_gradient, CV_64F, 0, 1);

    // TODO: Use cv::magnitude?
    // cv::magnitude(x_gradient, y_gradient, gradient);
    cv::pow(x_gradient, 2, x_gradient);
    cv::pow(y_gradient, 2, y_gradient);
    cv::add(x_gradient, y_gradient, gradient);
    cv::sqrt(gradient, gradient);
}

std::vector<int> extractSkyline(const cv::Mat &gradient, double thresh)
{
    int image_height = gradient.size[0];
    int image_width = gradient.size[1];
    std::vector<int> skyline(image_width, image_height - 1);

    for (int col = 0; col < image_width; ++col) {
        int row_index = 0;
        for (int row = 0; row < image_height; ++row) {
            row_index = row;
            if (gradient.at<double>(row, col) > thresh) {
                skyline[col] = row;
                break;
            }
        }

        if (row_index == 0) {
            skyline[col] = image_height - 1;
        }
    }

    return skyline;
}

void separateImage(cv::InputArray src, const std::vector<int> &skyline,
                   cv::OutputArray skyMask, cv::OutputArray otherMask)
{
    skyMask.create(src.size(), CV_8UC1);
    {
        auto mask = skyMask.getMat();
        mask.forEach<uchar>([&skyline](uchar &val, const int *pos) {
            val = pos[1] <= skyline[pos[0]] ? 255 : 0;
        });
    }

    otherMask.create(src.size(), CV_8UC1);
    {
        auto mask = otherMask.getMat();
        mask.forEach<uchar>([&skyline](uchar &val, const int *pos) {
            val = pos[1] > skyline[pos[0]] ? 255 : 0;
        });
    }
}

double calcSkyRegionEnergy(const std::vector<int> &skyline, cv::InputArray src)
{
    cv::Mat skyMask, groundMask;
    separateImage(src, skyline, skyMask, groundMask);

    cv::Mat skyRegion, groundRegion;
    src.copyTo(skyRegion, skyMask);
    src.copyTo(groundRegion, groundMask);

    std::vector<cv::Vec3b> groundRegionValues;
    {
        std::vector<cv::Point> indices;
        cv::findNonZero(groundMask, indices);

        if (indices.empty()) {
            return -1.;
        }

        groundRegionValues.reserve(indices.size());
        for (const auto &index : indices) {
            groundRegionValues.push_back(groundRegion.at<cv::Vec3b>(index));
        }
    }

    std::vector<cv::Vec3b> skyRegionValues;
    {
        std::vector<cv::Point> indices;
        cv::findNonZero(skyMask, indices);

        if (indices.empty()) {
            return -1.;
        }

        skyRegionValues.reserve(indices.size());
        for (const auto &index : indices) {
            skyRegionValues.push_back(skyRegion.at<cv::Vec3b>(index));
        }
    }

    cv::Mat ground_covar, ground_mean;
    cv::calcCovarMatrix(groundRegionValues, ground_covar, ground_mean,
                        cv::COVAR_ROWS | cv::COVAR_NORMAL | cv::COVAR_SCALE);
    cv::Mat ground_eig_val, ground_eig_vec;
    cv::eigen(ground_covar, ground_eig_val, ground_eig_vec);

    cv::Mat sky_covar, sky_mean;
    cv::calcCovarMatrix(skyRegionValues, sky_covar, sky_mean,
                        cv::COVAR_ROWS | cv::COVAR_SCALE | cv::COVAR_NORMAL);
    cv::Mat sky_eig_val, sky_eig_vec;
    cv::eigen(sky_covar, sky_eig_val, sky_eig_vec);

    double ground_det = cv::determinant(ground_covar);
    double sky_det = cv::determinant(sky_covar);
    double ground_eig_det = cv::determinant(ground_eig_vec);
    double sky_eig_det = cv::determinant(sky_eig_vec);

    int para = 2;
    return 1. / ((para * sky_det + ground_det) +
                 (para * sky_eig_det + ground_eig_det));
}

bool checkSkyExist(const std::vector<int> &skyline, double minX,
                   double thresh_2, double maxDiffX)
{
    const auto xMean = std::accumulate(skyline.cbegin(), skyline.cend(), 0) /
                       static_cast<double>(skyline.size());

    if (xMean < minX) {
        return false;
    }

    // TODO:
    // 1. Use adjacent_diffdrence
    // 2. Merge with checkFalseSky
    std::vector<int> border_diff(static_cast<int>(skyline.size() - 1), 0);
    for (auto i = static_cast<int>(skyline.size() - 1); i >= 0; i--) {
        border_diff[i] = std::abs(skyline[i + 1] - skyline[i]);
    }

    const auto diffXMean =
        std::accumulate(border_diff.cbegin(), border_diff.cend(), 0) /
        static_cast<double>(border_diff.size());

    return !(diffXMean > maxDiffX && xMean < thresh_2);
}

bool checkFalsePositiveSky(const std::vector<int> &border, double thresh_1)
{
    // TODO:
    // 1. Use adjacent_difference
    // 2. Return earlier
    std::vector<int> border_diff(static_cast<int>(border.size() - 1), 0);
    for (int i = static_cast<int>(border.size() - 1); i >= 0; i--) {
        border_diff[i] = std::abs(border[i + 1] - border[i]);
    }

    for (size_t i = 0; i < border_diff.size(); ++i) {
        if (border_diff[i] > thresh_1) {
            return true;
        }
    }

    return false;
}

std::vector<int> extractSkyline(cv::InputArray src, const Params &params)
{
    cv::Mat gradient;
    calcImageGradient(src, gradient);

    const auto n = static_cast<int>(std::floor(
                       (params.f_thres_sky_max - params.f_thres_sky_min) /
                       params.f_thres_sky_search_step)) +
                   1;

    // FIXME: Give it a proper name
    const auto aaa =
        params.f_thres_sky_min +
        (std::floor((params.f_thres_sky_max - params.f_thres_sky_min) / n) -
         1.);

    std::vector<int> bestSkyline;
    auto maxEnergy{0.};
    for (int k = 1; k < n + 1; k++) {
        double t = aaa * (k - 1);
        std::vector<int> skyline = extractSkyline(gradient, t);

        const auto energy = calcSkyRegionEnergy(skyline, src);
        if (energy < 0.) {
            LOG(INFO) << "Jn is -inf";
            continue;
        }

        if (energy > maxEnergy) {
            maxEnergy = energy;
            bestSkyline = skyline;
        }
    }

    return bestSkyline;
}

std::vector<int> refineSkyline(const std::vector<int> &skyline,
                               cv::InputArray _src)
{
    cv::Mat skyMask, groundMask;
    separateImage(_src, skyline, skyMask, groundMask);

    cv::Mat skyRegion, groundRegion;
    _src.copyTo(skyRegion, skyMask);
    _src.copyTo(groundRegion, groundMask);

    std::vector<cv::Vec3b> groundRegionValues;
    {
        std::vector<cv::Point> indices;
        cv::findNonZero(groundMask, indices);

        groundRegionValues.reserve(indices.size());
        for (const auto &index : indices) {
            groundRegionValues.push_back(groundRegion.at<cv::Vec3b>(index));
        }
    }

    std::vector<cv::Vec3b> skyRegionValues;
    {
        std::vector<cv::Point> indices;
        cv::findNonZero(skyMask, indices);

        skyRegionValues.reserve(indices.size());
        for (const auto &index : indices) {
            skyRegionValues.push_back(skyMask.at<cv::Vec3b>(index));
        }
    }

    cv::Mat skyRegionValues_f;
    cv::Mat{skyRegionValues}.reshape(1).convertTo(skyRegionValues_f, CV_32FC1);
    std::vector<int> labels;
    cv::kmeans(skyRegionValues_f, 2, labels,
               cv::TermCriteria{
                   cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.},
               10, cv::KMEANS_RANDOM_CENTERS);

    std::vector<cv::Vec3b> sky_label_0_image, sky_label_1_image;
    for (size_t i{0}; i < labels.size(); ++i) {
        const auto &val = skyRegionValues[i];
        if (labels[i] == 0) {
            sky_label_0_image.push_back(val);
        }
        else {
            sky_label_1_image.push_back(val);
        }
    }

    cv::Mat sky_covar_1, sky_mean_1;
    cv::calcCovarMatrix(sky_label_1_image, sky_covar_1, sky_mean_1,
                        cv::COVAR_ROWS | cv::COVAR_NORMAL | cv::COVAR_SCALE);
    cv::Mat ic_s1;
    cv::invert(sky_covar_1, ic_s1, cv::DECOMP_SVD);

    cv::Mat sky_covar_0, sky_mean_0;
    cv::calcCovarMatrix(sky_label_0_image, sky_covar_0, sky_mean_0,
                        cv::COVAR_ROWS | cv::COVAR_NORMAL | cv::COVAR_SCALE);
    cv::Mat ic_s0;
    cv::invert(sky_covar_0, ic_s0, cv::DECOMP_SVD);

    cv::Mat ground_covar, ground_mean;
    cv::calcCovarMatrix(groundRegionValues, ground_covar, ground_mean,
                        cv::COVAR_ROWS | cv::COVAR_NORMAL | cv::COVAR_SCALE);
    cv::Mat ic_g;
    cv::invert(ground_covar, ic_g, cv::DECOMP_SVD);

    cv::Mat sky_mean, sky_covar;
    cv::Mat ic_s;
    if (cv::Mahalanobis(sky_mean_0, ground_mean, ic_s0) >
        cv::Mahalanobis(sky_mean_1, ground_mean, ic_s1)) {
        sky_mean = sky_mean_0;
        sky_covar = sky_covar_0;
        ic_s = ic_s0;
    }
    else {
        sky_mean = sky_mean_1;
        sky_covar = sky_covar_1;
        ic_s = ic_s1;
    }

    const auto src = _src.getMat();

    std::vector<int> refineSkyline(skyline.size(), 0);
    for (size_t col = 0; col < skyline.size(); ++col) {
        double cnt = 0.0;
        for (int row = 0; row < skyline[col]; ++row) {
            cv::Mat ori_pix;
            src.row(row)
                .col(static_cast<int>(col))
                .convertTo(ori_pix, sky_mean.type());
            ori_pix = ori_pix.reshape(1, 1);

            double distance_s = cv::Mahalanobis(ori_pix, sky_mean, ic_s);
            double distance_g = cv::Mahalanobis(ori_pix, ground_mean, ic_g);

            if (distance_s < distance_g) {
                cnt++;
            }
        }

        refineSkyline[col] = cnt < (skyline[col] / 2) ? 0 : skyline[col];
    }

    return refineSkyline;
}

bool extractSkyMask(cv::InputArray src, cv::OutputArray mask,
                    const Params &params)
{
    std::vector<int> skyline = extractSkyline(src, params);

    if (!checkSkyExist(skyline, src.rows() / 30., src.rows() / 4., 2)) {
        return false;
    }

    if (checkFalsePositiveSky(skyline, src.cols() / 3.)) {
        skyline = refineSkyline(skyline, src);
    }

    cv::Mat _;
    separateImage(src, skyline, mask, _);

    return true;
}

} // namespace internal

void detect(cv::InputArray image, cv::OutputArray skyMask, const Params &params,
            cv::OutputArray _viz)
{
    internal::extractSkyMask(image, skyMask, params);

    if (_viz.needed()) {
        cv::addWeighted(image, 1., skyMask, 1., 0., _viz);
    }
}

} // namespace SkyAreaDetector

} // namespace tl
