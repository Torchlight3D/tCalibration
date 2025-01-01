#include "StereoCameraVerificationTask.h"

#include <format>

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include <tabulate/tabulate.hpp>

#include <tCalib/StereoCameraVerifyScene>
#include <tCamera/CameraAdaptor>
#include <tCore/StringUtils>
#include <tVision/BlurDetection>

#include "stereorectify.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {

template <typename T>
inline bool all_less(const std::vector<T> &vals, T max)
{
    return std::ranges::all_of(vals, [&max](const auto &v) { return v < max; });
}

std::string join(const std::vector<std::string> &parts,
                 const std::string &separator = ", ",
                 const std::string &concluder = {})
{
    if (parts.empty()) {
        return concluder;
    }

    auto first = parts.begin();
    auto last = parts.end();

    std::stringstream ss;
    ss << *first;
    ++first;

    while (first != last) {
        ss << separator;
        ss << *first;
        ++first;
    }

    ss << concluder;

    return ss.str();
}

} // namespace

///------- StereoCameraVerificationTask::Summary starts from here
bool StereoCameraVerificationTask::Summary::passed() const
{
    const auto appliedError =
        (errors | notAppliedErrors()) ^ notAppliedErrors();
    return appliedError == Error::None;
}

///------- StereoCameraVerificationTask::Impl starts from here
class StereoCameraVerificationTask::Impl
{
public:
    explicit Impl(const StereoCameraCalibResult &data);

    double calcSharpness(cv::InputArray img) const;
    double calcStereoRectifyRpe(cv::InputArray left, cv::InputArray right,
                                cv::OutputArray viz = cv::noArray()) const;

public:
    const StereoCameraCalibResult &_data;
    StereoCameraVerifyScene::Ptr _scene;
};

StereoCameraVerificationTask::Impl::Impl(const StereoCameraCalibResult &data)
    : _data(data)
{
}

double StereoCameraVerificationTask::Impl::calcSharpness(
    cv::InputArray img) const
{
    return FrequencyDomainEntropyBlurDetection{}.evaluate(img);
}

double StereoCameraVerificationTask::Impl::calcStereoRectifyRpe(
    cv::InputArray img0, cv::InputArray img1, cv::OutputArray viz) const
{
    const auto cam0 = fromCocCamera(_data.cameras[CameraLeft]);
    const auto cam1 = fromCocCamera(_data.cameras[CameraRight]);

    cv::Mat map1_left, map2_left, map1_right, map2_right;
    const auto inited =
        initUndistortRectifyMap(*cam0, *cam1, _data.R_c1c0, _data.t_c1c0,
                                map1_left, map2_left, map1_right, map2_right);

    if (!inited) {
        LOG(ERROR) << "Failed to calculate undistortion map.";
        return 0.;
    }

    cv::Mat leftRectified, rightRectified;
    cv::remap(img0, leftRectified, map1_left, map2_left, cv::INTER_NEAREST);
    cv::remap(img1, rightRectified, map1_right, map2_right, cv::INTER_NEAREST);

    std::vector<cv::Point2d> imgPoints0, imgPoints1;
    cv::Mat viz10_11;
    const auto detected = _scene->detect(leftRectified, rightRectified,
                                         imgPoints0, imgPoints1, viz10_11);

    if (!detected) {
        LOG(ERROR) << "Failed to find target from rectified stereo images.";
        return 0.;
    }

    cv::Mat mean;
    cv::reduce(cv::Mat{imgPoints1}.reshape(1) - cv::Mat{imgPoints0}.reshape(1),
               mean, 0, cv::REDUCE_AVG);
    const auto diffYMean = cv::Point2d(mean).y;

    return diffYMean;
}

///------- StereoCameraVerificationTask starts from here
StereoCameraVerificationTask::StereoCameraVerificationTask(
    const StereoCameraCalibResult &data,
    std::unique_ptr<StereoCameraVerifyScene> scene)
    : d(std::make_unique<Impl>(data))
{
    d->_scene = std::move(scene);
}

StereoCameraVerificationTask::~StereoCameraVerificationTask() = default;

StereoCameraVerificationTask::Summary StereoCameraVerificationTask::verify(
    cv::InputArray _left, cv::InputArray _right, const Reference &ref,
    cv::OutputArray _viz)
{
    using Error = StereoCameraVerificationTask::Summary::Error;

    Summary summary;

    /// 0. Check sample and verify data
    if (!d->_scene) {
        summary.errors = Error::InvalidConfigs;
        LOG(ERROR) << "Failed to verify:"
                      "Invalid configurations.";
        return summary;
    }
    if (_left.empty() || _right.empty()) {
        summary.errors = Error::InvalidImage;
        LOG(ERROR) << "Failed to verify:"
                      "Empty input images.";
        return summary;
    }

    /// Five visualization in 2x3 layout
    /// viz00: Left detection + right to left reprojections
    /// viz01: Right detection + epipolar error + triangulation error
    /// viz02: Relative size diff statistic
    /// viz10: Stereo rectify left
    /// viz11: Stereo reciify right
    /// viz12: None, black placeholder

    /// 1. Detect targets
    cv::Mat viz00, viz01;
    if (!d->_scene->detect(_left, _right, d->_data.cameras[CameraLeft],
                           d->_data.cameras[CameraRight], viz00, viz01)) {
        summary.errors = Error::FailedDetectTarget;
        LOG(ERROR) << "Failed to verify:"
                      "Failed to detect targets, or bad detections.";
        return summary;
    }

    /// 2. Test image quality
    const auto sharpness0 = d->calcSharpness(_left);
    const auto sharpness1 = d->calcSharpness(_right);
    summary.sharpness = {sharpness0, sharpness1};
    if (std::isless(sharpness0, ref.minSharpness) ||
        std::isless(sharpness1, ref.minSharpness)) {
        summary.errors |= Error::BlurryImage;
    }

    /// 3. Test Tracker criteria
    const auto trackerSummaries =
        d->_scene->solveTracker(d->_data, ref.tracker, viz01);
    for (const auto &summ : trackerSummaries) {
        summary.epipolarErrorMax.push_back(summ.epipolarErrorMax);
        summary.epipolarErrorMean.push_back(summ.epipolarErrorMean);
    }

    if (!all_less(summary.epipolarErrorMax, ref.tracker.maxEpipolarErrorMax)) {
        summary.errors |= Error::LargeEpipolarErrorMax;
    }
    if (!all_less(summary.epipolarErrorMean,
                  ref.tracker.maxEpipolarErrorMean)) {
        summary.errors |= Error::LargeEpipolarErrorMean;
    }

    /// 4. Test Estimator criteria
    cv::Mat viz02;
    _left.copyTo(viz02);
    const auto estimatorSummaries =
        d->_scene->solveEstimator(d->_data, ref.estimator, viz01, viz02);
    std::vector<double> Qs;
    for (const auto &summ : estimatorSummaries) {
        summary.relativeSizeFailureRate.push_back(summ.relativeSizeRejectRate);
        summary.relativeSizeDiffMedian.push_back(summ.relativeSizeDiffMedian);
        Qs.push_back(summ.shortDistanceRelativeSizeDiffQ1);
        Qs.push_back(summ.shortDistanceRelativeSizeDiffQ3);
    }
    std::sort(Qs.begin(), Qs.end());
    summary.shortDistanceRelativeSizeDiffIqr = Qs.back() - Qs.front();

    if (!all_less(summary.relativeSizeFailureRate,
                  ref.estimator.maxRelativeSizeFailureRate)) {
        summary.errors |= Error::PoorRelativeSizeEsitmation;
    }
    if (!all_less(summary.relativeSizeDiffMedian,
                  ref.estimator.maxRelativeSizeDiffMedian)) {
        summary.errors |= Error::LargeRelativeSizeDiffMedian;
    }
    if (!std::isless(summary.shortDistanceRelativeSizeDiffIqr,
                     ref.estimator.maxShortDistanceRelativeSizeDiffIqr)) {
        summary.errors |= Error::LargeShortDistanceRelativeSizeDiffIqr;
    }

    /// 5. Reproject back and forth
    const auto reprojSummaries =
        d->_scene->solveReprojections(d->_data, ref.reprojection, viz00);
    for (const auto &summ : reprojSummaries) {
        summary.rightToLeftRpeMedian.push_back(summ.rightToLeftRpeMedian);
    }

    if (!all_less(summary.rightToLeftRpeMedian,
                  ref.reprojection.maxRightToLeftErrorMedian)) {
        summary.errors |= Error::LargeRightToLeftRpeMedian;
    }

    /// 6. Test stereo rectification
    cv::Mat viz10_viz11;
    summary.stereoRectifyDiffYMean =
        d->calcStereoRectifyRpe(_left, _right, viz10_viz11);
    if (!std::isless(std::abs(summary.stereoRectifyDiffYMean),
                     ref.maxStereoMatchingDiffY)) {
        summary.errors |= Error::LargeStereoMatchingDiffY;
    }

    if (_viz.needed()) {
        const cv::Mat black{_left.size(), CV_8UC3, cv::Scalar::all(0)};

        cv::Mat row0, row1;
        cv::hconcat(std::vector{viz00, viz01, viz02}, row0);
        cv::hconcat(std::vector{viz10_viz11, black}, row1);
        cv::Mat merged;
        cv::vconcat(std::vector{row0, row1}, _viz);
    }

    return summary;
}

StereoCameraVerificationTask::Summary::Error
StereoCameraVerificationTask::notAppliedErrors()
{
    using Error = StereoCameraVerificationTask::Summary::Error;
    return Error::BlurryImage;
}

std::ostream &operator<<(std::ostream &os,
                         const StereoCameraVerificationTask::Summary &summ)
{
    using namespace tabulate;
    using Row_t = Table::Row_t;

    // NOTE: Format after adding data
    Table table;
    table.add_row(Row_t{"Items ", "Value"});

    {
        std::vector<std::string> strs;
        for (const auto &v : summ.sharpness) {
            strs.push_back(std::to_string(v));
        }
        table.add_row(Row_t{"Sharpness", std::format("[{}]", join(strs, ","))});
    }
    {
        std::vector<std::string> strs;
        for (const auto &v : summ.epipolarErrorMax) {
            strs.push_back(std::to_string(v));
        }
        table.add_row(
            Row_t{"Epipolar Error Max", std::format("[{}]", join(strs, ","))});
    }

    {
        std::vector<std::string> strs;
        for (const auto &v : summ.epipolarErrorMean) {
            strs.push_back(std::to_string(v));
        }
        table.add_row(
            Row_t{"Epipolar Error Mean", std::format("[{}]", join(strs, ","))});
    }

    {
        std::vector<std::string> strs;
        for (const auto &v : summ.relativeSizeFailureRate) {
            strs.push_back(str::percentage(v, 2));
        }
        table.add_row(Row_t{"Relative Size Failure Rate",
                            std::format("[{}]", join(strs, ","))});
    }

    {
        std::vector<std::string> strs;
        for (const auto &v : summ.relativeSizeDiffMedian) {
            strs.push_back(str::permille(v, 2));
        }
        table.add_row(Row_t{"Relative Size Diff Median",
                            std::format("[{}]", join(strs, ","))});
    }

    table.add_row(
        Row_t{"Short Distance Relative Size Error IQR",
              str::permille(summ.shortDistanceRelativeSizeDiffIqr, 2)});

    {
        std::vector<std::string> strs;
        for (const auto &v : summ.rightToLeftRpeMedian) {
            strs.push_back(std::to_string(v));
        }
        table.add_row(Row_t{"Right To Left RPE Median",
                            std::format("[{}]", join(strs, ","))});
    }

    table.add_row(Row_t{"Stereo Rectify Error Mean in Y",
                        std::to_string(summ.stereoRectifyDiffYMean)});

    // Fomat table
    table.column(0).format().font_align(FontAlign::left);
    table.column(1).format().font_align(FontAlign::center);

    os << "Verification Summary: "
          "\n"
       << table << "\n";
    return os;
}

namespace key {
// Reference
constexpr char kMinSharpness[]{"min_sharpness"};
constexpr char kTracker[]{"tracker"};
constexpr char kEstimator[]{"estimator"};
constexpr char kReprojections[]{"reprojections"};
constexpr char kMaxStereoMatchingDiffY[]{"max_stereo_matching_diff_y"};
// Summary
constexpr char kSharpness[]{"sharpness"};
constexpr char kEpipolarErrorMax[]{"epipolar_err_max"};
constexpr char kEpipolarErrorMean[]{"epipolar_err_mean"};
constexpr char kRelativeSizeFailureRate[]{"rel_size_failure_rate"};
constexpr char kRelativeSizeDiffMedian[]{"rel_size_diff_med"};
constexpr char kShortDistanceRelativeSizeDiffIqr[]{
    "short_dist_rel_size_diff_iqr"};
constexpr char kRightToLeftRpeMedian[]{"right_to_left_rpe_med"};
constexpr char kStereoRectifyDiffY[]{"stereo_matching_diff_y"};
} // namespace key

} // namespace tl

namespace nlohmann {

using namespace tl;

void to_json(json &j, const tl::StereoCameraVerificationTask::Reference &ref)
{
    j[key::kMinSharpness] = ref.minSharpness;
    j[key::kTracker] = ref.tracker;
    j[key::kEstimator] = ref.estimator;
    j[key::kReprojections] = ref.reprojection;
    j[key::kMaxStereoMatchingDiffY] = ref.maxStereoMatchingDiffY;
}

void from_json(const json &j, tl::StereoCameraVerificationTask::Reference &ref)
{
    j.at(key::kMinSharpness).get_to(ref.minSharpness);
    j.at(key::kTracker).get_to(ref.tracker);
    j.at(key::kEstimator).get_to(ref.estimator);
    j.at(key::kReprojections).get_to(ref.reprojection);
    j.at(key::kMaxStereoMatchingDiffY).get_to(ref.maxStereoMatchingDiffY);
}

void to_json(json &j, const tl::StereoCameraVerificationTask::Summary &summary)
{
    j[key::kSharpness] = summary.sharpness;
    j[key::kEpipolarErrorMax] = summary.epipolarErrorMax;
    j[key::kEpipolarErrorMean] = summary.epipolarErrorMean;
    j[key::kRelativeSizeFailureRate] = summary.relativeSizeFailureRate;
    j[key::kRelativeSizeDiffMedian] = summary.relativeSizeDiffMedian;
    j[key::kShortDistanceRelativeSizeDiffIqr] =
        summary.shortDistanceRelativeSizeDiffIqr;
    j[key::kRightToLeftRpeMedian] = summary.rightToLeftRpeMedian;
    j[key::kStereoRectifyDiffY] = summary.stereoRectifyDiffYMean;
}

void from_json(const json &j,
               tl::StereoCameraVerificationTask::Summary &summary)
{
    j.at(key::kSharpness).get_to(summary.sharpness);
    j.at(key::kEpipolarErrorMax).get_to(summary.epipolarErrorMax);
    j.at(key::kEpipolarErrorMean).get_to(summary.epipolarErrorMean);
    j.at(key::kRelativeSizeFailureRate).get_to(summary.relativeSizeFailureRate);
    j.at(key::kRelativeSizeDiffMedian).get_to(summary.relativeSizeDiffMedian);
    j.at(key::kShortDistanceRelativeSizeDiffIqr)
        .get_to(summary.shortDistanceRelativeSizeDiffIqr);
    j.at(key::kRightToLeftRpeMedian).get_to(summary.rightToLeftRpeMedian);
    j.at(key::kStereoRectifyDiffY).get_to(summary.stereoRectifyDiffYMean);
}

} // namespace nlohmann
