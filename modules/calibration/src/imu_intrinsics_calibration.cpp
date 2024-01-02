#include "imu_intrinsics_calibration.h"

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <json/json.hpp>

#include <AxMath/MathBase>
#include <AxImu/ImuIntegration>

namespace thoht {

struct AccSampleResidual
{
    const Eigen::Vector3d acc_;
    const double g_;

    AccSampleResidual(const Eigen::Vector3d& acc, double gravity)
        : acc_(acc), g_(gravity)
    {
    }

    template <typename T>
    bool operator()(const T* const params, T* residuals) const
    {
        auto acc = acc_.template cast<T>();
        // Assume body frame same as accelerometer frame, so bottom left
        // params in the misalignment matrix are set to zero
        ImuIntrinsics_<T> intrinsics{
            params[0], params[1], params[2], T(0),      T(0),      T(0),
            params[3], params[4], params[5], params[6], params[7], params[8]};

        auto corrected_sample = intrinsics.unbiasNormalize(acc);
        residuals[0] = T(g_) - corrected_sample.norm();

        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d& sample,
                                       double gravity)
    {
        constexpr int kResidualSize{1};
        constexpr int kParameterSize = imu::kParameterSize - 3;
        return new ceres::AutoDiffCostFunction<AccSampleResidual, kResidualSize,
                                               kParameterSize>(
            new AccSampleResidual(sample, gravity));
    }
};

struct GyroSamplesResidual
{
    const Eigen::Vector3d g_start_, g_end_;
    const ImuReadings gyros_;
    const DataInterval interval_;
    const double dt_;
    const bool optimize_bias_;

    GyroSamplesResidual(const Eigen::Vector3d& g_start,
                        const Eigen::Vector3d& g_end, const ImuReadings& gyros,
                        const DataInterval& interval, double dt,
                        bool optimize_bias)
        : g_start_(g_start),
          g_end_(g_end),
          gyros_(gyros),
          interval_(interval),
          dt_(dt),
          optimize_bias_(optimize_bias)
    {
    }

    template <typename T>
    bool operator()(const T* const params, T* residuals) const
    {
        ImuIntrinsics_<T> intrinsics{params[0],
                                     params[1],
                                     params[2],
                                     params[3],
                                     params[4],
                                     params[5],
                                     params[6],
                                     params[7],
                                     params[8],
                                     optimize_bias_ ? params[9] : T(0),
                                     optimize_bias_ ? params[10] : T(0),
                                     optimize_bias_ ? params[11] : T(0)};

        ImuReadings_<T> samples;
        samples.reserve(interval_.size());
        for (int i{interval_.start}; i <= interval_.end; i++) {
            auto gyro = gyros_[i].data().template cast<T>();
            samples.d().emplace_back(gyros_[i].timestamp(),
                                     intrinsics.unbiasNormalize(gyro));
        }

        Eigen::Matrix3<T> rmat;
        IntegrateGyroInterval(samples, rmat, dt_);

        auto diff = rmat.transpose() * g_start_.template cast<T>() -
                    g_end_.template cast<T>();

        residuals[0] = diff(0);
        residuals[1] = diff(1);
        residuals[2] = diff(2);
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d& gravityStart,
                                       const Eigen::Vector3d& gravityEnd,
                                       const ImuReadings& gyroSamples,
                                       const DataInterval& interval, double dt,
                                       bool optimizeBias)
    {
        constexpr int kResidualSize{3};
        if (optimizeBias) {
            return new ceres::AutoDiffCostFunction<
                GyroSamplesResidual, kResidualSize, imu::kParameterSize>(
                new GyroSamplesResidual(gravityStart, gravityEnd, gyroSamples,
                                        interval, dt, optimizeBias));
        }

        return (
            new ceres::AutoDiffCostFunction<GyroSamplesResidual, kResidualSize,
                                            imu::kParameterSize -
                                                imu::kBiasSize>(
                new GyroSamplesResidual(gravityStart, gravityEnd, gyroSamples,
                                        interval, dt, optimizeBias)));
    }
};

///------- ImuIntrinsicsCalibration::Impl starts from here
class ImuIntrinsicsCalibration::Impl
{
public:
    Impl();

    void init(const Options& options);

public:
    Options m_opts;
    std::vector<DataInterval> m_minCostStaticIntervals;
    ImuIntrinsics m_initAcclIntrinsics, m_initGyroIntrinsics;
    ImuIntrinsics m_acclIntrinsics, m_gyroIntrinsics;
    AccDatas m_calibAcclSamples;
    GyroDatas m_calibGyroSamples;
};

ImuIntrinsicsCalibration::Impl::Impl() {}

void ImuIntrinsicsCalibration::Impl::init(const Options& options)
{
    m_opts = options;
}

///------- ImuIntrinsicsCalibration starts from here
ImuIntrinsicsCalibration::ImuIntrinsicsCalibration(const Options& options)
    : d(std::make_unique<Impl>())
{
    d->init(options);
}

ImuIntrinsicsCalibration::~ImuIntrinsicsCalibration() = default;

const ImuIntrinsicsCalibration::Options& ImuIntrinsicsCalibration::options()
    const
{
    return d->m_opts;
}

ImuIntrinsicsCalibration::Options& ImuIntrinsicsCalibration::rOptions()
{
    return d->m_opts;
}

void ImuIntrinsicsCalibration::setInitAcclIntrinsics(
    const ImuIntrinsics& params)
{
    d->m_initAcclIntrinsics = params;
}

const ImuIntrinsics& ImuIntrinsicsCalibration::initAcclIntrinsics() const
{
    return d->m_initAcclIntrinsics;
}

void ImuIntrinsicsCalibration::setInitGyroIntrinsics(
    const ImuIntrinsics& params)
{
    d->m_initGyroIntrinsics = params;
}

const ImuIntrinsics& ImuIntrinsicsCalibration::initGyroIntrinsics() const
{
    return d->m_initGyroIntrinsics;
}

bool ImuIntrinsicsCalibration::calibAccelerometer(const AccDatas& acclSamples)
{
    LOG(INFO) << "Calibrating accelerometer...";

    d->m_calibAcclSamples.clear();
    d->m_minCostStaticIntervals.clear();

    const size_t sampleCount = acclSamples.size();

    const auto initInterval = DataInterval::initialInterval(
        acclSamples, d->m_opts.initStaticDuration);

    Eigen::Vector3d acclMean = acclSamples.mean(initInterval);
    Eigen::Vector3d::Index maxIdx;
    acclMean.maxCoeff(&maxIdx);
    acclMean[maxIdx] -= d->m_opts.gravity;
    d->m_initAcclIntrinsics.setBias(acclMean);
    LOG(INFO) << "Setting initial accelerometer bias: "
              << d->m_initAcclIntrinsics.bias().transpose();

    Eigen::Vector3d acclVar = acclSamples.variance(initInterval);
    const double acclThreshold = acclVar.norm();

    double minCost = math::kMaxDouble;
    int minCostFactor = -1;
    std::vector<double> minCostIntrinsics;
    for (int thresFactor{1}; thresFactor <= 10; thresFactor++) {
        std::vector<double> intrinsics(9);
        intrinsics[0] = d->m_initAcclIntrinsics.misYZ();
        intrinsics[1] = d->m_initAcclIntrinsics.misZY();
        intrinsics[2] = d->m_initAcclIntrinsics.misZX();
        intrinsics[3] = d->m_initAcclIntrinsics.scaleX();
        intrinsics[4] = d->m_initAcclIntrinsics.scaleY();
        intrinsics[5] = d->m_initAcclIntrinsics.scaleZ();
        intrinsics[6] = d->m_initAcclIntrinsics.biasX();
        intrinsics[7] = d->m_initAcclIntrinsics.biasY();
        intrinsics[8] = d->m_initAcclIntrinsics.biasZ();

        const auto staticIntervals =
            acclSamples.extractStaticIntervals(acclThreshold * thresFactor);

        std::vector<DataInterval> extractedIntervals;
        const auto staticSamples = acclSamples.extractSamples(
            staticIntervals, extractedIntervals,
            d->m_opts.minIntervalSampleCount, d->m_opts.useMeanAcc);

        LOG(INFO) << "Extracted " << extractedIntervals.size()
                  << " static intervals using factor " << thresFactor << " -> "
                  << acclThreshold;

        // TODO: Perform a quality test
        if (extractedIntervals.size() < d->m_opts.minStaticIntervalCount) {
            LOG(WARNING)
                << "Not enough static intervals, calibration is not possible.";
            continue;
        }

        ceres::Problem problem;
        for (const auto& sample : staticSamples.d()) {
            problem.AddResidualBlock(
                AccSampleResidual::create(sample.data(), d->m_opts.gravity),
                nullptr, intrinsics.data());
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (summary.final_cost < minCost) {
            minCost = summary.final_cost;
            minCostFactor = thresFactor;
            d->m_minCostStaticIntervals = staticIntervals;
            minCostIntrinsics = intrinsics;
        }

        LOG(INFO) << "Accelerometer residual " << summary.final_cost;
    }

    if (minCostFactor < 0) {
        LOG(ERROR) << "Failed to calibrate with current data.";
        return false;
    }

    d->m_acclIntrinsics = {minCostIntrinsics[0],
                           minCostIntrinsics[1],
                           minCostIntrinsics[2],
                           0.,
                           0.,
                           0.,
                           minCostIntrinsics[3],
                           minCostIntrinsics[4],
                           minCostIntrinsics[5],
                           minCostIntrinsics[6],
                           minCostIntrinsics[7],
                           minCostIntrinsics[8]};

    // Calibrate the input accelerometer data with the obtained calibration
    d->m_calibAcclSamples.reserve(sampleCount);
    for (const auto& sample : acclSamples.d()) {
        d->m_calibAcclSamples.d().emplace_back(
            sample.timestamp(),
            d->m_acclIntrinsics.unbiasNormalize(sample.data()));
    }

    LOG(INFO) << "Accelerometer misalignment matrix: "
                 "\n"
              << d->m_acclIntrinsics.misalignmentMatrix()
              << "\n"
                 "Accelerometer scale matrix: "
                 "\n"
              << d->m_acclIntrinsics.scaleMatrix()
              << "\n"
                 "Accelerometer bias: "
                 "\n"
              << d->m_acclIntrinsics.bias().transpose()
              << "\n"
                 "Accelerometer inverse scale factors: "
              << 1.0 / d->m_acclIntrinsics.scaleX() << " "
              << 1.0 / d->m_acclIntrinsics.scaleY() << " "
              << 1.0 / d->m_acclIntrinsics.scaleZ();

    return true;
}

bool ImuIntrinsicsCalibration::calibAccelerometerGyroscope(
    const AccDatas& acclSamples, const GyroDatas& gyroSamples)
{
    if (!calibAccelerometer(acclSamples)) {
        LOG(ERROR) << "Failed to calibrate accelerometer.";
        return false;
    }

    LOG(INFO) << "Calibrating gyroscope...";

    d->m_calibGyroSamples.clear();

    const size_t sampleCount = gyroSamples.size();

    // Compute the gyroscopes biases in the (static) initialization interval
    const auto initStaticGyroInterval = DataInterval::initialInterval(
        gyroSamples, d->m_opts.initStaticDuration);
    Eigen::Vector3d gyroBias = gyroSamples.mean(initStaticGyroInterval);

    // Remove the bias
    {
        ImuIntrinsics gyroIntrinsics{0., 0.,          0.,          0.,
                                     0., 0.,          1.,          1.,
                                     1., gyroBias(0), gyroBias(1), gyroBias(2)};
        d->m_calibGyroSamples.reserve(sampleCount);
        for (size_t i{0}; i < sampleCount; i++) {
            d->m_calibGyroSamples.d().emplace_back(
                gyroSamples[i].timestamp(),
                gyroIntrinsics.unbias(gyroSamples[i].data()));
        }
    }

    std::vector<DataInterval> extractedIntervals;
    const ImuReadings staticAcclMeans = d->m_calibAcclSamples.extractSamples(
        d->m_minCostStaticIntervals, extractedIntervals,
        d->m_opts.minIntervalSampleCount, true);
    const size_t staticPoseCount = staticAcclMeans.size();

    auto gyroIntrinsics = d->m_initGyroIntrinsics.toVector();
    // Bias has been estimated and removed in the initialization period
    gyroIntrinsics[9] = 0.0;
    gyroIntrinsics[10] = 0.0;
    gyroIntrinsics[11] = 0.0;

    ceres::Problem problem;
    for (int i = 0, tsIdx = 0; i < staticPoseCount - 1; i++) {
        Eigen::Vector3d currGVersor = staticAcclMeans[i].data().normalized();
        Eigen::Vector3d nextGVersor =
            staticAcclMeans[i + 1].data().normalized();

        double ts0 =
            d->m_calibAcclSamples[extractedIntervals[i].end].timestamp();
        double ts1 =
            d->m_calibAcclSamples[extractedIntervals[i + 1].start].timestamp();

        // Assume monotone signal time
        int start{-1};
        int end{-1};
        for (; tsIdx < sampleCount; tsIdx++) {
            if (start < 0) {
                if (d->m_calibGyroSamples[tsIdx].timestamp() >= ts0) {
                    start = tsIdx;
                }
            }
            else {
                if (d->m_calibGyroSamples[tsIdx].timestamp() >= ts1) {
                    end = tsIdx - 1;
                    break;
                }
            }
        }

        problem.AddResidualBlock(
            GyroSamplesResidual::create(
                currGVersor, nextGVersor, d->m_calibGyroSamples,
                DataInterval{start, end}, d->m_opts.gyroDataPeriod,
                d->m_opts.optimizeGyroBias),
            nullptr, gyroIntrinsics.data());
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    d->m_gyroIntrinsics = {gyroIntrinsics[0],
                           gyroIntrinsics[1],
                           gyroIntrinsics[2],
                           gyroIntrinsics[3],
                           gyroIntrinsics[4],
                           gyroIntrinsics[5],
                           gyroIntrinsics[6],
                           gyroIntrinsics[7],
                           gyroIntrinsics[8],
                           gyroBias(0) + gyroIntrinsics[9],
                           gyroBias(1) + gyroIntrinsics[10],
                           gyroBias(2) + gyroIntrinsics[11]};

    // Calibrate the input gyroscopes data with the obtained calibration
    for (const auto& sample : gyroSamples.d()) {
        d->m_calibGyroSamples.d().emplace_back(
            sample.timestamp(),
            d->m_gyroIntrinsics.unbiasNormalize(sample.data()));
    }

    LOG(INFO) << summary.FullReport();

    return true;
}

const ImuIntrinsics& ImuIntrinsicsCalibration::acclIntrinsics() const
{
    return d->m_acclIntrinsics;
}
const ImuIntrinsics& ImuIntrinsicsCalibration::gyroIntrinsics() const
{
    return d->m_gyroIntrinsics;
}

const AccDatas& ImuIntrinsicsCalibration::calibratedAcclSamples() const
{
    return d->m_calibAcclSamples;
}

const GyroDatas& ImuIntrinsicsCalibration::calibratedGyroSamples() const
{
    return d->m_calibGyroSamples;
}

namespace key {
constexpr char kScope[]{"imu_intrinsics_calibration"};
constexpr char kGravity[]{"gravity"};
constexpr char kInitStaticDuration[]{"initial_static_duration"};
constexpr char kGyroDataPeriod[]{"gyroscope_dt"};
constexpr char kMinIntervalSampleCount[]{"min_interval_sample_count"};
constexpr char kMinStaticIntervalCount[]{"min_static_intervals"};
constexpr char kUseMeanAcc[]{"use_mean_acc"};
constexpr char kOptimizeGyroBias[]{"optimize_gyro_bias"};
} // namespace key

bool ImuIntrinsicsCalibration::setFromJson(const std::string& json)
{
    //
    return false;
}

void ImuIntrinsicsCalibration::toJsonString(std::string& json) const
{
    nlohmann::json j;
    j[key::kScope][key::kGravity] = d->m_opts.gravity;
    j[key::kScope][key::kInitStaticDuration] = d->m_opts.initStaticDuration;
    j[key::kScope][key::kGyroDataPeriod] = d->m_opts.gyroDataPeriod;
    j[key::kScope][key::kMinIntervalSampleCount] =
        d->m_opts.minIntervalSampleCount;
    j[key::kScope][key::kMinStaticIntervalCount] =
        d->m_opts.minStaticIntervalCount;
    j[key::kScope][key::kUseMeanAcc] = d->m_opts.useMeanAcc;
    j[key::kScope][key::kOptimizeGyroBias] = d->m_opts.optimizeGyroBias;

    // TODO: toString???
}

} // namespace thoht
