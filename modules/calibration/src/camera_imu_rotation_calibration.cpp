#include "camera_imu_rotation_calibration.h"

#include <glog/logging.h>
#include <json/json.hpp>

#include <AxMath/MathBase>
#include <AxMath/EigenUtils>
#include <AxImu/ImuData>

namespace thoht {

namespace {

class MovingAverage
{
public:
    explicit MovingAverage(size_t capacity)
        : capacity_(capacity), values_(new double[capacity])
    {
        assert(capacity >= 1);
    }
    ~MovingAverage() { delete[] values_; }

    void add(double val)
    {
        if (!initialized()) {
            head_ = values_;
            *head_ = val;
            tail_ = head_;
            inc(tail_);
            total_ = val;
            return;
        }

        if (filled()) {
            total_ -= *head_;
            inc(head_);
        }

        *tail_ = val;
        inc(tail_);

        total_ += val;
    }

    double average() const
    {
        auto size = this->size();
        if (size == 0) {
            return 0.;
        }
        return total_ / static_cast<double>(size);
    }

private:
    void inc(double*& p)
    {
        if (++p >= values_ + capacity_) {
            p = values_;
        }
    }

    ptrdiff_t size() const
    {
        if (!initialized()) {
            return 0;
        }

        if (filled()) {
            return capacity_;
        }

        return (capacity_ + tail_ - head_) % capacity_;
    }

    bool initialized() const { return head_; }
    bool filled() const { return head_ == tail_; }

private:
    size_t capacity_;
    double* const values_;  //! values in the window
    double* head_{nullptr}; //! point to the earliest element
    double* tail_{nullptr}; //! point to the latest element
    double total_{0.};
};

} // namespace

///------- CameraImuRotationCalibration::Impl starts from here
class CameraImuRotationCalibration::Impl
{
public:
    Impl();

    void init(const Options& options);

    double solveClosedForm(const Vector3dList& visualAngularVelocities,
                           const Vector3dList& imuAngularVelocities,
                           const std::vector<double>& timestamps, double td,
                           Eigen::Matrix3d& rotation,
                           Eigen::Vector3d& bias) const;

    void solveClosedForm(const Eigen::MatrixXd& angVis,
                         const Eigen::MatrixXd& angImu,
                         const std::vector<double>& t, double td,
                         Eigen::Matrix3d& Rs, Eigen::MatrixXd& bias, double& f);

public:
    Options opts_;
    Vector3dList smoothedImuAngVels_;
    Vector3dList smoothedVisAngVels_;
};

CameraImuRotationCalibration::Impl::Impl() {}

void CameraImuRotationCalibration::Impl::init(const Options& options)
{
    opts_ = options;
}

double CameraImuRotationCalibration::Impl::solveClosedForm(
    const Vector3dList& visAngVels, const Vector3dList& imuAngVels,
    const std::vector<double>& timestamps, double td, Eigen::Matrix3d& rotation,
    Eigen::Vector3d& bias) const
{
    // TODO: check size
    const size_t sampleCount = timestamps.size();

    Timeline timestampsWithOffset;
    timestampsWithOffset.reserve(sampleCount);
    std::transform(timestamps.begin(), timestamps.end(),
                   std::back_inserter(timestampsWithOffset),
                   [&td](const auto& t) { return t - td; });

    Vector3dList interpolatedVisAngVels;
    math::InterpolateVector3s(timestampsWithOffset, timestamps, visAngVels,
                              interpolatedVisAngVels);

    // Compute mean velocity.
    Eigen::Vector3d meanImuAngVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d meanVisAngVel = Eigen::Vector3d::Zero();
    for (size_t i{0}; i < interpolatedVisAngVels.size(); ++i) {
        meanImuAngVel += imuAngVels[i];
        meanVisAngVel += interpolatedVisAngVels[i];
    }
    meanImuAngVel /= static_cast<double>(sampleCount);
    meanVisAngVel /= static_cast<double>(sampleCount);

    // Centralized
    Eigen::MatrixXd P, Q;
    P.resize(sampleCount, 3);
    Q.resize(sampleCount, 3);
    for (size_t i{0}; i < sampleCount; ++i) {
        P.row(i) = imuAngVels[i] - meanImuAngVel;
        Q.row(i) = interpolatedVisAngVels[i] - meanVisAngVel;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd{
        P.transpose() * Q, Eigen::ComputeFullU | Eigen::ComputeFullV};
    const auto& V = svd.matrixV();
    const auto U_t = svd.matrixU().transpose();

    Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
    if ((V * U_t).determinant() < 0.) {
        C(2, 2) = -1.;
    }
    rotation = V * C * U_t;

    // Estimate bias
    bias = Eigen::Vector3d::Zero();
    if (opts_.estimateGyroBias) {
        bias = meanVisAngVel - rotation * meanImuAngVel;
    }

    // Get median error
    //    std::vector<double> errors;
    //    errors.reserve(sampleCount);
    //    for (size_t i{0}; i < sampleCount; ++i) {
    //        Eigen::Vector3d D =
    //            interpolated_vis_vels[i] - (rotation * imu_vels[i] + bias);
    //        errors.emplace_back(D.squaredNorm());
    //    }
    // const double med_error = utils::FindMedian(errors);

    constexpr double kHuberK{1.345};
    constexpr double kHuberK_2{kHuberK * kHuberK};

    double error{0.};
    for (size_t i{0}; i < sampleCount; ++i) {
        const Eigen::Vector3d D =
            interpolatedVisAngVels[i] - (rotation * imuAngVels[i] + bias);
        const double err = D.squaredNorm();
        if (err > kHuberK) {
            error += 2. * kHuberK * std::sqrt(err) - kHuberK_2;
        }
        else {
            error += err;
        }
    }

    return error;
}

///------- CameraImuRotationCalibration starts from here
CameraImuRotationCalibration::CameraImuRotationCalibration(
    const Options& options)
    : d(std::make_unique<Impl>())
{
    d->init(options);
}

CameraImuRotationCalibration::~CameraImuRotationCalibration() = default;

const CameraImuRotationCalibration::Options&
CameraImuRotationCalibration::options() const
{
    return d->opts_;
}

CameraImuRotationCalibration::Options& CameraImuRotationCalibration::rOptions()
{
    return d->opts_;
}

bool CameraImuRotationCalibration::calculate(
    const StampedQuaterniondList& visualOrientations,
    const StampedVector3dList& imuAngularVelocities, Eigen::Matrix3d& Ric,
    double& imuToCameraTimeOffset, Eigen::Vector3d& gyroBias)
{
    const auto& t_start_cam = visualOrientations.begin()->first;
    const auto& t_end_cam = visualOrientations.rbegin()->first;
    const auto& t_start_imu = imuAngularVelocities.begin()->first;
    const auto& t_end_imu = imuAngularVelocities.rbegin()->first;

    double t_start = std::max(t_start_cam, t_start_imu);
    double t_end = std::max(t_end_cam, t_end_imu);

    Timeline imu_ts;
    Vector3dList imu_ang_vels;
    for (const auto& [t, vel] : imuAngularVelocities) {
        if (t >= t_start && t <= t_end) {
            imu_ts.push_back(t - t_start);
            imu_ang_vels.push_back(vel);
        }
    }

    Timeline vis_ts;
    QuaterniondList vis_rots;
    for (const auto& [t, rot] : visualOrientations) {
        if (t >= t_start && t <= t_end) {
            vis_ts.push_back(t - t_start);
            vis_rots.push_back(rot);
        }
    }

    QuaterniondList interp_vis_rots;
    math::InterpolateQuaternions(vis_ts, imu_ts, vis_rots, interp_vis_rots);

    // Compute visual angular velocities
    QuaterniondList vis_rots_diff;
    for (size_t i{1}; i < interp_vis_rots.size(); ++i) {
        Eigen::Quaterniond q;
        q.w() = interp_vis_rots[i].w() - interp_vis_rots[i - 1].w();
        q.x() = interp_vis_rots[i].x() - interp_vis_rots[i - 1].x();
        q.y() = interp_vis_rots[i].y() - interp_vis_rots[i - 1].y();
        q.z() = interp_vis_rots[i].z() - interp_vis_rots[i - 1].z();
        vis_rots_diff.push_back(q);
    }
    vis_rots_diff.push_back(vis_rots_diff.back());

    const double diff_dt = -2. / d->opts_.imuInterval;
    Vector3dList vis_ang_vels;
    for (size_t i{0}; i < vis_rots_diff.size(); ++i) {
        auto vis_rot = vis_rots_diff[i] * interp_vis_rots[i].inverse();
        Eigen::Vector3d vis_ang_vel = vis_rot.vec() * diff_dt;
        // Suppress extremely large velocities > 2 * pi rad/s
        if (std::abs(vis_ang_vel[0]) > math::two_pi<double> ||
            std::abs(vis_ang_vel[1]) > math::two_pi<double> ||
            std::abs(vis_ang_vel[2]) > math::two_pi<double>) {
            if (i > 1) {
                vis_ang_vel = vis_ang_vels[i - 1];
            }
            else {
                vis_ang_vel.setZero();
            }
        }
        vis_ang_vels.push_back(vis_ang_vel);
    }

    // Calculate moving average to smooth the values a little bit
    d->smoothedImuAngVels_.clear();
    d->smoothedVisAngVels_.clear();

    constexpr size_t kWindowSize{15};
    MovingAverage imu_x{kWindowSize}, imu_y{kWindowSize}, imu_z{kWindowSize};
    MovingAverage vis_x{kWindowSize}, vis_y{kWindowSize}, vis_z{kWindowSize};
    for (size_t i{0}; i < imu_ang_vels.size(); ++i) {
        imu_x.add(imu_ang_vels[i][0]);
        imu_y.add(imu_ang_vels[i][1]);
        imu_z.add(imu_ang_vels[i][2]);
        d->smoothedImuAngVels_.emplace_back(imu_x.average(), imu_y.average(),
                                            imu_z.average());

        vis_x.add(vis_ang_vels[i][0]);
        vis_y.add(vis_ang_vels[i][1]);
        vis_z.add(vis_ang_vels[i][2]);
        d->smoothedVisAngVels_.emplace_back(vis_x.average(), vis_y.average(),
                                            vis_z.average());
    }

    const double gRatio = (1. + std::sqrt(5.)) / 2.;
    constexpr double kTolerance{1e-4};
    constexpr double kMaxOffset{1.0};

    double a = -kMaxOffset;
    double b = kMaxOffset;
    double c = b - (b - a) / gRatio;
    double dd = a + (b - a) / gRatio; // duplicate name

    int iter{0};
    double error{0.};
    while (std::abs(c - dd) > kTolerance) {
        Eigen::Matrix3d Rsc, Rsd;
        Eigen::Vector3d biasc, biasd;
        const double fc =
            d->solveClosedForm(d->smoothedVisAngVels_, d->smoothedImuAngVels_,
                               imu_ts, c, Rsc, biasc);
        const double fd =
            d->solveClosedForm(d->smoothedVisAngVels_, d->smoothedImuAngVels_,
                               imu_ts, dd, Rsd, biasd);

        if (fc < fd) {
            b = dd;
            Ric = Rsc;
            if (d->opts_.estimateGyroBias) {
                gyroBias = biasc;
            }
            error = fc;
        }
        else {
            a = c;
            Ric = Rsd;
            if (d->opts_.estimateGyroBias) {
                gyroBias = biasd;
            }
            error = fd;
        }

        c = b - (b - a) / gRatio;
        dd = a + (b - a) / gRatio;

        iter++;
    }
    imuToCameraTimeOffset = (b + a) / 2;

    LOG(INFO) << "Finished Golden-Section search in " << iter
              << " iterations."
                 "\n"
                 "Final estimated results are:"
                 "\n"
                 "Gyroscope to camera rotation(in quaternion) is: "
              << Eigen::Quaterniond{Ric}.coeffs()
              << "\n"
                 "Gyroscope bias(rad/s): "
              << gyroBias.transpose()
              << "\n"
                 "Time offset(s): "
              << imuToCameraTimeOffset
              << "\n"
                 "Alignment error: "
              << error;

    return true;
}

const Vector3dList& CameraImuRotationCalibration::smoothedImuAngularVelocities()
    const
{
    return d->smoothedImuAngVels_;
}

const Vector3dList&
CameraImuRotationCalibration::smoothedVisualAngularVelocities() const
{
    return d->smoothedVisAngVels_;
}

} // namespace thoht
