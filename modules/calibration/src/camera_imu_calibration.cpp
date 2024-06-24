#include "camera_imu_calibration.h"

#include <glog/logging.h>
#include <json/json.hpp>

#include "spline_trajectory_estimator.h"

namespace tl {

namespace {
constexpr int kSplineOrder{6};
}

///------- CameraImuCalibration::Impl starts from here
class CameraImuCalibration::Impl
{
public:
    Impl();

    void init(const Options& options);

    void initializeGravity(const ImuDatas& data);

public:
    Options opts_;

    std::vector<double> m_cam_timestamps;
    StampedVector3dList m_gyro_data;
    StampedVector3dList m_acc_data;

    SplineWeightingData spline_weight_data_;

    double m_t0_spline;
    double m_t_end_spline;

    uint64_t nr_knots_so3_;
    uint64_t nr_knots_r3_;

    double m_init_cam_line_delay = 0.;
    bool m_calib_cam_line_delay = false;

    //! Gravity in sensor frame
    Eigen::Vector3d init_gravity_;

    Sophus::SE3d m_init_T_i_c;

    CameraId m_camId;

    bool m_gravity_initialized = false;
    bool reestimate_biases_ = false;

    Scene m_scene;

    SplineTrajectoryEstimator<kSplineOrder> m_trajectory;
};

CameraImuCalibration::Impl::Impl() {}

void CameraImuCalibration::Impl::init(const Options& options)
{
    opts_ = options;
}

void CameraImuCalibration::Impl::initializeGravity(const ImuDatas& imu_data)
{
    for (const auto& vis_stamp : m_cam_timestamps) {
        const View* view = [this, &vis_stamp]() -> const View* {
            const auto viewIds = m_scene.viewIdsAtTime(vis_stamp);
            const auto found = std::find_if(
                viewIds.cbegin(), viewIds.cend(), [this](const auto& viewId) {
                    return m_scene.view(viewId) &&
                           (m_scene.cameraId(viewId) == m_camId);
                });

            if (found == viewIds.cend()) {
                return nullptr;
            }

            return m_scene.view(*found);
        }();

        //        const auto* view = m_scene.view();
        if (!view) {
            continue;
        }

        const Eigen::Quaterniond q_w_c{
            view->camera().orientationAsRotationMatrix().transpose()};
        const auto p_w_c = view->camera().position();
        const auto T_a_i = Sophus::SE3d{q_w_c, p_w_c} * m_init_T_i_c.inverse();

        if (!m_gravity_initialized) {
            for (const auto& acc : imu_data.acc.d()) {
                const auto ad = acc.asVector();
                const int64_t t = acc.timestamp();
                if (std::abs(t - vis_stamp) < 1. / 30.) {
                    init_gravity_ = T_a_i.so3() * ad;
                    m_gravity_initialized = true;
                    LOG(INFO)
                        << "g_a initialized with " << init_gravity_.transpose()
                        << " at timestamp: " << t;
                }

                if (m_gravity_initialized) {
                    break;
                }
            }
        }
    }
    m_trajectory.setGravity(init_gravity_);
}

///------- CameraImuCalibration starts from here
CameraImuCalibration::CameraImuCalibration(const Options& options)
    : d(std::make_unique<Impl>())
{
    d->init(options);
}

CameraImuCalibration::~CameraImuCalibration() = default;

void CameraImuCalibration::setScene(Scene::Ptr scene)
{
    // TODO
}

void CameraImuCalibration::initSpline(const Scene& scene,
                                      const Sophus::SE3d& T_i_c_init,
                                      const SplineWeightingData& sewData,
                                      double dt_imu_to_cam, const ImuDatas& imu,
                                      double ld_init,
                                      const ImuIntrinsics& accl_intrinsics,
                                      const ImuIntrinsics& gyro_intrinsics)
{
    d->m_scene = scene;
    d->spline_weight_data_ = sewData;
    d->m_init_T_i_c = T_i_c_init;

    d->m_trajectory.setTic(T_i_c_init);
    d->m_trajectory.setIMUIntrinsics(accl_intrinsics, gyro_intrinsics);

    // Set camera timestamps and sort them
    for (const auto& id : scene.viewIds()) {
        d->m_cam_timestamps.push_back(scene.view(id)->timestamp());
    }
    std::sort(d->m_cam_timestamps.begin(), d->m_cam_timestamps.end());

    // initialize readout with 1/fps * 1/image_rows
    d->m_init_cam_line_delay = ld_init;
    d->m_trajectory.setCameraLineDelay(d->m_init_cam_line_delay);

    LOG(INFO) << "Initialized Line Delay to: "
              << time::sToNs(d->m_init_cam_line_delay) << "ns";

    // find smallest vision timestamp and set spline times
    auto result = std::minmax_element(d->m_cam_timestamps.begin(),
                                      d->m_cam_timestamps.end());
    d->m_t0_spline =
        d->m_cam_timestamps[result.first - d->m_cam_timestamps.begin()];
    d->m_t_end_spline =
        d->m_cam_timestamps[result.second - d->m_cam_timestamps.begin()];
    const auto start_t = time::sToNs(d->m_t0_spline);
    const auto end_t =
        time::sToNs(d->m_t_end_spline + 0.01) + d->m_init_cam_line_delay;
    const auto dt_so3 = time::sToNs(d->spline_weight_data_.so3_dt);
    const auto dt_r3 = time::sToNs(d->spline_weight_data_.r3_dt);

    d->m_trajectory.setTimes(dt_so3, dt_r3, start_t, end_t);

    LOG(INFO) << "Spline initialized with: "
                 "\n"
                 "Start/End: "
              << d->m_t0_spline << "/" << d->m_t_end_spline
              << "\n"
                 "Knots spacing r3/so3: "
              << d->spline_weight_data_.r3_dt << "/"
              << d->spline_weight_data_.so3_dt;

    d->nr_knots_so3_ = (end_t - start_t) / dt_so3 + kSplineOrder;
    d->nr_knots_r3_ = (end_t - start_t) / dt_r3 + kSplineOrder;

    LOG(INFO) << "Initializing " << d->nr_knots_so3_
              << " SO3 knots."
                 "\n"
                 "Initializing "
              << d->nr_knots_r3_ << " R3 knots.";

    // after initing times, let's now initialize the knots using the known
    // camera poses (T_w_c)
    d->m_trajectory.setScene(d->m_scene);
    d->m_trajectory.batchInitSO3R3VisPoses();
    // TODO make the bias spline dt configurable
    d->m_trajectory.initBiasSplines(accl_intrinsics.bias(),
                                    gyro_intrinsics.bias(), 10 * 1e9, 10 * 1e9,
                                    1.0, 1e-1);

    LOG(INFO) << "Add visual measurements to spline...";
    if (d->m_init_cam_line_delay != 0.0) { // Rolling shutter
        for (const auto& viewId : scene.viewIds()) {
            d->m_trajectory.addRSCameraMeasurement(scene.view(viewId), 0.0);
        }
    }
    else { // Global shutter
        for (const auto& viewId : scene.viewIds()) {
            d->m_trajectory.addGSCameraMeasurement(scene.view(viewId), 0.0);
        }
    }

    LOG(INFO) << "Adding IMU measurements to spline...";
    for (size_t i{0}; i < imu.acc.size(); ++i) {
        const auto& acc_data = imu.acc[i];
        const double t = acc_data.timestamp() + dt_imu_to_cam;
        if (t < d->m_t0_spline || d->m_t_end_spline) {
            continue;
        }

        d->m_gyro_data[t] = acc_data.asVector();
        d->m_acc_data[t] = acc_data.asVector();
        if (!d->m_trajectory.addAccelerometerMeasurement(
                acc_data.asVector(), time::sToNs(t), 1. / sewData.r3_var)) {
            LOG(ERROR) << "Failed to add accelerometer measurement at time "
                       << t;
        }

        if (!d->m_trajectory.addGyroscopeMeasurement(
                imu.gyro[i].asVector(), time::sToNs(t), 1. / sewData.so3_var)) {
            LOG(ERROR) << "Failed to add gyroscope measurement at time: " << t;
        }
    }

    d->initializeGravity(imu);
}

void CameraImuCalibration::setKnownGravityDir(const Eigen::Vector3d& gravity)
{
    d->m_trajectory.setGravity(gravity);
}

double CameraImuCalibration::optimize(int iterations, int flags)
{
    auto summary = d->m_trajectory.optimize(iterations, flags);
    return d->m_trajectory.calcMeanReprojectionError();
}

void CameraImuCalibration::getScene(Scene& recon) const
{
    for (const auto& vis_stamp : d->m_cam_timestamps) {
        const auto time = time::sToNs(vis_stamp);

        Sophus::SE3d pose;
        d->m_trajectory.poseAt(time, pose);

        const auto viewId =
            recon.addView(std::to_string(time), time, kCameraLeftId);
        auto view = recon.rView(viewId);
        view->setEstimated(true);

        auto& camera = view->rCamera();
        camera.setOrientationFromRotationMatrix(
            pose.rotationMatrix().transpose());
        camera.setPosition(pose.translation());
    }
}

void CameraImuCalibration::clearSpline()
{
    d->m_cam_timestamps.clear();
    d->m_gyro_data.clear();
    d->m_acc_data.clear();
}

void CameraImuCalibration::getIMUIntrinsics(ImuIntrinsics& acc_intrinsics,
                                            ImuIntrinsics& gyr_intrinsics,
                                            int64_t time)
{
    acc_intrinsics = d->m_trajectory.accelerometerIntrinsicsAt(time);
    gyr_intrinsics = d->m_trajectory.gyroscopeIntrinsicsAt(time);
}

void CameraImuCalibration::setCalibrateRSLineDelay(bool on)
{
    d->m_calib_cam_line_delay = on;
}

bool CameraImuCalibration::calibrateRSLineDelay() const
{
    return d->m_calib_cam_line_delay;
}

void CameraImuCalibration::setInitRSLineDelay(double delay)
{
    d->m_init_cam_line_delay = delay;
}

double CameraImuCalibration::initialRSLineDelay() const
{
    return d->m_init_cam_line_delay;
}

double CameraImuCalibration::calibratedRSLineDelay() const
{
    return d->m_trajectory.cameraLineDelay();
}

void CameraImuCalibration::setCameraId(CameraId camId) { d->m_camId = camId; }

std::vector<double> CameraImuCalibration::getCamTimestamps() const
{
    return d->m_cam_timestamps;
}

StampedVector3dList CameraImuCalibration::getGyroMeasurements() const
{
    return d->m_gyro_data;
}

StampedVector3dList CameraImuCalibration::getAcclMeasurements() const
{
    return d->m_acc_data;
}

Eigen::Vector3d CameraImuCalibration::acceleratorBiasAt(int64_t time) const
{
    return d->m_trajectory.GetAcclBias(time);
}

Eigen::Vector3d CameraImuCalibration::gyroscopeBiasAt(int64_t time) const
{
    return d->m_trajectory.GetGyroBias(time);
}

Eigen::Vector3d CameraImuCalibration::estimatedGravity() const
{
    return d->m_trajectory.gravity();
}

void CameraImuCalibration::imuToCameraTransform(Eigen::Quaterniond& quat,
                                                Eigen::Vector3d& trans) const
{
    quat = d->m_trajectory.Tic().so3().unit_quaternion();
    trans = d->m_trajectory.Tic().translation();
}

} // namespace tl
