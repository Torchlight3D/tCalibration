#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>
#include <opencv2/core/mat.hpp>

namespace tl {

constexpr int _n_corrections = 7;
constexpr int _n_order_polyfit = 6;
constexpr int _n_coeffs_polyfit = _n_order_polyfit + 1;

constexpr int GRADINFO_SCORE = 0;
constexpr int STEREOMATCHES_SCORE = 1;

constexpr int GAMMA_CORRECTIONS = 0;
constexpr int EV_CORRECTIONS = 1;

constexpr int IMU_ESTIM = 0;
constexpr int OPTIFLOW_ESTIM = 1;

// Defining custom Eigen variable type
using Vector5f = Eigen::Vector<float, 5>;
using VectorNcorrect = Eigen::Matrix<float, _n_corrections, 1>;
using VectorNorderFit = Eigen::Matrix<float, _n_order_polyfit + 1, 1>;

struct ExposureParameters
{
    // Exposure time in seconds
    float exposure;

    // Camera gain in dB
    float gain;
};

// Metrics and other useful variables used/produced by the controller
struct ControlVariables
{
    // score/metric associated to each gamma-corrected image (e.g.
    // avg. gradient info or number of stereo matches)
    VectorNcorrect scores;

    // coefficients for the polynomial fitted to the gamma-corrected image
    // metrics
    VectorNorderFit polyfit_coeffs;

    // optimal gamma determined based on simulated gamma corrections
    float gamma_optim;

    // optimal change in exposure value based on simulated
    // changes in exposure values
    float deltaEV_optim;

    // Hand-tuned score balancing noise and motion blur for exposure-gain
    // balance
    float next_frame_optim_score;

    // predicted exposure value for the next frame
    float exposure_value_next;

    // next exposure time determined
    float exposure_next;

    // next gain determined
    float gain_next;
};

struct ControlParameters
{
    using ParameterMatrix =
        Eigen::Matrix<float, _n_corrections, _n_coeffs_polyfit>;

    int n_corrections = _n_corrections;
    int n_order_polyfit = _n_order_polyfit;
    int n_coeffs_polyfit = _n_coeffs_polyfit;

    // gamma values used for prediction
    VectorNcorrect gamma_values;

    // pre-computed matrix of powers of gamma for curve fitting
    ParameterMatrix gamma_powers_matrix;

    // QR decomposition of the above
    Eigen::ColPivHouseholderQR<ParameterMatrix> gamma_powers_matrix_decomposed;
    // Lookup tables for gamma
    std::vector<cv::Mat> gamma_tables;

    // change in exposure value used for prediction (alternative to gamma_values
    // which relies on photometric response)
    VectorNcorrect delta_EVs;
    // pre-computed matrix of powers of delta_EVs for curve fitting
    ParameterMatrix delta_EVs_powers_matrix;
    // QR decomposition of the above
    Eigen::ColPivHouseholderQR<ParameterMatrix>
        delta_EVs_powers_matrix_decomposed;
    // Lookup tables for imposed change in exposure value (relies on photometric
    // response)
    std::vector<cv::Mat> delta_EV_tables;

    // Min/Max illumination (as mean pixel value)
    double Imin = 10;
    double Imax = 245;

    // Downsampling size
    int down_size_h = 270;
    int down_size_w = 360;

    // Control parameters based on Shim 2018 and Mehta 2020.
    double kp = 1.6; // 0.8
    double d = 0.1;
    double R;

    // Parameters for computing Shim gradient metric
    double met_act_thresh = 0.3;
    // Lookup tables for gradient info metric (Shim 2014)
    cv::Mat metric_table;
    double sigma = 255.0 * met_act_thresh;
    double lambda = 1000.0;

    // Parameters for balancing gain and exposure
    // 0.1; //0.1; // weight on cost associated to gain (wrt to
    // average blur length in pixels) -> higher value = use lower gain
    double w = 0.02;
};

struct Frame
{
    // Basic input variables
    // Image number
    int seq = 0;
    // Camera timestamp
    float timestamp;
    // actual image
    cv::Mat img;
    // Exposure time in seconds
    float exposure;
    // Camera gain in dB
    float gain;

    // (Optional) instantaneous twist estimate in camera optical ref. frame
    // (v_x,v_y,v_z,w_x,w_y,w_z)
    Eigen::Matrix<float, 6, 1> twist;

    // Derived variables
    // Exposure value computed according to Mehta 2020.
    float exposure_value;
    // estimated average pixel speed
    float avg_pixel_speed = 0.0;
    // Keep track of quality metrics associated to each frame
    ControlVariables control_vars;
};

struct Camera
{
    std::string camid;
    // Photometric calibration (necessary for some methods)
    Vector5f photocalib;
    // See Bergmann 2018 "Online Photometric Calibration of Auto Exposure Video
    // for Realtime Visual Odometry and SLAM" Info:
    // https://vision.in.tum.de/research/vslam/photometric-calibration Code:
    // https://github.com/tum-vision/online_photometric_calibration Coefficients
    // of a fifth-order polynomial fit. X: Exposure, I: Intensity ln(X) = p[0] +
    // p[1]*I + p[2]*I^2 + p[3]*I^3 + p[4]*I^4 + p[5]*I^5

    // # of horizontal pixels/m * focal length
    float fx;
    // # of vectical pixels/m * focal length
    float fy;
    // horizontal field of view (rads)
    float fov_x;
    // vectical field of view (rads)
    float fov_y;
    // number of pixels per rad, horizontal
    float px_per_rad_x;
    // number of pixels per rad, vectical
    float px_per_rad_y;

    // minimum gain factor in dB (note: do not set to >=0, otherwise FLIR camera
    // switched to auto-gain)
    float gain_min = 0.1;
    // maximum gain factor in dB
    float gain_max = 40.0;

    // minimum exposure time in seconds (note: do not set to >=0, otherwise FLIR
    // camera switched to auto-expose)
    float exposure_min = 0.000105;
    // maximum exposure time in seconds
    float exposure_max = 0.015000;

    // camera F-number
    float F = 1.2;

    // minimum exposure value
    float exposure_value_min = 2 * std::log2(F) - std::log2(exposure_max) -
                               std::log2(10) / 20 * gain_max;

    // maximum possible exposure value
    float exposure_value_max = 2 * std::log2(F) - std::log2(exposure_min) -
                               std::log2(10) / 20 * gain_min;

    // Buffered camera frames
    std::vector<Frame> frames;
    // Images height in px
    int height;
    // Images width in px
    int width;
};

struct ImuMeasurement
{
    // linear acceleration (m/s^2), [a_x, a_y, a_z]
    Eigen::Vector3f lin_accel;
    // rotational velocity (rad/s), [w_x, w_y, w_z]
    Eigen::Vector3f ang_vel;
    // timestamp in seconds
    float timestamp;
};

struct Imu
{
    std::string imuid;
    std::vector<ImuMeasurement> measures;
    // Transform from IMU frame to camera frame
    Eigen::Matrix<float, 4, 4> T_imu_2_cam;
};

class ExpGainController
{
public:
    // Control parameters
    ControlParameters control_params;
    // Number of cameras considered by the controller
    int n_cameras;
    // Cameras considered by the controller
    std::vector<Camera> cameras;
    // Imus considered by the controller
    std::vector<Imu> imus;

    // ExpGainController(const std::string& _controller_name)
    //     : controller_name{_controller_name}
    // {
    // }

    // Initialization functions
    void add_camera(const std::string& id, int h, int w, float F, float fx,
                    float fy, const Vector5f& calib);
    void add_imu(const std::string& id, const Eigen::Matrix4f& T_imu_2_cam);
    void set_gamma_values(const VectorNcorrect& values);

    void init();
    void compute_gamma_LUTs();
    void compute_metric_LUT();
    void compute_delta_EV_LUTs();

    // Data acquisition
    void add_frame(int camid, cv::Mat img, float exposure_time, float gain,
                   int seq, float timestamp);
    void add_imu_meas(int imuid, const Eigen::Vector3f& lin_accel,
                      const Eigen::Vector3f& ang_vel, float timestamp);

    // Optional (depends on implementation)
    // void grab_imu();
    // void get_optical_flow();
    // void get_local_features();
    // void get_twist();
    ExposureParameters update_params(int score = GRADINFO_SCORE,
                                     int correction = GAMMA_CORRECTIONS,
                                     int blur_estim = IMU_ESTIM);
};
} // namespace tl
