#include "autoexpo1.h"

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

namespace tl {

using Vector6d = Eigen::Vector<double, 6>;
using Vector7d = Eigen::Vector<double, 7>;

cv::Mat lookUpTable_01(1, 256, CV_8U);
cv::Mat lookUpTable_05(1, 256, CV_8U);
cv::Mat lookUpTable_08(1, 256, CV_8U);
cv::Mat lookUpTable_1(1, 256, CV_8U);
cv::Mat lookUpTable_12(1, 256, CV_8U);
cv::Mat lookUpTable_15(1, 256, CV_8U);
cv::Mat lookUpTable_19(1, 256, CV_8U);
cv::Mat lookUpTable_metric(1, 256, CV_8U);

Vector6d curveFit(const Vector7d& x, const Vector7d& y)
{
    Eigen::Matrix<double, 7, 6> A;
    for (int i = 0; i < 7; i++) {
        for (int j = 5; j >= 0; j--) {
            A(i, 5 - j) = std::pow(x[i], j);
        }
    }

    Vector7d b;
    for (int i = 0; i < 7; i++) {
        b(i, 0) = y[i];
    }

    Eigen::MatrixXd A1 = A.transpose() * A;
    Eigen::MatrixXd b1 = A.transpose() * b;
    // Eigen::MatrixXd Q =A1.colPivHouseholderQr().solve(b1);
    Eigen::MatrixXd Q = A1.inverse() * b1;

    constexpr int n = 5;
    Vector6d coff;
    for (int i = 0; i < n + 1; i++) {
        coff[i] = Q(i);
    }

    return coff;
}

double* curveFit(double x[7], double y[7])
{
    Eigen::MatrixXd A(7, 6);
    for (int i = 0; i < 7; i++) {
        for (int j = 5; j >= 0; j--) {
            A(i, 5 - j) = pow(x[i], j);
        }
    }

    Eigen::MatrixXd b(7, 1);
    for (int i = 0; i < 7; i++) {
        b(i, 0) = y[i];
    }

    Eigen::MatrixXd A1 = A.transpose() * A;
    Eigen::MatrixXd b1 = A.transpose() * b;
    // Eigen::MatrixXd Q =A1.colPivHouseholderQr().solve(b1);
    Eigen::MatrixXd Q = A1.inverse() * b1;

    int n = 5;
    static double coff[6];
    for (int i = 0; i < n + 1; i++) {
        coff[i] = Q(i);
    }

    return coff;
}

double findRoots1(double a[6], double check)
{
    double ad[5];
    ad[4] = 5 * a[0];
    ad[3] = 4 * a[1];
    ad[2] = 3 * a[2];
    ad[1] = 2 * a[3];
    ad[0] = a[4];

    Eigen::Matrix4d companion_mat;
    for (int n = 0; n < 4; n++) {
        for (int m = 0; m < 4; m++) {
            if (n == m + 1) {
                companion_mat(n, m) = 1.0;
            }
            if (m == 4 - 1) {
                companion_mat(n, m) = -ad[n] / ad[4];
            }
        }
    }

    Eigen::MatrixXcd eig = companion_mat.eigenvalues();
    double opt_gamma = 997.0, met_temp;
    double roots1[4];
    for (int i = 0; i < 4; i++) {
        // met_temp is used to check if the root can return a
        // larger metric
        met_temp = 0.0;
        // if statement to determine whether or not root is true
        if (std::imag(eig(i)) == 0) {
            roots1[i] = std::real(eig(i));
        }
        else {
            roots1[i] = 1000; // if root is imaginary, assign an overshoot value
        }

        // check if the calculated root is within range (.5,2)
        if ((roots1[i] < 2.0) && (roots1[i] > 0.5)) {
            for (int j = 0; j < 6; j++) {
                met_temp = met_temp + a[j] * std::pow(roots1[i], 5 - j);
            }

            // if the root can return a metric that is greater than current
            // metric
            if (met_temp > check) {
                opt_gamma = roots1[i];
                LOG(INFO) << "in function maximum metric is: " << met_temp;
            }
        }
        LOG(INFO) << "eig(i) is: " << std::real(eig(i))
                  << "   ima: " << std::imag(eig(i));
    }
    return opt_gamma;
}

double image_gradient_gamma(cv::Mat& src_img, int j)
{
    // Accepting the raw image and the index of gamma value as input argument

    cv::Mat res = src_img.clone();
    ///////////////////// The following computes the image gradient of the
    /// gamma-processed image /////////////////////
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y, dst_img;

    cv::Mat weight_ori = cv::Mat::ones(res.rows, res.cols, CV_64FC1);

    // Using the corresponding index to find out the correct lookuptable to use
    if (j == 0) {
        cv::LUT(src_img, lookUpTable_01, res);
    }
    else if (j == 1) {
        cv::LUT(src_img, lookUpTable_05, res);
    }
    else if (j == 2) {
        cv::LUT(src_img, lookUpTable_08, res);
    }
    else if (j == 3) {
        cv::LUT(src_img, lookUpTable_1, res);
    }
    else if (j == 4) {
        cv::LUT(src_img, lookUpTable_12, res);
    }
    else if (j == 5) {
        cv::LUT(src_img, lookUpTable_15, res);
    }
    else if (j == 6) {
        cv::LUT(src_img, lookUpTable_19, res);
    }

    // Define variables that will be used in the sobel gradient determination
    // function
    constexpr int scale = 1;
    constexpr int delta = 0;
    constexpr int ddepth = CV_8UC1;

    // Call the Sobel function to determine gradient image in x and y direction

    // Gradient X
    cv::Sobel(res, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    cv::Sobel(res, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst_img);

    ////////////////////// The following computes the gradient metric based on
    /// the gradient image ///////////////////////////

    // Method 1: simple sumation of total gradient
    // double metric= cv::sum(dst_img)[0]; // simple sum of total gradient as
    // metric (Alternative 1 of the Shim's metric)

    // Method 2: Shim's 2014 gradient metric function
    // Using the metric equation given in Shim's 2014 paper
    cv::LUT(dst_img, lookUpTable_metric, res);
    res.convertTo(res, CV_64FC1);

    // cv::imshow("image_to_show",dst_img); // comment later
    return cv::sum(res)[0];
}

ExpNode::ExpNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), it_(nh)
{
    const std::string image_topic = "camera/image_raw";
    const std::string service_call =
        "camera/spinnaker_camera_nodelet/set_parameters";
    const std::string exp_param_call =
        "camera/spinnaker_camera_nodelet/exposure_time";
    const std::string gain_param_call = "camera/spinnaker_camera_nodelet/gain";

    LOG(INFO) << "the  image topic given in launch file? :"
              << nh.getParam("/image_topic", image_topic);
    LOG(INFO) << "the value of image topic is : " << image_topic;
    LOG(INFO) << "the  image topic given in launch file? :"
              << nh.getParam("/service_call", service_call);
    LOG(INFO) << "the value of service call val is : " << service_call;
    LOG(INFO) << "the  image topic given in launch file? :"
              << nh.getParam("/exp_param_call", exp_param_call);
    LOG(INFO) << "the value of exp param is : " << exp_param_call;
    LOG(INFO) << "the  image topic given in launch file? :"
              << nh.getParam("/gain_param_call", gain_param_call);
    LOG(INFO) << "the value of gain param is : " << gain_param_call;
    LOG(INFO) << "the  kp given in launch file? :" << nh.getParam("/kp", kp);
    LOG(INFO) << "the value of kp is : " << kp;

    generate_LUT();
    sub_camera_ = it_.subscribe(image_topic, 1, &ExpNode::CameraCb, this);
}

void ExpNode::CameraCb(const sensor_msgs::ImageConstPtr& msg)

{
    if (check_rate) {
        try {
            check_rate = false;

            cv::Mat image_capture = cv_bridge::toCvCopy(msg, "rgb8")->image;
            cv::cvtColor(image_capture, image_capture, cv::COLOR_BGR2GRAY);

            // cv::Size size(512,612); // may want to try size(408,342) if speed
            // is limited
            const cv::Size size(342, 408);
            cv::Mat image_current;
            cv::resize(image_capture, image_current, size);
            // image_capture = image_current;

            // Call the image processing funciton here (i.e. the gamma
            // processing), returning a float point gamma value

            // calculate the upper limit of shutter speed [unit:microsecond]
            if ((1.0 / frame_rate_req) * 1000000.0 > 32754.0) {
                upper_shutter = 32754.0;
            }
            else {
                upper_shutter = round(1000000.0 / frame_rate_req);
            }

            // loop to call image_gradient_gamma function to obtain image
            // gradient of each gamma manually adjust the possible gamma values
            // and the number of gamma to use
            for (int i = 0; i < 7; ++i) {
                // passing the corresponding index
                metric[i] = image_gradient_gamma(image_current, i) / 1000000;
                LOG(INFO) << "\nmetric " << i << " is:" << metric[i];
                LOG(INFO) << "  " << metric[i];
            }

            // loop to find out the index that correslated to the
            // optimum/maximum gamma value
            double temp = -1.0;
            for (int i = 0; i < 7; i++) {
                if (metric[i] > temp) {
                    temp = metric[i];
                    gamma_index = i;
                }
            }

            // Curve Fitting
            // Call the curve fitting function to find out coefficient
            double* coeff_curve = curveFit(gamma, metric);

            double coeff[6];
            for (int i = 0; i < 6; i++) {
                coeff[i] = *(coeff_curve + i);
                LOG(INFO) << "\ncoeff " << i << " is:" << coeff[i];
            }

            // calling function findRoots1 to find opt_gamma
            max_gamma = findRoots1(coeff, metric[gamma_index]);

            LOG(INFO) << "\nopt_gamma now is:  " << max_gamma;

            double metric_check = 0.0;
            for (int i = 0; i < 6; i++) {
                metric_check += coeff[i] * std::pow(max_gamma, 5 - i);
            }
            LOG(INFO) << "metric_check = " << metric_check;

            if (max_gamma < 1.0 / 1.9 || max_gamma > 1.9) {
                // find out the optimum gamma value associated with highest
                // image gradient
                max_gamma = gamma[gamma_index];
            }
            else if (metric[gamma_index] > metric_check) {
                max_gamma = gamma[gamma_index];
            }

            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // alpha value refers to Shim's 2014 paper
            if (max_gamma < 1)
            //{alpha = 0.5;} //original paper used 0.5
            {
                alpha = 1.0;
            }
            if (max_gamma >= 1) {
                alpha = 1.0;
            }

            // ros::param::get("/blackfly/spinnaker_camera_nodelet/exposure_time",
            // shutter_cur); // get the current shutter
            // ros::param::get("/blackfly/spinnaker_camera_nodelet/gain",
            // gain_cur); // get the current gain

            // get the current shutter
            ros::param::get(exp_param_call, shutter_cur);
            ros::param::get(gain_param_call, gain_cur); // get the current gain

            // unit from micro-second to second
            shutter_cur = shutter_cur / 1000000;

            // The 7.84 here is because the camera
            // we are using has a F-number of 2.8
            expCur =
                std::log2(7.84 / (shutter_cur * std::pow(2, (gain_cur / 6.0))));

            // This update function was implemented in Shim's 2018 paper which
            // is an update version of his 2014 paper
            R = d * std::tan((2 - max_gamma) * std::atan2(1, d) -
                             std::atan2(1, d)) +
                1;
            expNew = (1 + alpha * kp * (R - 1)) * expCur;

            // Shim's 2014 version of update function
            // expNew = (1+ kp * alpha * (1-max_gamma)) * expCur;

            //[unit: micro-second]  Note: 7.84 = 2.8*2.8
            shutter_new = (7.84) * 1000000 / (std::pow(2, expNew));

            // std::cout << "\ngain: " << round(gain_cur) << "   shutter_new:
            // "<< shutter_new << std::endl; //

            if (shutter_new > upper_shutter) {
                gain_flag = true;
                shutter_new = upper_shutter;
            }
            else if (shutter_new < lower_shutter) {
                gain_flag = true;
                shutter_new = lower_shutter;
            }
            else {
                gain_flag = false;
            }

            if (gain_flag) {
                gain_new = 6.0 * (expCur - expNew) + gain_cur;

                if (gain_new > 30) {
                    gain_new = 30;
                }
                else if (gain_new < -10) {
                    gain_new = -10;
                }
                gain_flag = false;
            }
            else if (shutter_new < upper_shutter &&
                     shutter_new > lower_shutter) {
                gain_new = 0.0;
                // gain_flag = false;
                // gain_cur=gain_new;
            }

            ///////////////////////////// Comment or delete the following three
            /// lines in implementation ////////////////
            LOG(INFO) << "\ngain: " << round(gain_cur)
                      << "   shutter_new: " << shutter_new;
            LOG(INFO) << "\ncurrent opt met: " << metric[gamma_index]
                      << "; met at 1.0: " << metric[3];

            LOG(INFO) << "max gamma: " << max_gamma
                      << " ; gain_new: " << gain_new
                      << "; shutter_cur: " << shutter_cur
                      << " ; shutter_new:  " << shutter_new;

            ///////////////////////////////////////////////////////////////////
            // Then call the function to determine what exposure time and gain
            // to update
            ///////////////////////////////////////////////////////////////////

            // may input the gain and exposure time update
            ChangeParam(shutter_new, gain_new);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'mono8'.",
                      msg->encoding.c_str());
        }
    }
    else {
        check_rate = true;
    }
}

// may have input of the updated
// gain, exposure time settings
void ExpNode::ChangeParam(double shutter_new, double gain_new)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    // enable "acquisition_frame_rate_enable"
    dynamic_reconfigure::BoolParameter acq_fps_bool;
    // Auto gain, white balance and exposure off
    dynamic_reconfigure::StrParameter gain_auto_str, exp_auto_str, wb_auto_str;
    dynamic_reconfigure::DoubleParameter acq_fps_double, exp_time_double,
        gain_double, exp_auto_upper_double;
    dynamic_reconfigure::Config conf;

    // set constant frame rate
    // maximum frame rate: 80 fps
    acq_fps_bool.name = "acquisition_frame_rate_enable";
    acq_fps_bool.value = true;
    conf.bools.push_back(acq_fps_bool);

    // trun off built-in auto exposure
    exp_auto_str.name = "exposure_auto"; // shut off auto exposure
    exp_auto_str.value = "Off";
    conf.strs.push_back(exp_auto_str);

    // turn off auto gain
    gain_auto_str.name = "auto_gain"; // shut off auto gain adjustment
    gain_auto_str.value = "Off";
    conf.strs.push_back(gain_auto_str);

    // wb_auto_str.name = "auto_white_balance"; // shut off auto white balance
    // wb_auto_str.value = "Off";
    // cosf.strs.push_back(wb_auto_str);

    // Set required frame rate
    // maximum frame rate: 80 fps
    acq_fps_double.name = "acquisition_frame_rate";
    acq_fps_double.value = frame_rate_req; // change frame rate as needed
    conf.doubles.push_back(acq_fps_double);

    // Update exposure time
    // Maximum range: 0 to 32754 [unit: micro-seconds]
    exp_time_double.name = "exposure_time";
    exp_time_double.value = shutter_new;
    conf.doubles.push_back(exp_time_double);

    // Update gain
    gain_double.name = "gain"; // Maximum range: -10 to 30
    gain_double.value = gain_new;
    conf.doubles.push_back(gain_double);

    // calculate and set highest possible exposure time (the camera has a limit
    // of 32754 [unit: microsecond])
    exp_auto_upper_double.name = "auto_exposure_time_upper_limit";
    if ((1.0 / frame_rate_req) * 1000000.0 > 32754.0) {
        exp_auto_upper_double.value = 32754.0;
    }
    else {
        exp_auto_upper_double.value = 1000000.0 / frame_rate_req;
    }
    conf.doubles.push_back(exp_auto_upper_double);

    srv_req.config = conf;

    // ros::service::call("/blackfly/spinnaker_camera_nodelet/set_parameters",srv_req,
    // srv_resp);
    ros::service::call(service_call, srv_req, srv_resp);
}

void ExpNode::generate_LUT()
{
    // Need to keep the same gamma values as in imageCallBack
    constexpr double gamma[7]{1.0 / 1.9, 1.0 / 1.5, 1.0 / 1.2, 1.0,
                              1.2,       1.5,       1.9};

    // The sigma value is used in Shim's 2014 paper as a
    // activation threshhold, the paper used a value of 0.06
    double sigma = 255.0 * met_act_thresh;
    // double lamda = 1000.0; // The lamda value used in Shim's 2014 paper as a
    // control parameter to adjust the mapping tendency (larger->steeper)

    uchar* p;
    uchar* q = lookUpTable_metric.ptr();

    for (int j = 0; j < 7; j++) {
        if (j == 0) {
            p = lookUpTable_01.ptr();
        }
        else if (j == 1) {
            p = lookUpTable_05.ptr();
        }
        else if (j == 2) {
            p = lookUpTable_08.ptr();
        }
        else if (j == 3) {
            p = lookUpTable_1.ptr();
        }
        else if (j == 4) {
            p = lookUpTable_12.ptr();
        }
        else if (j == 5) {
            p = lookUpTable_15.ptr();
        }
        else if (j == 6) {
            p = lookUpTable_19.ptr();
        }

        for (int i = 0; i < 256; ++i) {
            p[i] = cv::saturate_cast<uchar>(std::pow(i / 255.0, 1 / gamma[j]) *
                                            255.0);

            // The following if statement to create a lookup table based on the
            // activation threshold value in Shim's 2014 paper
            if (i >= sigma) {
                q[i] =
                    255 * ((std::log10(lamda * ((i - sigma) / 255.0) + 1)) /
                           (std::log10(lamda * ((255.0 - sigma) / 255.0) + 1)));
            }
            else {
                q[i] = 0;
            }
        }
    }
}

} // namespace tl
