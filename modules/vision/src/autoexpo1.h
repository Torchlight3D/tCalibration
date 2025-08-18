#pragma once

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

namespace tl {
class ExpNode
{
public:
    ExpNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
    void CameraCb(const sensor_msgs::ImageConstPtr &msg);
    void ChangeParam(double shutter_new, double gain_new);
    void generate_LUT();

private:
    bool check_rate = false;
    double frame_rate_req = 10.0; // maximum 80 fps

    double gamma[7] = {1.0 / 1.9, 1.0 / 1.5, 1.0 / 1.2, 1.0, 1.2, 1.5, 1.9};
    double metric[7];
    double max_metric;
    double max_gamma, alpha, expNew, expCur, shutter_cur, shutter_new, gain_cur,
        gain_new, upper_shutter;
    double lower_shutter = 100.0; // adjust if necessary [unit: micro-second]
    double kp = 0.4;              // contorl the speed to convergence
    double d = 0.1,
           R; // parameters used in the nonliear function in Shim's 2018 paper
    int gamma_index; // index to record the location of the optimum gamma value
    bool gain_flag = false;

    // Parameters that correlated to Shim's Gradient Metric
    double met_act_thresh = 0.06;
    // The lamda value used in Shim's 2014 paper as a control
    // parameter to adjust the mapping tendency (larger->steeper)
    double lamda = 1000.0;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_camera_;
};
} // namespace tl
