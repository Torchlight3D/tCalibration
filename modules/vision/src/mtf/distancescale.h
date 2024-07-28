#pragma once

#include <Eigen/Core>

#include "mtfcore.h"

// TODO: Duplicated code
static bool t_intersect(double &pix, double &piy, double v1x, double v1y,
                        double d1x, double d1y, double v2x, double v2y,
                        double d2x, double d2y);

class Distance_scale
{
public:
    Distance_scale();

    void construct(Mtf_core &mtf_core, bool pose_based = false,
                   cv::Rect *dimension_correction = nullptr,
                   double user_focal_ratio = -1,
                   const std::string &correspondence_file = {});

    cv::Point2d normalize_img_coords(double pixel_x, double pixel_y) const;

    Eigen::Vector3d backproject(double pixel_x, double pixel_y) const;

    void estimate_depth_img_coords(double pixel_x, double pixel_y,
                                   double &depth) const;

    void estimate_depth_world_coords(double world_x, double world_y,
                                     double &depth) const;

    cv::Point2d estimate_world_coords(double pixel_x, double pixel_y) const;

    cv::Point2d world_to_image(double world_x, double world_y,
                               double world_z = 1.0) const;

    double get_normal_angle_z() const;

    double get_normal_angle_y() const;

public:
    cv::Point2d zero;
    cv::Point2d transverse;
    cv::Point2d longitudinal;
    double chart_scale;
    int largest_block_index;

    cv::Point2d prin;
    double focal_length;
    double user_provided_focal_ratio;
    double centre_depth;
    double distortion;
    double img_scale;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    Eigen::Matrix3d fwdP;
    Eigen::Vector3d fwdT;
    Eigen::Matrix3d invP; // from image to world coords
    Eigen::Vector3d cop;  // centre of projection (i.e., camera)

    double bundle_rmse;
    bool fiducials_found;

    double roll_angle = 0;
    double yaw_angle = 0;
    double pitch_angle = 0;

    double page_scale_factor = 1;

    // this is an index into include/fiducial_positions.h :
    // fiducial_scale_factor, thus it identifies the specific page
    int fiducial_code_set = -1;

    std::vector<cv::Point2d> fid_img_points;
    std::vector<cv::Point3d> fid_world_points;

private:
    void enumerate_combinations(int n, int k);

private:
    std::vector<std::vector<int>> combinations;
};
