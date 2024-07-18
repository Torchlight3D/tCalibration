#pragma once

#include <map>

#include "types.h"

namespace ipa_Fiducials {

struct FiducialPiParameters
{
    int m_id;             // Unique ID of tag
    cv::Point2d m_offset; // Offset of tag to target coordinate system

    cv::Rect_<double>
        m_sharpness_pattern_area_rect3d; // rectangle describing the area for
                                         // sharpness computation in 2d
                                         // coordinates within the marker plane
                                         // with respect to the marker origin

    double line_width_height; ///< Common width and height of fiducial

    // Assert that cross_ratio(line0) > cross_ratio(line1)
    double d_line0_AB; ///< Distance of from A to B of 4pt line A-B-C-D
    double d_line0_AC; ///< Distance of from A to C of 4pt line A-B-C-D

    double d_line1_AB; ///< Distance of from A to B of 4pt line A-B-C-D
    double d_line1_AC; ///< Distance of from A to C of 4pt line A-B-C-D
};

/// Struct to represent a pi fiducial
struct t_pi
{
    void sparse_copy_to(t_pi& copy)
    {
        copy.parameters.m_id = parameters.m_id;
        copy.marker_points = marker_points;
    }

    FiducialPiParameters parameters;

    double cross_ration_0; ///< Cross ration for line type 0
    double cross_ration_1; ///< Cross ration for line type 1

    std::vector<std::vector<cv::RotatedRect>>
        fitting_image_lines_0; ///< lines that fit to the first cross ratio
    std::vector<std::vector<cv::RotatedRect>>
        fitting_image_lines_1; ///< lines that fit to the second cross ratio

    int no_matching_lines; ///< Number of matching sides (at most 4 per marker)

    std::vector<cv::Point2f>
        marker_points; ///< ellipse coordinates in marker coordinate system
    std::vector<cv::RotatedRect>
        image_points; ///< ellipse coordinates in marker coordinate system
};

class FiducialModelPi
{
public:
    FiducialModelPi();
    ~FiducialModelPi();

    bool Init(cv::Mat& camera_matrix, std::string directory_and_filename,
              bool log_or_calibrate_sharpness_measurements = false,
              cv::Mat extrinsic_matrix = cv::Mat());

    bool SetExtrinsics(cv::Mat& camera_matrix,
                       cv::Mat extrinsic_matrix = cv::Mat());

    bool ApplyExtrinsics(cv::Mat& rot_CfromO, cv::Mat& trans_CfromO);

    bool GetPoints(cv::Mat& image, std::vector<t_points>& vec_points);

    /// Locates the fiducial within the image and inferes the camera pose from
    /// it
    /// @param scene image
    /// @return <code>RET_FAILED</code> if no tag could be detected
    /// <code>RET_OK</code> on success
    bool GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose);

    /// Computes a measure of image sharpness by analyzing the marker region and
    /// the inserted Siemens star
    /// @param image Scene image
    /// @param pose_CfromO Pose of the detected tag relative to the camera
    /// @param fiducial_parameters Container for several parameters of the
    /// utilized marker. Mainly the offsets are relevant to the function.
    /// @param sharpness_measure Degree of image sharpness (in [0...1] =
    /// [blurry...sharp]) computed by the function.
    /// @return <code>RET_FAILED</code> if the tag id is invalid
    /// <code>RET_OK</code> on success
    bool GetSharpnessMeasure(
        const cv::Mat& image, t_pose pose_CfromO,
        const FiducialPiParameters& fiducial_parameters,
        double& sharpness_measure,
        double sharpness_calibration_parameter_m = 9139.749632393357,
        double sharpness_calibration_parameter_n = -2670187.875850272);

    // Gets the camera matrix
    // @return 3x3 camera matrix (fx 0 cx, 0 fy cy, 0 0 1)
    cv::Mat GetCameraMatrix() const;

    // Set the camera matrix
    // @param camera_matrix 3x3 camera matrix (fx 0 cx, 0 fy cy, 0 0 1)
    void SetCameraMatrix(cv::Mat camera_matrix);

    // Gets the distortion coeffs
    // @return 1x4 distortion coeffs matrix (k1,k2,p1,p2)
    cv::Mat GetDistortionCoeffs() const;

    // Sets the distortion coeffs
    // @param dist_coeffs 1x4 distortion coeffs matrix (k1,k2,p1,p2)
    void SetDistortionCoeffs(cv::Mat dist_coeffs);

    // Gets the general fiducial parameters for a certain marker
    // @return general fiducial parameters
    FiducialPiParameters GetGeneralFiducialParameters(int marker_id);

    /// Load fiducial-centric coordinates of markers from file
    /// @param directory Directory, where the parameters of all fiducials are
    /// stores
    bool LoadParameters(std::string directory_and_filename);
    bool LoadParameters(std::vector<FiducialPiParameters> pi_tags);

    std::string GetType() { return "PI"; };

private:
    cv::Mat m_camera_matrix;     ///< Intrinsics of camera for PnP estimation
    cv::Mat m_dist_coeffs;       ///< Intrinsics of camera for PnP estimation
    cv::Mat m_extrinsic_XYfromC; ///< Extrinsics 4x4 of camera to rotate and
                                 ///< translate determined transformation before
                                 ///< returning it

protected:
    ///< map of marker id to some general
    ///< parameters like offsets
    std::map<int, FiducialPiParameters> m_general_fiducial_parameters;

    // width of current image divided by reference
    // width of 640px
    double m_image_size_factor;

    struct SharpnessLogData
    {
        int pixel_count;
        double distance_to_camera;
        double sharpness_score;
    };

    ///< if true, the sharpness
    ///< measurements are logged
    ///< and saved to disc for
    ///< calibration of the curve
    ///< or directly calibrated
    ///< within the program
    bool m_log_or_calibrate_sharpness_measurements;
    ///< structure for logging measured data
    std::vector<SharpnessLogData> m_log_data;

private:
    bool ProjectionValid(cv::Mat& rot_CfromO, cv::Mat& trans_CfromO,
                         cv::Mat& camera_matrix, cv::Mat& pattern_coords,
                         cv::Mat& image_coords);

    std::vector<t_pi> m_ref_tag_vec; ///< reference tags to be recognized
    cv::Mat m_debug_img;             ///< image that holds debugging output
    bool m_use_fast_pi_tag;
};

} // end namespace ipa_Fiducials
