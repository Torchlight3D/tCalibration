#include "pitagdetector.h"

#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <tCore/Math>

#include "ellipserefine.h"

namespace tl {
namespace {
bool TagUnique(const std::vector<t_pi>& detections, const t_pi& newTag)
{
    // Insert if not already existing
    bool duplicate = true;
    for (const auto& detection : detections) {
        duplicate = true;
        for (int j = 0; j < 12; j++) {
            if (detection.image_points[j].center !=
                newTag.image_points[j].center) {
                duplicate = false;
                break;
            }
        }
        if (duplicate) {
            return false;
        }
    }
    return true;
}

bool AnglesValid2D(const std::vector<cv::RotatedRect>& image_points)
{
    // Check angles
    const auto vec_03 = cv::normalize(
        cv::Vec2f{image_points[3].center - image_points[0].center});
    const auto vec_36 = cv::normalize(
        cv::Vec2f{image_points[6].center - image_points[3].center});
    const auto vec_69 = cv::normalize(
        cv::Vec2f{image_points[9].center - image_points[6].center});
    const auto vec_90 = cv::normalize(
        cv::Vec2f{image_points[0].center - image_points[9].center});

    float angle_ur = math::radToDeg(std::acos(-vec_03.dot(vec_36)));
    float angle_lr = math::radToDeg(std::acos(-vec_36.dot(vec_69)));
    float angle_ll = math::radToDeg(std::acos(-vec_69.dot(vec_90)));
    float angle_ul = math::radToDeg(std::acos(-vec_90.dot(vec_03)));

    // float max_symtry_deg_diff = 40;
    // if (std::abs(angle_ur - angle_ll) > max_symtry_deg_diff ||
    //     std::abs(angle_ul - angle_lr) > max_symtry_deg_diff) {
    //     return false;
    // }

    float min_deg_angle = 20;
    if (std::abs(angle_ur) < min_deg_angle ||
        std::abs(angle_lr) < min_deg_angle ||
        std::abs(angle_ll) < min_deg_angle ||
        std::abs(angle_ul) < min_deg_angle) {
        return false;
    }

    return true;
}

} // namespace

bool FiducialModelPi::Init(cv::Mat& camera_matrix,
                           std::string directory_and_filename,
                           bool log_or_calibrate_sharpness_measurements,
                           cv::Mat extrinsic_matrix)
{
    if (!SetExtrinsics(camera_matrix, extrinsic_matrix)) {
        return false;
    }

    m_log_or_calibrate_sharpness_measurements =
        log_or_calibrate_sharpness_measurements;

    return LoadParameters(directory_and_filename);
}

bool FiducialModelPi::SetExtrinsics(cv::Mat& cameraMatrix,
                                    cv::Mat extrinsic_matrix)
{
    // TODO: Check extrinsic matrix dim 3x4
    if (cameraMatrix.empty()) {
        LOG(ERROR) << "Camera matrix not initialized";
        return false;
    }

    m_camera_matrix = cameraMatrix.clone();

    m_extrinsic_XYfromC = cv::Mat::eye(4, 4, CV_64FC1);
    if (!extrinsic_matrix.empty()) {
        m_extrinsic_XYfromC(cv::Range{0, 3}, cv::Range{0, 4}) =
            extrinsic_matrix;
    }

    return true;
}

bool FiducialModelPi::ApplyExtrinsics(cv::Mat& rot_CfromO,
                                      cv::Mat& trans_CfromO)
{
    // Copy ORIGINAL rotation and translation to frame
    cv::Mat frame_CfromO = cv::Mat::eye(4, 4, CV_64FC1);
    frame_CfromO(cv::Range{0, 3}, cv::Range{0, 3}) = rot_CfromO;
    frame_CfromO(cv::Range{0, 3}, cv::Range{3, 4}) = trans_CfromO;

    cv::Mat frame_XYfromO = m_extrinsic_XYfromC * frame_CfromO;

    // Copy MODIFIED rotation and translation to frame

    rot_CfromO = frame_XYfromO(cv::Range{0, 3}, cv::Range{0, 3});
    trans_CfromO = frame_XYfromO(cv::Range{0, 3}, cv::Range{3, 4});

    return true;
}

bool FiducialModelPi::GetSharpnessMeasure(
    const cv::Mat& image, t_pose pose_CfromO,
    const FiducialPiParameters& fiducial_parameters, double& sharpness_measure,
    double sharpness_calibration_parameter_m,
    double sharpness_calibration_parameter_n)
{
    if (fiducial_parameters.m_id == -1) {
        LOG(ERROR)
            << "Could not find general fiducial parameters to the provided id.";
        return false;
    }

    // 1. select image region for sharpness analysis
    cv::Mat point3d_marker = cv::Mat::zeros(4, 3, CV_64FC1);
    point3d_marker.at<double>(0, 0) =
        fiducial_parameters.m_sharpness_pattern_area_rect3d.x +
        fiducial_parameters.m_offset.x; // upper left
    point3d_marker.at<double>(0, 1) =
        -fiducial_parameters.m_sharpness_pattern_area_rect3d.y +
        fiducial_parameters.m_offset.y;
    point3d_marker.at<double>(1, 0) =
        fiducial_parameters.m_sharpness_pattern_area_rect3d.x +
        fiducial_parameters.m_sharpness_pattern_area_rect3d.width +
        fiducial_parameters.m_offset.x; // upper right
    point3d_marker.at<double>(1, 1) =
        -fiducial_parameters.m_sharpness_pattern_area_rect3d.y +
        fiducial_parameters.m_offset.y;
    point3d_marker.at<double>(2, 0) =
        fiducial_parameters.m_sharpness_pattern_area_rect3d.x +
        fiducial_parameters.m_sharpness_pattern_area_rect3d.width +
        fiducial_parameters.m_offset.x; // lower right
    point3d_marker.at<double>(2, 1) =
        -fiducial_parameters.m_sharpness_pattern_area_rect3d.y -
        fiducial_parameters.m_sharpness_pattern_area_rect3d.height +
        fiducial_parameters.m_offset.y;
    point3d_marker.at<double>(3, 0) =
        fiducial_parameters.m_sharpness_pattern_area_rect3d.x +
        fiducial_parameters.m_offset.x; // lower left
    point3d_marker.at<double>(3, 1) =
        -fiducial_parameters.m_sharpness_pattern_area_rect3d.y -
        fiducial_parameters.m_sharpness_pattern_area_rect3d.height +
        fiducial_parameters.m_offset.y;

    std::vector<cv::Point> sharpness_area(4);
    cv::Point min_point(image.cols - 1, image.rows - 1);
    cv::Point max_point(0, 0);
    for (int i = 0; i < 4; ++i) {
        cv::Mat point3d_camera =
            pose_CfromO.rot * point3d_marker.row(i).t() + pose_CfromO.trans;
        cv::Mat point2d_camera = m_camera_matrix * point3d_camera;
        sharpness_area[i].x = std::max(
            0, std::min(image.cols - 1, cvRound(point2d_camera.at<double>(0) /
                                                point2d_camera.at<double>(2))));
        sharpness_area[i].y = std::max(
            0, std::min(image.rows - 1, cvRound(point2d_camera.at<double>(1) /
                                                point2d_camera.at<double>(2))));

        min_point.x = std::min(min_point.x, sharpness_area[i].x);
        min_point.y = std::min(min_point.y, sharpness_area[i].y);
        max_point.x = std::max(max_point.x, sharpness_area[i].x);
        max_point.y = std::max(max_point.y, sharpness_area[i].y);
    }

    cv::Mat roi = image(cv::Range{min_point.y, max_point.y},
                        cv::Range{min_point.x, max_point.x});

    // 2. compute sharpness measure
    cv::Mat gray_image;
    {
        cv::Mat _;
        cv::cvtColor(roi, _, cv::COLOR_BGR2GRAY);
        cv::normalize(_, gray_image, 0., 255., cv::NORM_MINMAX);
    }

    // cv::imshow("gray_image", gray_image);
    // cv::Mat image_copy = image.clone();
    // for (int i = 0; i < 4; ++i) {
    //     cv::line(image_copy, sharpness_area[i], sharpness_area[(i + 1) % 4],
    //              CV_RGB(0, 255, 0), 2);
    // }

    // map sharpness_area into the roi
    for (auto& point : sharpness_area) {
        point -= min_point;
    }

    // variant M_V (std. dev. of gray values)
    std::vector<uchar> gray_values;
    double avg_gray = 0.;
    for (int v = 0; v < roi.rows; ++v) {
        for (int u = 0; u < roi.cols; ++u) {
            if (cv::pointPolygonTest(sharpness_area, cv::Point2i(u, v), false) >
                0) {
                uchar val = gray_image.at<uchar>(v, u);
                gray_values.push_back(val);
                avg_gray += (double)val;
            }
        }
    }

    int pixel_count = (int)gray_values.size();
    if (pixel_count > 0) {
        avg_gray /= (double)pixel_count;
    }

    double sharpness_score = 0.;
    for (int i = 0; i < pixel_count; ++i) {
        sharpness_score += (double)((int)gray_values[i] - (int)avg_gray) *
                           ((int)gray_values[i] - (int)avg_gray);
    }

    //		double m = 9139.749632393357;	// these numbers come from measuring
    // pixel_count and sharpness_score in all possible situations and
    // interpolating a function (here: a linear function y=m*x+n) with that data
    //		double n = -2670187.875850272;

    // how far is the score  from the linear  sharpness function
    sharpness_measure = std::min(
        1., sharpness_score / (sharpness_calibration_parameter_m * pixel_count +
                               sharpness_calibration_parameter_n));

    // if (m_log_or_calibrate_sharpness_measurements) {
    //     SharpnessLogData log;
    //     log.distance_to_camera = cv::norm(pose_CfromO.trans, cv::NORM_L2);
    //     log.pixel_count = pixel_count;
    //     log.sharpness_score = sharpness_score;
    //     m_log_data.push_back(log);

    //     cv::imshow("image", image);
    //     int key = cv::waitKey(10);
    //     if (key == 's') {
    //         std::ofstream file("sharpness_log_file.txt", std::ios::out);
    //         if (!file.is_open())
    //             LOG(ERROR) << "Could not open file.";
    //         else {
    //             for (unsigned int i = 0; i < m_log_data.size(); ++i) {
    //                 file << m_log_data[i].pixel_count << "\t"
    //                      << m_log_data[i].distance_to_camera << "\t"
    //                      << m_log_data[i].sharpness_score << "\n";
    //             }
    //             file.close();
    //             LOG(INFO) << "All data successfully written to disk.";
    //         }
    //     }
    //     else if (key == 'c') {
    //         // compute linear regression for calibration curve and
    //         outputresult
    //         // on screen i.e. the m and n parameters come from measuring
    //         // pixel_count and sharpness_score in all possible situations and
    //         // interpolating a function (here: a linear function y=m*x+n)
    //         with
    //         // that data
    //         cv::Mat A = cv::Mat::ones(int(m_log_data.size()), 2, CV_64FC1);
    //         cv::Mat b(int(m_log_data.size()), 1, CV_64FC1);
    //         for (unsigned int i = 0; i < m_log_data.size(); ++i) {
    //             A.at<double>(i, 0) = m_log_data[i].pixel_count;
    //             b.at<double>(i) = m_log_data[i].sharpness_score;
    //         }
    //         cv::Mat line_parameters;
    //         cv::solve(A, b, line_parameters, cv::DECOMP_QR);

    //         std::cout
    //             << "The line parameters for the sharpness measure
    //             calibration"
    //                "curve are:\n  m = "
    //             << std::setprecision(15) << line_parameters.at<double>(0)
    //             << "\n  n = " << line_parameters.at<double>(1)
    //             << "\n\nPress any key to record further data and calibrate "
    //                "again with the present the additional data.\n"
    //             << std::endl;
    //         cv::waitKey();
    //     }
    // }

    // cv::imshow("sharpness area", image_copy);
    // cv::waitKey(10);

    return true;
}

cv::Mat FiducialModelPi::GetCameraMatrix() const { return m_camera_matrix; }

void FiducialModelPi::SetCameraMatrix(cv::Mat camera_matrix)
{
    m_camera_matrix = camera_matrix.clone();
}

cv::Mat FiducialModelPi::GetDistortionCoeffs() const
{
    if (m_dist_coeffs.empty())
        return cv::Mat::zeros(1, 4, CV_64FC1);
    return m_dist_coeffs;
}

void FiducialModelPi::SetDistortionCoeffs(cv::Mat dist_coeffs)
{
    m_dist_coeffs = dist_coeffs.clone();
}

FiducialPiParameters FiducialModelPi::GetGeneralFiducialParameters(
    int marker_id) const
{
    if (m_general_fiducial_parameters.contains(marker_id)) {
        return m_general_fiducial_parameters.at(marker_id);
    }

    FiducialPiParameters empty;
    empty.m_id = -1;
    return empty;
}

FiducialModelPi::FiducialModelPi() {}

FiducialModelPi::~FiducialModelPi() {}

bool FiducialModelPi::GetPoints(cv::Mat& image,
                                std::vector<t_points>& vec_points)
{
    // express relative to a 640x480 pixels camera image
    m_image_size_factor = image.cols * 1. / 640.0;

    cv::Mat src_mat_8U1;

    constexpr bool debug = false;
    if constexpr (debug) {
        m_debug_img = image.clone();
    }

    // ------------ Convert image to gray scale if necessary -------------------
    if (image.channels() == 3) {
        src_mat_8U1.create(image.rows, image.cols, CV_8UC1);
        cv::cvtColor(image, src_mat_8U1, cv::COLOR_RGB2GRAY);
    }
    else {
        src_mat_8U1 = image;
    }

    cv::Mat grayscale_image = src_mat_8U1.clone();

    // ------------ Filtering --------------------------------------------------
    if (false) {
        // Divide the image by its morphologically closed counterpart
        cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(19, 19));
        cv::Mat closed;
        cv::morphologyEx(src_mat_8U1, closed, cv::MORPH_CLOSE, kernel);

        // divide requires floating-point
        src_mat_8U1.convertTo(src_mat_8U1, CV_32F);
        cv::divide(src_mat_8U1, closed, src_mat_8U1, 1, CV_32F);
        cv::normalize(src_mat_8U1, src_mat_8U1, 0, 255, cv::NORM_MINMAX);
        // convert back to unsigned int
        src_mat_8U1.convertTo(src_mat_8U1, CV_8UC1);
    }

    // ------------ Adaptive thresholding --------------------------------------

    int minus_c = 21;
    // express relative to a 640x480 pixels camera
    // image
    int half_kernel_size = 20 * m_image_size_factor;

    if (m_use_fast_pi_tag) {
        minus_c = 11;
        half_kernel_size = 5;
    }

    cv::adaptiveThreshold(src_mat_8U1, src_mat_8U1, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                          2 * half_kernel_size + 1, minus_c);

    // ------------ Contour extraction --------------------------------------
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(src_mat_8U1, contours, cv::RETR_LIST,
                     cv::CHAIN_APPROX_NONE);

    if (debug) {
        cv::Mat contour_image = m_debug_img;
        for (size_t i = 0; i < contours.size(); i++) {
            cv::drawContours(contour_image, contours, (int)i, CV_RGB(255, 0, 0),
                             1, cv::LINE_8);
        }
    }

    // ------------ Ellipse extraction --------------------------------------
    // Min ellipse size at 70cm distance is 20x20 pixels
    int min_ellipse_size = 3;
    // int min_contour_points = int(1.5 * min_ellipse_size);
    int max_ellipse_aspect_ratio = 7;

    if (m_use_fast_pi_tag) {
        min_ellipse_size = 5;
        max_ellipse_aspect_ratio = 15;
    }

    std::vector<cv::RotatedRect> ellipses;
    for (const auto& contour : contours) {
        if (contour.size() < 6) {
            continue;
        }

        cv::Mat pointsf;
        cv::Mat(contour).convertTo(pointsf, CV_32F);
        cv::RotatedRect box = cv::fitEllipse(pointsf);

        // Plausibility checks
        double box_max = std::max(box.size.width, box.size.height);
        double box_min = std::min(box.size.width, box.size.height);
        if (box_max > box_min * max_ellipse_aspect_ratio)
            continue;
        if (box_max > std::min(src_mat_8U1.rows, src_mat_8U1.cols) * 0.2)
            continue;
        if (box_min < 0.5 * min_ellipse_size)
            continue;
        if (box_max < min_ellipse_size)
            continue;
        if (box.center.x < 0 || box.center.x >= src_mat_8U1.cols ||
            box.center.y < 0 || box.center.y >= src_mat_8U1.rows)
            continue;

        bool add_ellipse = true;

        if (m_use_fast_pi_tag) {
            double ellipse_aspect_ratio = box.size.height / box.size.width;
            if (box.size.height > box.size.width) {
                ellipse_aspect_ratio = 1 / ellipse_aspect_ratio;
            }
            if (box.size.area() > 200 && ellipse_aspect_ratio < 0.1) {
                continue;
            }

            // order ellipses in ascending order with respect to size
            for (size_t j = 0; j < ellipses.size() && add_ellipse; j++) {
                if (box.size.area() > ellipses[j].size.area()) {
                    ellipses.insert(ellipses.begin() + j, box);
                    add_ellipse = false;
                }
            }
        }
        else {
            // Check for double borders on circles and keep only larger ones
            for (auto& ellipse : ellipses) {
                double dist_thresh = box_min * 0.1;
                // TOOD: Use normL1
                double dist = std::abs(box.center.x - ellipse.center.x) +
                              std::abs(box.center.y - ellipse.center.y);
                if (dist < dist_thresh) {
                    add_ellipse = false;
                    ellipse = box;
                    break;
                }
            }
        }

        if (add_ellipse) {
            ellipses.push_back(box);
        }
    }

    // Fast Pi Tag
    std::vector<cv::Point> points;
    std::vector<cv::Rect> rois;
    // Fil cv::Mat with -1
    cv::Mat ellipsevoting = cv::Mat::ones(src_mat_8U1.size(), CV_32FC1) * -1.f;
    cv::Mat ellipsedensity = cv::Mat::zeros(src_mat_8U1.size(), CV_8UC1);

    if (m_use_fast_pi_tag) {
        std::vector<size_t> badellipses;

        // ellipse density voting
        for (const auto ellipse : ellipses) {
            unsigned int votingsize =
                (int)std::min(src_mat_8U1.rows, src_mat_8U1.cols) * 0.005;
            unsigned int vr_x = votingsize;
            unsigned int vr_y = votingsize;

            for (int k = -(int)vr_x / 2; k < (int)vr_x / 2; k++) {
                for (int l = -(int)vr_y / 2; l < (int)vr_y / 2; l++) {
                    int x = ellipse.center.x + l;
                    int y = ellipse.center.y + k;

                    // Border Overshoot
                    if (x >= src_mat_8U1.cols || x < 0)
                        continue;
                    if (y >= src_mat_8U1.rows || y < 0)
                        continue;

                    ellipsedensity.at<unsigned char>(y, x) =
                        ellipsedensity.at<unsigned char>(y, x) + 1;
                }
            }
        }

        // check if a ellipse is already in the same place - > Take bigger
        // one(ellipses are order in ascending order) if(false) -> vote
        for (size_t i = 0; i < ellipses.size(); i++) {
            // unsigned int min =
            // std::min(ellipses[i].size.height,ellipses[i].size.width); voting
            // area
            unsigned int votingsize =
                (int)std::min(src_mat_8U1.rows, src_mat_8U1.cols) * 0.003;
            unsigned int vr_x = votingsize;
            unsigned int vr_y = votingsize;

            bool insert = true;

            for (int k = -(int)vr_x / 2; k < (int)vr_x / 2; k++) {
                for (int l = -(int)vr_y / 2; l < (int)vr_y / 2; l++) {
                    int x = ellipses[i].center.x + l;
                    int y = ellipses[i].center.y + k;

                    // Border Overshoot
                    if (x >= src_mat_8U1.cols || x < 0)
                        continue;
                    if (y >= src_mat_8U1.rows || y < 0)
                        continue;

                    if (ellipsevoting.at<float>(y, x) != -1) {
                        insert = false;
                    }
                }
            }

            if (insert) {
                for (int k = -(int)vr_x / 2; k < (int)vr_x / 2; k++) {
                    for (int l = -(int)vr_y / 2; l < (int)vr_y / 2; l++) {
                        int x = ellipses[i].center.x + l;
                        int y = ellipses[i].center.y + k;

                        // Border Overshoot
                        if (x >= src_mat_8U1.cols || x < 0)
                            continue;
                        if (y >= src_mat_8U1.rows || y < 0)
                            continue;

                        ellipsevoting.at<float>(y, x) = (float)i;
                    }
                }
            }
            else {
                badellipses.push_back(i);
            }
        }

        // store points with high ellipse density
        for (int c = 4; (points.empty() || c >= 3) && c >= 2; c--) {
            for (int i = 0; i < ellipsedensity.rows; i++) {
                for (int j = 0; j < ellipsedensity.cols; j++) {
                    if (ellipsedensity.at<unsigned char>(i, j) >= c) {
                        cv::Point2i point;
                        point.x = j;
                        point.y = i;
                        points.push_back(point);
                    }
                }
            }
        }

        // kick points which are too close together
        int max_distance =
            (int)std::max(src_mat_8U1.rows, src_mat_8U1.cols) * 0.15;
        for (size_t i{0}; i < points.size(); i++) {
            for (auto j = i + 1; j < points.size(); j++) {
                if (int dist = (int)cv::norm(points[i] - points[j]);
                    dist < max_distance) {
                    points.erase(points.begin() + j);
                    j--;
                }
            }
        }

        // choose size of ROI
        for (const auto& point : points) {
            // Find ellipse with smallest distance to point[i]
            unsigned int id = 0;
            double min_dist =
                cv::norm(cv::Point2f(point) - ellipses[id].center);

            for (size_t j = 1; j < ellipses.size(); j++) {
                if (double dist =
                        cv::norm(cv::Point2f(point) - ellipses[j].center);
                    dist < min_dist) {
                    min_dist = dist;
                    id = j;
                }
            }

            // compute roi and save in rois
            int side = (int)std::max(src_mat_8U1.cols, src_mat_8U1.rows) * 0.2;

            cv::Point topleft = point - cv::Point{side, side};

            // keep image borders
            topleft.x = std::clamp(topleft.x, 0, src_mat_8U1.cols - 2 * side);
            topleft.y = std::clamp(topleft.y, 0, src_mat_8U1.rows - 2 * side);

            // Shrink rois to max number of  ellipses
            std::vector<cv::RotatedRect> ellipses_roi(ellipses);
            std::vector<cv::RotatedRect> ellipses_roi_tmp;

            // TODO This values are only  valid when used with the Kinect in
            // FullHd mode
            // TODO Update is necessary to make it work with all cameras ..
            // based on ellipse density in image
            // TODO See Diploma Thesis: Robust Object Detection Using Fiducial
            // Markers from Matthias Nösner
            unsigned int max_ellipses_roi = 0.05 * ellipses.size();
            max_ellipses_roi = std::clamp(max_ellipses_roi, 150u, 450u);
            // unsigned int max_ellipses_roi = 150;
            int stepsize = 2;

            while (ellipses_roi.size() > max_ellipses_roi) {
                ellipses_roi_tmp.clear();
                for (const auto& ellipse : ellipses_roi) {
                    if (topleft.x < ellipse.center.x &&
                        ellipse.center.x < topleft.x + 2 * side &&
                        topleft.y < ellipse.center.y &&
                        ellipse.center.y < topleft.y + 2 * side) {
                        ellipses_roi_tmp.push_back(ellipse);
                    }
                }

                ellipses_roi.clear();
                for (const auto& ellipse : ellipses_roi_tmp) {
                    ellipses_roi.push_back(ellipse);
                }

                topleft.x += stepsize;
                topleft.y += stepsize;
                side -= stepsize;
            }

            // shrink as long a no ellipses are lost
            bool shrink = true;
            while (shrink) {
                topleft.x += stepsize;
                topleft.y += stepsize;
                side -= stepsize;

                ellipses_roi_tmp.clear();

                for (const auto& ellipse : ellipses_roi) {
                    if (topleft.x < ellipse.center.x &&
                        ellipse.center.x < topleft.x + 2 * side &&
                        topleft.y < ellipse.center.y &&
                        ellipse.center.y < topleft.y + 2 * side) {
                        ellipses_roi_tmp.push_back(ellipse);
                    }
                }

                // detect loosing ellipses
                if (ellipses_roi_tmp.size() < ellipses_roi.size()) {
                    shrink = false;
                    topleft.x -= stepsize;
                    topleft.y -= stepsize;
                    side += stepsize;
                }

                ellipses_roi.clear();
                for (const auto& ellipse : ellipses_roi_tmp) {
                    ellipses_roi.push_back(ellipse);
                }
            }

            // rois have the same id like the points in vector "points"
            rois.push_back(
                cv::Rect(topleft.x, topleft.y, (int)2 * side, (int)2 * side));

            if (debug) {
                rectangle(m_debug_img,
                          cv::Rect(topleft.x, topleft.y, (int)2 * side,
                                   (int)2 * side),
                          CV_RGB(255, 255, 255));
            }
        }

        // Kick ellipses in inverted order
        while (!badellipses.empty()) {
            size_t index = badellipses.back();
            badellipses.pop_back();
            ellipses.erase(ellipses.begin() + index);
        }
    }

    if (debug) {
        cv::Mat ellipse_image = m_debug_img;
        for (const auto& ellipse : ellipses) {
            cv::ellipse(ellipse_image, ellipse, CV_RGB(0, 255, 0), 1,
                        cv::LINE_AA);
            cv::ellipse(ellipse_image, ellipse.center, ellipse.size * 0.5f,
                        ellipse.angle, 0, 360, CV_RGB(255, 255, 0), 1,
                        cv::LINE_AA);
        }

        // Fast Pi Tag
        // Make them white for visualization
        if (m_use_fast_pi_tag) {
            cv::Mat ellipsedensity_img =
                cv::Mat::zeros(src_mat_8U1.size(), CV_8UC1);

            // draw points
            for (const auto& point : points) {
                ellipsedensity_img.at<uchar>(point) = 255;
            }

            // draw rois
            for (size_t i = 0; i < rois.size(); i++) {
                // cv::imshow("Roi:", m_debug_img(rois[i]));
            }

            // TODO: Use cv::Mat operation
            for (int i = 0; i < ellipsevoting.rows; i++) {
                for (int j = 0; j < ellipsevoting.cols; j++) {
                    if (ellipsevoting.at<float>(i, j) > -1) {
                        ellipsevoting.at<float>(i, j) = 255;
                    }
                }
            }

            // cv::imshow("80 Ellipsevoting", ellipsevoting);
            // cv::imshow("90 Ellipsedensity", ellipsedensity_img);
        }

        // cv::imshow("40 Ellipses", ellipse_image);
        // cv::waitKey(0);
    }

    // PITAG: For PITAG the loop is executed only once with all ellipses in the
    // cloud FASTPITAG: For FASTPITAG the number of loop excecutions is based on
    // the number of rois
    std::vector<cv::RotatedRect> ellipses_copy(ellipses);
    bool once = true;

    for (size_t n = 0; n < points.size() || (!m_use_fast_pi_tag && once);
         n++) { // Each point is the center of a roi
        once = false;

        if (m_use_fast_pi_tag) {
            ellipses.clear();
            // prepare ellipse cloud for marker detection
            for (const auto& ellipse : ellipses_copy) {
                if (rois[n].x < ellipse.center.x &&
                    ellipse.center.x < rois[n].x + rois[n].width &&
                    rois[n].y < ellipse.center.y &&
                    ellipse.center.y < rois[n].y + rois[n].height) {
                    // ellipses size is 10*smaller than roi
                    double factor = 0.05;
                    if ((int)ellipse.size.width * ellipse.size.height <
                        (int)rois[n].width * rois[n].height * factor) {
                        ellipses.push_back(ellipse);
                    }
                }
            }
        }

        // ------------ Fiducial corner extraction
        // --------------------------------------
        std::vector<std::vector<cv::RotatedRect>> marker_lines;
        double max_pixel_dist_to_line;         // Will be set automatically
        double max_ellipse_difference;         // Will be set automatically
        double deviation_of_aspectratio = 0.3; // m_use_fast_pi_tag

        // Compute area
        std::vector<double> ref_A;
        std::vector<double> ref_Ratio;
        for (const auto& ellipse : ellipses) {
            ref_A.push_back(std::max(ellipse.size.width, ellipse.size.height));
        }

        // Fast PiTag
        if (m_use_fast_pi_tag) {
            for (const auto& ellipse : ellipses) {
                double ellipse_aspect_ratio =
                    ellipse.size.height / ellipse.size.width;
                if (ellipse.size.height > ellipse.size.width) {
                    ellipse_aspect_ratio = 1 / ellipse_aspect_ratio;
                }
                ref_Ratio.push_back(ellipse_aspect_ratio);
            }
        }

        for (size_t i{0}; i < ellipses.size(); i++) {
            for (auto j = i + 1; j < ellipses.size(); j++) {
                // Fast Pi Tag
                if (m_use_fast_pi_tag) {
                    if (std::abs(ref_Ratio[i] - ref_Ratio[j]) >
                        deviation_of_aspectratio) {
                        continue;
                    }
                }

                // Check area
                max_ellipse_difference = 0.5 * std::min(ref_A[i], ref_A[j]);
                if (std::abs(ref_A[i] - ref_A[j]) > max_ellipse_difference) {
                    continue;
                }

                // Compute line equation
                cv::Point2f vec_IJ = ellipses[j].center - ellipses[i].center;
                double dot_IJ_IJ = vec_IJ.ddot(vec_IJ);

                // Check all other ellipses if they fit to the line equation
                // Condition: Between two points are at most two other points
                // Not more and not less
                std::vector<cv::RotatedRect> line_candidate;
                int nLine_Candidates = 0;

                for (size_t k = 0; k < ellipses.size() && nLine_Candidates < 2;
                     k++) {
                    // Fast Pi Tag
                    if (m_use_fast_pi_tag) {
                        if (std::abs(ref_Ratio[j] - ref_Ratio[k]) >
                            deviation_of_aspectratio) {
                            continue;
                        }
                    }

                    // Check area
                    max_ellipse_difference = 0.5 * std::min(ref_A[j], ref_A[k]);
                    if (std::abs(ref_A[j] - ref_A[k]) >
                        max_ellipse_difference) {
                        continue;
                    }

                    if (k == i || k == j) {
                        continue;
                    }

                    // Check if k lies on the line between i and j
                    cv::Point2f vec_IK =
                        ellipses[k].center - ellipses[i].center;
                    double t_k = vec_IK.ddot(vec_IJ) / dot_IJ_IJ;
                    if (t_k < 0 || t_k > 1) {
                        continue;
                    }

                    // Check distance to line
                    cv::Point2f proj_k = ellipses[i].center + vec_IJ * t_k;
                    const cv::Point2f vec_KprojK = proj_k - ellipses[k].center;
                    auto d_k_sqr = cv::normL2Sqr<float>(vec_KprojK);

                    max_pixel_dist_to_line = std::sqrt(std::min(
                        ellipses[k].size.height, ellipses[k].size.width));
                    max_pixel_dist_to_line =
                        std::max(2., max_pixel_dist_to_line);
                    if (d_k_sqr >
                        max_pixel_dist_to_line * max_pixel_dist_to_line) {
                        continue;
                    }

                    for (auto l = k + 1;
                         l < ellipses.size() && nLine_Candidates < 2; l++) {
                        // Fast Pi Tag
                        if (m_use_fast_pi_tag) {
                            if (std::abs(ref_Ratio[k] - ref_Ratio[l]) >
                                deviation_of_aspectratio) {
                                continue;
                            }
                        }

                        // Check area
                        max_ellipse_difference =
                            0.5 * std::min(ref_A[k], ref_A[l]);
                        if (std::abs(ref_A[k] - ref_A[l]) >
                            max_ellipse_difference) {
                            continue;
                        }

                        if (l == i || l == j) {
                            continue;
                        }

                        // Check if l lies on the line between i and j
                        cv::Point2f vec_IL =
                            ellipses[l].center - ellipses[i].center;
                        double t_l = vec_IL.ddot(vec_IJ) / dot_IJ_IJ;
                        if (t_l < 0 || t_l > 1) {
                            continue;
                        }

                        // Check distance to line
                        cv::Point2f proj_l = ellipses[i].center + vec_IJ * t_l;
                        cv::Point2f vec_LprojL = proj_l - ellipses[l].center;
                        auto d_l_sqr = cv::normL2Sqr<float>(vec_LprojL);

                        max_pixel_dist_to_line = std::sqrt(std::min(
                            ellipses[l].size.height, ellipses[l].size.width));
                        max_pixel_dist_to_line =
                            std::max(2., max_pixel_dist_to_line);
                        if (d_l_sqr >
                            max_pixel_dist_to_line * max_pixel_dist_to_line) {
                            continue;
                        }

                        // Yeah, we found 4 fitting points
                        line_candidate.push_back(ellipses[i]);
                        if (t_k < t_l) {
                            line_candidate.push_back(ellipses[k]);
                            line_candidate.push_back(ellipses[l]);
                        }
                        else {
                            line_candidate.push_back(ellipses[l]);
                            line_candidate.push_back(ellipses[k]);
                        }
                        line_candidate.push_back(ellipses[j]);
                        nLine_Candidates++;
                    }
                }

                // See condition above
                if (nLine_Candidates == 1) {
                    marker_lines.push_back(line_candidate);
                }
            }
        }

        if (debug) {
            // cv::Mat line_image = cv::Mat::zeros(src_mat_8U1.size(), CV_8UC3);
            cv::Mat line_image = m_debug_img.clone();
            for (const auto& rrects : marker_lines) {
                cv::line(line_image, rrects[0].center, rrects[3].center,
                         CV_RGB(255, 255, 0), 1, cv::LINE_8);
            }
            // cv::imshow("50 Lines", line_image);
            // cv::waitKey(0);
        }

        // ------------ Fiducial line association
        // --------------------------------------
        double cross_ratio_max_dist = 0.03;
        std::vector<t_pi> final_tag_vec;

        for (auto& refTag : m_ref_tag_vec) {
            refTag.fitting_image_lines_0.clear();
            refTag.fitting_image_lines_1.clear();
        }

        for (const auto& rrects : marker_lines) {
            // Cross ratio i
            double l_AB = cv::norm(rrects[1].center - rrects[0].center);
            double l_BD = cv::norm(rrects[3].center - rrects[1].center);
            double l_AC = cv::norm(rrects[2].center - rrects[0].center);
            double l_CD = cv::norm(rrects[3].center - rrects[2].center);
            double cross_ratio_i = (l_AB / l_BD) / (l_AC / l_CD);

            // Associate lines to markers based on their cross ratio
            for (auto& refTag : m_ref_tag_vec) {
                if (std::abs(cross_ratio_i - refTag.cross_ration_0) <
                    cross_ratio_max_dist) {
                    refTag.fitting_image_lines_0.push_back(rrects);
                }
                else if (std::abs(cross_ratio_i - refTag.cross_ration_1) <
                         cross_ratio_max_dist) {
                    refTag.fitting_image_lines_1.push_back(rrects);
                }
            }
        }

        if (debug) {
            // cv::Mat line_image = cv::Mat::zeros(src_mat_8U1.size(), CV_8UC3);
            cv::Mat line_image = m_debug_img.clone();
            for (const auto& refTag : m_ref_tag_vec) {
                for (const auto& rrects : refTag.fitting_image_lines_0) {
                    cv::line(line_image, rrects[0].center, rrects[3].center,
                             CV_RGB(0, 255, 255), 1, cv::LINE_8);
                }

                for (const auto& rrects : refTag.fitting_image_lines_1) {
                    cv::line(line_image, rrects[0].center, rrects[3].center,
                             CV_RGB(255, 0, 255), 1, cv::LINE_8);
                }
            }

            // cv::imshow("51 Valid Lines", line_image);
            // cv::waitKey(0);
        }

        // Search for all tag types independently
        for (auto& refTag : m_ref_tag_vec) {
            std::vector<t_pi> ul_tag_vec;
            std::vector<t_pi> lr_tag_vec;

            // Take into account that multi associations from one line to many
            // others may occure
            std::vector<std::vector<int>> ul_idx_lines_0(
                refTag.fitting_image_lines_0.size(), std::vector<int>());
            std::vector<std::vector<int>> lr_idx_lines_1(
                refTag.fitting_image_lines_1.size(), std::vector<int>());

            /// Upper left
            // Check for a common upper left corner cross_ratio = largest
            for (size_t j{0}; j < refTag.fitting_image_lines_0.size(); j++) {
                for (auto k = j + 1; k < refTag.fitting_image_lines_0.size();
                     k++) {
                    auto& rrect_j = refTag.fitting_image_lines_0[j];
                    auto& rrect_k = refTag.fitting_image_lines_0[k];

                    bool corners_are_matching = false;
                    bool reorder_j = false;
                    bool reorder_k = false;
                    if (rrect_j[0].center == rrect_k[0].center) {
                        corners_are_matching = true;
                    }
                    else if (rrect_j[3].center == rrect_k[0].center) {
                        corners_are_matching = true;
                        reorder_j = true;
                    }
                    else if (rrect_j[0].center == rrect_k[3].center) {
                        corners_are_matching = true;
                        reorder_k = true;
                    }
                    else if (rrect_j[3].center == rrect_k[3].center) {
                        corners_are_matching = true;
                        reorder_j = true;
                        reorder_k = true;
                    }

                    if (!corners_are_matching)
                        continue;

                    // Index 0 should corresponds to the common corner
                    if (reorder_j) {
                        cv::RotatedRect tmp = rrect_j[3];
                        rrect_j[3] = rrect_j[0];
                        rrect_j[0] = tmp;
                        tmp = rrect_j[2];
                        rrect_j[2] = rrect_j[1];
                        rrect_j[1] = tmp;
                    }
                    if (reorder_k) {
                        cv::RotatedRect tmp = rrect_k[3];
                        rrect_k[3] = rrect_k[0];
                        rrect_k[0] = tmp;
                        tmp = rrect_k[2];
                        rrect_k[2] = rrect_k[1];
                        rrect_k[1] = tmp;
                    }

                    // Compute angular ordering (clockwise)
                    cv::Point2f tag_corner0 = rrect_j[3].center;
                    cv::Point2f tag_corner1 = rrect_k[3].center;
                    cv::Point2f tag_cornerUL = rrect_j[0].center;
                    cv::Point2f tag_center =
                        tag_corner1 + 0.5 * (tag_corner0 - tag_corner1);

                    cv::Point2f vec_center_cUL = tag_cornerUL - tag_center;
                    cv::Point2f vec_center_c0 = tag_corner0 - tag_center;
                    cv::Point2f vec_center_c1 = tag_corner1 - tag_center;

                    double sign_c0 = vec_center_cUL.cross(vec_center_c0);
                    double sign_c1 = vec_center_cUL.cross(vec_center_c1);
                    // One must be positive and the other negative
                    // Otherwise the two lines are collinear
                    if (sign_c0 * sign_c1 >= 0) {
                        continue;
                    }

                    int idx0 = j;
                    int idx1 = k;
                    if (sign_c0 > 0) {
                        idx0 = k;
                        idx1 = j;
                    }

                    t_pi tag;
                    tag.image_points =
                        std::vector<cv::RotatedRect>(12, cv::RotatedRect());
                    tag.image_points[0] = refTag.fitting_image_lines_0[idx1][0];
                    tag.image_points[1] = refTag.fitting_image_lines_0[idx1][1];
                    tag.image_points[2] = refTag.fitting_image_lines_0[idx1][2];
                    tag.image_points[3] = refTag.fitting_image_lines_0[idx1][3];

                    tag.image_points[9] = refTag.fitting_image_lines_0[idx0][3];
                    tag.image_points[10] =
                        refTag.fitting_image_lines_0[idx0][2];
                    tag.image_points[11] =
                        refTag.fitting_image_lines_0[idx0][1];

                    ul_idx_lines_0[j].push_back(int(ul_tag_vec.size()));
                    ul_idx_lines_0[k].push_back(int(ul_tag_vec.size()));
                    ul_tag_vec.push_back(tag);
                }
            }

            /// Lower right
            // Check for a common lower right corner cross_ratio = lowest
            for (size_t j{0}; j < refTag.fitting_image_lines_1.size(); j++) {
                for (auto k = j + 1; k < refTag.fitting_image_lines_1.size();
                     k++) {
                    auto& rrect_j = refTag.fitting_image_lines_1[j];
                    auto& rrect_k = refTag.fitting_image_lines_1[k];

                    bool corners_are_matching = false;
                    bool reorder_j = false;
                    bool reorder_k = false;
                    if (rrect_j[0].center == rrect_k[0].center) {
                        corners_are_matching = true;
                    }
                    else if (rrect_j[3].center == rrect_k[0].center) {
                        corners_are_matching = true;
                        reorder_j = true;
                    }
                    else if (rrect_j[0].center == rrect_k[3].center) {
                        corners_are_matching = true;
                        reorder_k = true;
                    }
                    else if (rrect_j[3].center == rrect_k[3].center) {
                        corners_are_matching = true;
                        reorder_j = true;
                        reorder_k = true;
                    }

                    if (!corners_are_matching) {
                        continue;
                    }

                    // Index 0 should corresponds to the common corner
                    if (reorder_j) {
                        cv::RotatedRect tmp = rrect_j[3];
                        rrect_j[3] = rrect_j[0];
                        rrect_j[0] = tmp;
                        tmp = rrect_j[2];
                        rrect_j[2] = rrect_j[1];
                        rrect_j[1] = tmp;
                    }
                    if (reorder_k) {
                        cv::RotatedRect tmp = rrect_k[3];
                        rrect_k[3] = rrect_k[0];
                        rrect_k[0] = tmp;
                        tmp = rrect_k[2];
                        rrect_k[2] = rrect_k[1];
                        rrect_k[1] = tmp;
                    }

                    // Compute angular ordering (clockwise)
                    cv::Point2f tag_corner0 = rrect_j[3].center;
                    cv::Point2f tag_corner1 = rrect_k[3].center;
                    cv::Point2f tag_cornerUL = rrect_j[0].center;
                    cv::Point2f tag_center =
                        tag_corner1 + 0.5 * (tag_corner0 - tag_corner1);

                    cv::Point2f vec_center_cUL = tag_cornerUL - tag_center;
                    cv::Point2f vec_center_c0 = tag_corner0 - tag_center;
                    cv::Point2f vec_center_c1 = tag_corner1 - tag_center;

                    // Angle from cUL to c0 is negative if sign is positive and
                    // vice versa
                    double sign_c0 = vec_center_cUL.cross(vec_center_c0);
                    double sign_c1 = vec_center_cUL.cross(vec_center_c1);
                    // One must be positive and the other negative
                    // Otherwise the two lines are collinear
                    if (sign_c0 * sign_c1 >= 0) {
                        continue;
                    }

                    int idx0 = j;
                    int idx1 = k;
                    if (sign_c0 > 0) {
                        idx0 = k;
                        idx1 = j;
                    }

                    t_pi tag;
                    tag.image_points =
                        std::vector<cv::RotatedRect>(12, cv::RotatedRect());
                    tag.image_points[6] = refTag.fitting_image_lines_1[idx1][0];
                    tag.image_points[7] = refTag.fitting_image_lines_1[idx1][1];
                    tag.image_points[8] = refTag.fitting_image_lines_1[idx1][2];
                    tag.image_points[9] = refTag.fitting_image_lines_1[idx1][3];

                    tag.image_points[3] = refTag.fitting_image_lines_1[idx0][3];
                    tag.image_points[4] = refTag.fitting_image_lines_1[idx0][2];
                    tag.image_points[5] = refTag.fitting_image_lines_1[idx0][1];

                    lr_idx_lines_1[j].push_back(int(lr_tag_vec.size()));
                    lr_idx_lines_1[k].push_back(int(lr_tag_vec.size()));
                    lr_tag_vec.push_back(tag);
                }
            }

            /// Lower left or Upper right
            // Check for a common lower left or upper right corner Now, lines
            // could already participate in matchings of ul and lr corners
            // cross_ratio = different
            for (size_t j{0}; j < refTag.fitting_image_lines_0.size(); ++j) {
                for (size_t k{0}; k < refTag.fitting_image_lines_1.size();
                     ++k) {
                    auto& rrects0 = refTag.fitting_image_lines_0[j];
                    auto& rrect1 = refTag.fitting_image_lines_1[k];

                    bool corners_are_matching = false;
                    bool reorder_j = false;
                    bool reorder_k = false;
                    if (rrects0[0].center == rrect1[0].center) {
                        corners_are_matching = true;
                    }
                    else if (rrects0[3].center == rrect1[0].center) {
                        corners_are_matching = true;
                        reorder_j = true;
                    }
                    else if (rrects0[0].center == rrect1[3].center) {
                        corners_are_matching = true;
                        reorder_k = true;
                    }
                    else if (rrects0[3].center == rrect1[3].center) {
                        corners_are_matching = true;
                        reorder_j = true;
                        reorder_k = true;
                    }

                    if (!corners_are_matching) {
                        continue;
                    }

                    // Index 0 should corresponds to the common corner
                    if (reorder_j) {
                        cv::RotatedRect tmp = rrects0[3];
                        rrects0[3] = rrects0[0];
                        rrects0[0] = tmp;
                        tmp = rrects0[2];
                        rrects0[2] = rrects0[1];
                        rrects0[1] = tmp;
                    }
                    if (reorder_k) {
                        cv::RotatedRect tmp = rrect1[3];
                        rrect1[3] = rrect1[0];
                        rrect1[0] = tmp;
                        tmp = rrect1[2];
                        rrect1[2] = rrect1[1];
                        rrect1[1] = tmp;
                    }

                    // Compute angular ordering (clockwise)
                    cv::Point2f tag_corner0 = rrects0[3].center;
                    cv::Point2f tag_corner1 = rrect1[3].center;
                    cv::Point2f tag_cornerUL = rrects0[0].center;
                    cv::Point2f tag_center =
                        tag_corner1 + 0.5 * (tag_corner0 - tag_corner1);

                    cv::Point2f vec_center_cUL = tag_cornerUL - tag_center;
                    cv::Point2f vec_center_c0 = tag_corner0 - tag_center;
                    cv::Point2f vec_center_c1 = tag_corner1 - tag_center;

                    double sign_c0 = vec_center_cUL.cross(vec_center_c0);
                    double sign_c1 = vec_center_cUL.cross(vec_center_c1);
                    // One must be positive and the other negative
                    // Otherwise the two lines are collinear
                    if (sign_c0 * sign_c1 >= 0) {
                        continue;
                    }

                    t_pi tag;
                    tag.image_points =
                        std::vector<cv::RotatedRect>(12, cv::RotatedRect());

                    if (sign_c0 > 0) {
                        // Lower left cornerfinal_tag_vec
                        tag.image_points =
                            std::vector<cv::RotatedRect>(12, cv::RotatedRect());
                        tag.image_points[9] = rrects0[0];
                        tag.image_points[10] = rrects0[1];
                        tag.image_points[11] = rrects0[2];
                        tag.image_points[0] = rrects0[3];

                        tag.image_points[6] = rrect1[3];
                        tag.image_points[7] = rrect1[2];
                        tag.image_points[8] = rrect1[1];

                        // Check if lines participated already in a matching
                        if (ul_idx_lines_0[j].empty() &&
                            lr_idx_lines_1[k].empty()) {
                            if (TagUnique(final_tag_vec, tag)) {
                                refTag.sparse_copy_to(tag);
                                tag.no_matching_lines = 2;
                                final_tag_vec.push_back(tag);
                            }
                        }
                        else if (!ul_idx_lines_0[j].empty() &&
                                 lr_idx_lines_1[k].empty()) {
                            for (const auto& ul : ul_idx_lines_0[j]) {
                                t_pi final_tag;
                                final_tag.image_points = tag.image_points;

                                // Add matching line segment to final tag
                                final_tag.image_points[1] =
                                    ul_tag_vec[ul].image_points[1];
                                final_tag.image_points[2] =
                                    ul_tag_vec[ul].image_points[2];
                                final_tag.image_points[3] =
                                    ul_tag_vec[ul].image_points[3];

                                if (TagUnique(final_tag_vec, final_tag)) {
                                    refTag.sparse_copy_to(final_tag);
                                    final_tag.no_matching_lines = 3;
                                    final_tag_vec.push_back(final_tag);
                                }
                            }
                        }
                        else if (ul_idx_lines_0[j].empty() &&
                                 !lr_idx_lines_1[k].empty()) {
                            for (const auto& lr : lr_idx_lines_1[k]) {
                                t_pi final_tag;
                                final_tag.image_points = tag.image_points;

                                // Add matching line segment to final tag
                                final_tag.image_points[3] =
                                    lr_tag_vec[lr].image_points[3];
                                final_tag.image_points[4] =
                                    lr_tag_vec[lr].image_points[4];
                                final_tag.image_points[5] =
                                    lr_tag_vec[lr].image_points[5];

                                if (TagUnique(final_tag_vec, final_tag)) {
                                    refTag.sparse_copy_to(final_tag);
                                    final_tag.no_matching_lines = 3;
                                    final_tag_vec.push_back(final_tag);
                                }
                            }
                        }
                        else if (!ul_idx_lines_0[j].empty() &&
                                 !lr_idx_lines_1[k].empty()) {
                            // YEAH buddy. You've got a complete matching
                            for (const auto& ul : ul_idx_lines_0[j]) {
                                for (const auto& lr : lr_idx_lines_1[k]) {
                                    t_pi final_tag;
                                    final_tag.image_points = tag.image_points;

                                    // Add matching line segment from ul to
                                    // final tag
                                    final_tag.image_points[1] =
                                        ul_tag_vec[ul].image_points[1];
                                    final_tag.image_points[2] =
                                        ul_tag_vec[ul].image_points[2];
                                    final_tag.image_points[3] =
                                        ul_tag_vec[ul].image_points[3];

                                    // Add matching line segment from lr to
                                    // final tag
                                    final_tag.image_points[4] =
                                        lr_tag_vec[lr].image_points[4];
                                    final_tag.image_points[5] =
                                        lr_tag_vec[lr].image_points[5];

                                    // Check consistency
                                    if (ul_tag_vec[ul].image_points[3].center !=
                                            lr_tag_vec[lr]
                                                .image_points[3]
                                                .center ||
                                        ul_tag_vec[ul].image_points[9].center !=
                                            lr_tag_vec[lr]
                                                .image_points[9]
                                                .center)
                                        continue;

                                    if (!TagUnique(final_tag_vec, final_tag))
                                        continue;

                                    if (AnglesValid2D(final_tag.image_points)) {
                                        refTag.sparse_copy_to(final_tag);
                                        final_tag.no_matching_lines = 4;
                                        final_tag_vec.push_back(final_tag);
                                    }
                                }
                            }
                        }
                    }
                    else {
                        // Upper right corner
                        tag.image_points =
                            std::vector<cv::RotatedRect>(12, cv::RotatedRect());
                        tag.image_points[0] = rrects0[3];
                        tag.image_points[1] = rrects0[2];
                        tag.image_points[2] = rrects0[1];
                        tag.image_points[3] = rrects0[0];

                        tag.image_points[4] = rrect1[1];
                        tag.image_points[5] = rrect1[2];
                        tag.image_points[6] = rrect1[3];

                        // Check if lines participated already in a matching
                        if (ul_idx_lines_0[j].empty() &&
                            lr_idx_lines_1[k].empty()) {
                            if (TagUnique(final_tag_vec, tag)) {
                                refTag.sparse_copy_to(tag);
                                tag.no_matching_lines = 2;
                                final_tag_vec.push_back(tag);
                            }
                        }
                        else if (!ul_idx_lines_0[j].empty() &&
                                 lr_idx_lines_1[k].empty()) {
                            for (const auto& ul : ul_idx_lines_0[j]) {
                                t_pi final_tag;
                                final_tag.image_points = tag.image_points;

                                // Add matching line segment to final tag
                                final_tag.image_points[9] =
                                    ul_tag_vec[ul].image_points[9];
                                final_tag.image_points[10] =
                                    ul_tag_vec[ul].image_points[10];
                                final_tag.image_points[11] =
                                    ul_tag_vec[ul].image_points[11];

                                if (TagUnique(final_tag_vec, final_tag)) {
                                    refTag.sparse_copy_to(final_tag);
                                    final_tag.no_matching_lines = 3;
                                    final_tag_vec.push_back(final_tag);
                                }
                            }
                        }
                        else if (ul_idx_lines_0[j].empty() &&
                                 !lr_idx_lines_1[k].empty()) {
                            for (const auto& lr : lr_idx_lines_1[k]) {
                                t_pi final_tag;
                                final_tag.image_points = tag.image_points;

                                // Add matching line segment to final tag
                                final_tag.image_points[7] =
                                    lr_tag_vec[lr].image_points[7];
                                final_tag.image_points[8] =
                                    lr_tag_vec[lr].image_points[8];
                                final_tag.image_points[9] =
                                    lr_tag_vec[lr].image_points[9];

                                if (TagUnique(final_tag_vec, final_tag)) {
                                    refTag.sparse_copy_to(final_tag);
                                    final_tag.no_matching_lines = 3;
                                    final_tag_vec.push_back(final_tag);
                                }
                            }
                        }
                        else if (!ul_idx_lines_0[j].empty() &&
                                 !lr_idx_lines_1[k].empty()) {
                            // YEAH buddy. You've got a complete matching
                            for (const auto& ul : ul_idx_lines_0[j]) {
                                for (const auto& lr : lr_idx_lines_1[k]) {
                                    t_pi final_tag;
                                    final_tag.image_points = tag.image_points;

                                    // Add matching line segment from ul to
                                    // final tag
                                    final_tag.image_points[9] =
                                        ul_tag_vec[ul].image_points[9];
                                    final_tag.image_points[10] =
                                        ul_tag_vec[ul].image_points[10];
                                    final_tag.image_points[11] =
                                        ul_tag_vec[ul].image_points[11];

                                    // Add matching line segment from lr to
                                    // final tag
                                    final_tag.image_points[7] =
                                        lr_tag_vec[lr].image_points[7];
                                    final_tag.image_points[8] =
                                        lr_tag_vec[lr].image_points[8];

                                    // Check consistency
                                    if (ul_tag_vec[ul].image_points[3].center !=
                                            lr_tag_vec[lr]
                                                .image_points[3]
                                                .center ||
                                        ul_tag_vec[ul].image_points[9].center !=
                                            lr_tag_vec[lr]
                                                .image_points[9]
                                                .center)
                                        continue;

                                    if (!TagUnique(final_tag_vec, final_tag))
                                        continue;

                                    if (AnglesValid2D(final_tag.image_points)) {
                                        refTag.sparse_copy_to(final_tag);
                                        final_tag.no_matching_lines = 4;
                                        final_tag_vec.push_back(final_tag);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // ------------ Refine ellipses ------------------------------------
        int min_matching_lines = 4;

        int sobel_winsize = 3;
        float gauss_smooth_sigma = 3.0;

        if (sobel_winsize % 2 == 0) {
            sobel_winsize++;
            LOG(ERROR) << " Sobel winsize changed to " << sobel_winsize
                       << " (must be odd)";
        }

        // Sobel
        cv::Mat input_image_smooth;
        if (gauss_smooth_sigma > 0.0f) {
            cv::GaussianBlur(grayscale_image, input_image_smooth, cv::Size(),
                             gauss_smooth_sigma, gauss_smooth_sigma);
        }
        else {
            input_image_smooth = grayscale_image.clone();
        }
        cv::Mat gradient_x(input_image_smooth.rows, input_image_smooth.cols,
                           CV_16S);
        cv::Mat gradient_y(input_image_smooth.rows, input_image_smooth.cols,
                           CV_16S);
        cv::Sobel(input_image_smooth, gradient_x, CV_16S, 1, 0, sobel_winsize);
        cv::Sobel(input_image_smooth, gradient_y, CV_16S, 0, 1, sobel_winsize);

        cv::Mat refine_image = image.clone();
        for (auto& detection : final_tag_vec) {
            if (detection.no_matching_lines < min_matching_lines) {
                continue;
            }

            for (auto& rrect : detection.image_points) {
                cv::Matx33d refined_ellipse;
                if (cv::runetag::ipa_Fiducials::ellipserefine(
                        rrect, gradient_x, gradient_y, refined_ellipse)) {
                    cv::Point2d ellipse_center =
                        cv::runetag::ipa_Fiducials::ellipseCenter(
                            refined_ellipse);

                    cv::circle(refine_image, rrect.center, 1,
                               CV_RGB(0, 255, 0));
                    cv::circle(refine_image, ellipse_center, 1,
                               CV_RGB(0, 0, 255));

                    rrect.center.x = ellipse_center.x;
                    rrect.center.y = ellipse_center.y;
                }
                else {
                    LOG(ERROR) << "Ellipse refine failed";
                }
            }
        }

        if (debug) {
            cv::Mat tag_image = image.clone();
            const std::array rgbValVec{
                cv::Vec3b(0, 0, 0), cv::Vec3b(255, 255, 255),
                cv::Vec3b(255, 0, 0), cv::Vec3b(0, 255, 255),
                cv::Vec3b(0, 255, 0)};
            for (const auto& detection : final_tag_vec) {
                if (detection.no_matching_lines != 4) {
                    continue;
                }

                bool connect_points = false;
                for (size_t j = 0; j < detection.image_points.size(); j++) {
                    cv::Vec3b rgbVal = rgbValVec[detection.no_matching_lines];
                    if (detection.image_points[j].center.x != 0) {
                        cv::circle(tag_image, detection.image_points[j].center,
                                   3, CV_RGB(0, 255, 0), -1, cv::LINE_AA);
                        if (connect_points) {
                            cv::line(tag_image,
                                     detection.image_points[j - 1].center,
                                     detection.image_points[j].center, rgbVal,
                                     1, cv::LINE_AA);
                        }
                        connect_points = true;
                    }
                    else {
                        connect_points = false;
                    }
                }
            }
            // cv::imshow("60 Tags", tag_image);
        }

        for (const auto& detection : final_tag_vec) {
            if (detection.no_matching_lines < min_matching_lines) {
                continue;
            }

            t_points tag_points;
            tag_points.id = detection.parameters.m_id;
            tag_points.marker_points = std::vector<cv::Point2f>();
            tag_points.image_points = std::vector<cv::Point2f>();

            for (size_t j = 0; j < detection.image_points.size(); j++) {
                tag_points.marker_points.push_back(detection.marker_points[j]);
                tag_points.image_points.push_back(
                    detection.image_points[j].center);
            }

            vec_points.push_back(tag_points);
        }
    }

    return !vec_points.empty();
}

bool FiducialModelPi::GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose)
{
    std::vector<t_points> vec_points;
    if (!GetPoints(image, vec_points)) {
        return false;
    }

    // ------------ Compute pose --------------------------------------

    for (const auto& points : vec_points) {
        int nPoints = std::ranges::count_if(
            points.image_points,
            [](const cv::Point2f& pt) { return pt.x != 0.f; });

        cv::Mat pattern_coords(nPoints, 3, CV_32F);
        cv::Mat image_coords(nPoints, 2, CV_32F);

        float* p_pattern_coords = 0;
        float* p_image_coords = 0;
        int idx = 0;
        for (const auto& pt : points.image_points) {
            if (pt.x != 0) {
                p_pattern_coords = pattern_coords.ptr<float>(idx);
                p_pattern_coords[0] = pt.x;
                p_pattern_coords[1] = pt.y;
                p_pattern_coords[2] = 0;

                p_image_coords = image_coords.ptr<float>(idx);
                p_image_coords[0] = pt.x;
                p_image_coords[1] = pt.y;

                idx++;
            }
        }

        t_pose tag_pose;
        tag_pose.id = points.id;
        cv::solvePnP(pattern_coords, image_coords, GetCameraMatrix(),
                     GetDistortionCoeffs(), tag_pose.rot, tag_pose.trans);

        // Apply transformation
        cv::Mat rmat_CfromO;
        cv::Rodrigues(tag_pose.rot, rmat_CfromO);

        cv::Mat reprojection_matrix = GetCameraMatrix();
        if (!ProjectionValid(rmat_CfromO, tag_pose.trans, reprojection_matrix,
                             pattern_coords, image_coords)) {
            continue;
        }

        ApplyExtrinsics(rmat_CfromO, tag_pose.trans);
        rmat_CfromO.copyTo(tag_pose.rot);
        vec_pose.push_back(tag_pose);
    }

    // Fast Pi Tag
    if (m_use_fast_pi_tag) {
        // Maximum detection distance
        double max_detection_distance = 5.1;
        for (size_t h = 0; h < vec_pose.size(); h++) {
            if (vec_pose[h].trans.at<double>(2) > max_detection_distance) {
                vec_pose.erase(vec_pose.begin() + h);
                h--;
            }
        }

        // Kick double detected Markers
        // minimum distance between two markers
        double min_marker_distance = 0.01;
        for (size_t h{0}; h < vec_pose.size(); h++) {
            for (auto b = h + 1; b < vec_pose.size(); b++) {
                if (vec_pose[h].id == vec_pose[b].id) {
                    double distance_between_markers =
                        cv::sqrt(vec_pose[h].trans.at<double>(0) *
                                     vec_pose[b].trans.at<double>(0) +
                                 vec_pose[h].trans.at<double>(1) *
                                     vec_pose[b].trans.at<double>(1) +
                                 vec_pose[h].trans.at<double>(2) *
                                     vec_pose[b].trans.at<double>(2));

                    if (distance_between_markers > min_marker_distance) {
                        vec_pose.erase(vec_pose.begin() + b);
                        b--;
                    }
                }
            }
        }
    }

    return !vec_points.empty();
}

bool FiducialModelPi::ProjectionValid(cv::Mat& rmat_CfromO,
                                      cv::Mat& tvec_CfromO,
                                      cv::Mat& camera_matrix, cv::Mat& pts_in_O,
                                      cv::Mat& image_coords)
{
    // express relative to a 640x480 pixels camera image
    double max_avg_pixel_error = 5 * m_image_size_factor;

    // Check angles
    float* p_pts_in_O = 0;
    double* p_pt_in_O = 0;
    double* p_pt_4x1_in_C = 0;
    double* p_pt_3x1_in_C = 0;
    double* p_pt_3x1_2D = 0;
    float* p_image_coords;

    cv::Mat pt_in_O(4, 1, CV_64FC1);
    p_pt_in_O = pt_in_O.ptr<double>(0);
    cv::Mat pt_4x1_in_C;
    cv::Mat pt_3x1_in_C(3, 1, CV_64FC1);
    p_pt_3x1_in_C = pt_3x1_in_C.ptr<double>(0);
    cv::Mat pt_3x1_2D(3, 1, CV_64FC1);
    p_pt_3x1_2D = pt_3x1_2D.ptr<double>(0);

    // Create 4x4 frame CfromO
    cv::Mat frame_CfromO = cv::Mat::eye(4, 4, CV_64FC1);
    frame_CfromO(cv::Range{0, 3}, cv::Range{0, 3}) = rmat_CfromO;
    frame_CfromO(cv::Range{0, 3}, cv::Range{3, 4}) = tvec_CfromO;

    // Check reprojection error
    double dist = 0;
    for (int i = 0; i < pts_in_O.rows; i++) {
        p_image_coords = image_coords.ptr<float>(i);
        p_pts_in_O = pts_in_O.ptr<float>(i);

        p_pt_in_O[0] = p_pts_in_O[0];
        p_pt_in_O[1] = p_pts_in_O[1];
        p_pt_in_O[2] = p_pts_in_O[2];
        p_pt_in_O[3] = 1;

        cv::Mat pt_4x1_in_C = frame_CfromO * pt_in_O;
        p_pt_4x1_in_C = pt_4x1_in_C.ptr<double>(0);
        p_pt_3x1_in_C[0] = p_pt_4x1_in_C[0] / p_pt_4x1_in_C[3];
        p_pt_3x1_in_C[1] = p_pt_4x1_in_C[1] / p_pt_4x1_in_C[3];
        p_pt_3x1_in_C[2] = p_pt_4x1_in_C[2] / p_pt_4x1_in_C[3];

        pt_3x1_2D = camera_matrix * pt_3x1_in_C;
        pt_3x1_2D /= p_pt_3x1_2D[2];

        dist = std::sqrt((p_pt_3x1_2D[0] - p_image_coords[0]) *
                             (p_pt_3x1_2D[0] - p_image_coords[0]) +
                         (p_pt_3x1_2D[1] - p_image_coords[1]) *
                             (p_pt_3x1_2D[1] - p_image_coords[1]));

        if (dist > max_avg_pixel_error) {
            return false;
        }
    }

    return true;
}

bool FiducialModelPi::LoadParameters(std::vector<FiducialPiParameters> pi_tags)
{
    m_ref_tag_vec.clear();
    for (const auto& param : pi_tags) {
        t_pi ref_tag;
        double tag_size = param.line_width_height;

        ref_tag.parameters = param;

        double d_line0_AB = tag_size * param.d_line0_AB;            // AB
        double d_line0_BD = tag_size - tag_size * param.d_line0_AB; // BD
        double d_line0_AC = tag_size * param.d_line0_AC;            // AC
        double d_line0_CD = tag_size - tag_size * param.d_line0_AC; // CD
        ref_tag.cross_ration_0 =
            (d_line0_AB / d_line0_BD) / (d_line0_AC / d_line0_CD);

        double d_line1_AB = tag_size * param.d_line1_AB;
        double d_line1_BD = tag_size - tag_size * param.d_line1_AB;
        double d_line1_AC = tag_size * param.d_line1_AC;
        double d_line1_CD = tag_size - tag_size * param.d_line1_AC;
        ref_tag.cross_ration_1 =
            (d_line1_AB / d_line1_BD) / (d_line1_AC / d_line1_CD);

        // Marker coordinates
        ref_tag.marker_points.push_back(cv::Point2f(0, -0));
        ref_tag.marker_points.push_back(cv::Point2f(float(d_line0_AB), -0));
        ref_tag.marker_points.push_back(cv::Point2f(float(d_line0_AC), -0));
        ref_tag.marker_points.push_back(cv::Point2f(float(tag_size), -0));
        ref_tag.marker_points.push_back(
            cv::Point2f(float(tag_size), -float(d_line1_AB)));
        ref_tag.marker_points.push_back(
            cv::Point2f(float(tag_size), -float(d_line1_AC)));
        ref_tag.marker_points.push_back(
            cv::Point2f(float(tag_size), -float(tag_size)));
        ref_tag.marker_points.push_back(
            cv::Point2f(float(d_line1_AC), -float(tag_size)));
        ref_tag.marker_points.push_back(
            cv::Point2f(float(d_line1_AB), -float(tag_size)));
        ref_tag.marker_points.push_back(cv::Point2f(0, -float(tag_size)));
        ref_tag.marker_points.push_back(cv::Point2f(0, -float(d_line0_AC)));
        ref_tag.marker_points.push_back(cv::Point2f(0, -float(d_line0_AB)));

        // Offset
        for (unsigned int j = 0; j < ref_tag.marker_points.size(); j++) {
            ref_tag.marker_points[j].x += param.m_offset.x;
            ref_tag.marker_points[j].y += param.m_offset.y;
        }

        double delta = ref_tag.cross_ration_0 / ref_tag.cross_ration_1;
        if (std::abs(delta - 1) < 0.05) {
            LOG(WARNING) << "Skipping fiducial due to equal cross ratios";
        }
        else if (delta < 1) {
            LOG(WARNING) << "Skipping fiducial " << ref_tag.parameters.m_id
                         << " due to cross ratios";
            LOG(WARNING) << "Cross ratio 0 must be larger than cross ratio 1";
        }
        else {
            m_ref_tag_vec.push_back(ref_tag);
        }

        if (m_ref_tag_vec.empty()) {
            LOG(ERROR) << "No valid fiducials loaded";
            return false;
        }
    }
    return true;
}

bool FiducialModelPi::LoadParameters(std::string directory_and_filename)
{
    // ...
    return false;
}

} // namespace tl
