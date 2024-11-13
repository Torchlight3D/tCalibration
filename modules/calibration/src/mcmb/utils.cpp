#include "utils.h"

#include <numeric>
#include <random>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/imgproc.hpp>

namespace tl::mcmb {

double median(std::vector<double> &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

double calcLinePara(const std::vector<cv::Point2f> &points, double &a,
                    double &b, double &c)
{
    std::vector<cv::Point2f> points_f;
    points_f.reserve(points.size());
    for (const auto &pt : points) {
        points_f.emplace_back(pt);
    }

    // Direction + Point on line
    cv::Vec4f line;
    cv::fitLine(points_f, line, cv::DIST_L2, 0., 1e-2, 1e-2);
    a = line[1];
    b = -line[0];
    c = line[0] * line[3] - line[1] * line[2];

    auto res{0.};
    for (const auto &pt : points) {
        res += std::abs(pt.x * a + pt.y * b + c);
    }
    res /= points.size();

    return res;
}

cv::Mat RT2Proj(cv::InputArray rmat, cv::InputArray tvec)
{
    cv::Mat T = cv::Mat_<double>::eye(4, 4);
    rmat.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
    tvec.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));
    return T;
}

cv::Mat RVecT2Proj(cv::InputArray rvec, cv::InputArray tvec)
{
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    return RT2Proj(rmat, tvec);
}

cv::Mat RVecT2ProjInt(cv::InputArray _rvec, cv::InputArray _tvec,
                      cv::InputArray _K)
{
    CV_Assert(_K.cols() == 3 && _K.rows() == 3);

    cv::Mat rmat;
    cv::Rodrigues(_rvec, rmat);

    const auto K = _K.getMat();
    cv::Mat KR = K * rmat;
    cv::Mat Kt = K * _tvec.getMat();

    cv::Mat P = cv::Mat_<double>::eye(4, 4);
    KR.copyTo(P(cv::Range(0, 3), cv::Range(0, 3)));
    Kt.copyTo(P(cv::Range(0, 3), cv::Range(3, 4)));
    return P;
}

void Proj2RT(cv::InputArray _T, cv::OutputArray _rvec, cv::OutputArray _tvec)
{
    const auto T = _T.getMat();
    cv::Rodrigues(T(cv::Range(0, 3), cv::Range(0, 3)), _rvec);
    T(cv::Range(0, 3), cv::Range(3, 4)).copyTo(_tvec);
}

void invertRvecT(cv::InputArray rvec, cv::InputArray tvec,
                 cv::OutputArray rvec_inv, cv::OutputArray tvec_inv)
{
    Proj2RT(RVecT2Proj(rvec, tvec).inv(), rvec_inv, tvec_inv);
}

void invertRvecT(cv::InputOutputArray rvec, cv::InputOutputArray tvec)
{
    Proj2RT(RVecT2Proj(rvec, tvec).inv(), rvec, tvec);
}

cv::Mat vectorProj(std::vector<float> ProjV)
{
    cv::Mat rvec(1, 3, CV_64F);
    rvec.at<double>(0) = (double)ProjV[0];
    rvec.at<double>(1) = (double)ProjV[1];
    rvec.at<double>(2) = (double)ProjV[2];

    cv::Mat tvec(3, 1, CV_64F);
    tvec.at<double>(0) = (double)ProjV[3];
    tvec.at<double>(1) = (double)ProjV[4];
    tvec.at<double>(2) = (double)ProjV[5];

    return RVecT2Proj(rvec, tvec);
}

// Projection matrix to float array
std::array<float, 6> ProjToVec(cv::InputArray Proj)
{
    cv::Mat rvec, tvec;
    Proj2RT(Proj, rvec, tvec);
    return {(float)rvec.at<double>(0), (float)rvec.at<double>(1),
            (float)rvec.at<double>(2), (float)tvec.at<double>(0),
            (float)tvec.at<double>(1), (float)tvec.at<double>(2)};
}

std::vector<cv::Point3f> transformPoints(
    const std::vector<cv::Point3f> &point3s, cv::Mat rvec, cv::Mat tvec)
{
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    double r11 = rmat.at<double>(0, 0);
    double r12 = rmat.at<double>(0, 1);
    double r13 = rmat.at<double>(0, 2);
    double r21 = rmat.at<double>(1, 0);
    double r22 = rmat.at<double>(1, 1);
    double r23 = rmat.at<double>(1, 2);
    double r31 = rmat.at<double>(2, 0);
    double r32 = rmat.at<double>(2, 1);
    double r33 = rmat.at<double>(2, 2);
    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);

    std::vector<cv::Point3f> newPoint3s;
    newPoint3s.reserve(point3s.size());
    for (const auto &point3 : point3s) {
        float x = tx + r11 * point3.x + r12 * point3.y + r13 * point3.z;
        float y = ty + r21 * point3.x + r22 * point3.y + r23 * point3.z;
        float z = tz + r31 * point3.x + r32 * point3.y + r33 * point3.z;
        newPoint3s.emplace_back(x, y, z);
    }

    return newPoint3s;
}

void projectPointsWithDistortion(cv::InputArray objectPoints,
                                 cv::InputArray rvec, cv::InputArray tvec,
                                 cv::InputArray cameraMatrix,
                                 cv::InputArray distortion,
                                 cv::OutputArray imagePoints, int type)
{
    switch (type) {
        case 0:
            cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                              distortion, imagePoints);
            break;
        case 1:
            cv::fisheye::projectPoints(objectPoints, imagePoints, rvec, tvec,
                                       cameraMatrix, distortion, 0.);
            break;
        default:
            CV_Error(cv::Error::StsBadFlag, "Unsupported camera model!");
            break;
    }
}

cv::Point3f triangulatePointNViews(const std::vector<cv::Point2f> &points,
                                   const std::vector<cv::Mat> &rvecs,
                                   const std::vector<cv::Mat> &tvecs,
                                   cv::InputArray cameraMatrix)
{
    // TODO: Check empty is too easy
    CV_Assert(!points.empty() && points.size() == rvecs.size() &&
              points.size() == tvecs.size());

    cv::Mat A;
    for (size_t i{0}; i < points.size(); i++) {
        const auto P = RVecT2ProjInt(rvecs[i], tvecs[i], cameraMatrix);
        const auto M3 = P.row(2);
        A.push_back(points[i].x * M3 - P.row(0));
        A.push_back(points[i].y * M3 - P.row(1));
    }

    cv::Mat W, U, VT;
    cv::SVDecomp(A, W, U, VT);

    const cv::Mat lastRow = VT.row(3);
    lastRow.convertTo(lastRow, CV_32F);
    cv::Point3f point3;
    point3.x = lastRow.at<float>(0) / lastRow.at<float>(3);
    point3.y = lastRow.at<float>(1) / lastRow.at<float>(3);
    point3.z = lastRow.at<float>(2) / lastRow.at<float>(3);
    return point3;
}

cv::Point3f triangulatePointNViewsRansac(
    const std::vector<cv::Point2f> &point2s, const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs, cv::InputArray cameraMatrix,
    cv::InputArray distortion, double threshold, double confidence,
    int iterations)
{
    // TODO: Check empty is too easy
    CV_Assert(!point2s.empty() && point2s.size() == rvecs.size() &&
              point2s.size() == tvecs.size());

    // Init parameters
    int N = iterations;
    int trialcount = 0;
    int countit = 0;

    const auto numPoints = point2s.size();

    // Indices to shuffle
    std::vector<size_t> indices(numPoints);
    std::iota(indices.begin(), indices.end(), size_t(0));

    std::random_device rd{};
    std::mt19937_64 rng{rd()};

    // Ransac loop
    std::vector<size_t> inliers;
    cv::Point3f point3{};
    while (N > trialcount && countit < iterations) {
        // Randomly pick 2 points
        std::ranges::shuffle(indices, rng);

        const std::vector _point2s{point2s[indices[0]], point2s[indices[1]]};
        const std::vector _rvecs{rvecs[indices[0]], rvecs[indices[1]]};
        const std::vector _tvecs{tvecs[indices[0]], tvecs[indices[1]]};
        const cv::Point3f _point3 =
            triangulatePointNViews(_point2s, _rvecs, _tvecs, cameraMatrix);

        // Compute inliers
        std::vector<size_t> _inliers;
        for (size_t k{0}; k < numPoints; k++) {
            const std::vector point3s{_point3};

            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(point3s, rvecs[k], tvecs[k], cameraMatrix,
                              distortion, projectedPoints);
            if (cv::norm(point2s[k] - projectedPoints[0]) < threshold) {
                _inliers.push_back(k);
            }
        }
        trialcount++;

        // Keep the best one
        if (_inliers.size() > inliers.size()) {
            _inliers.swap(inliers);
            point3 = _point3;

            // with probability p, a data set with no outliers.
            constexpr auto myepsilon = 1e-5;

            const double fracinliers =
                static_cast<double>(inliers.size()) / numPoints;
            double pNoOutliers = 1 - std::pow(fracinliers, 3);
            if (pNoOutliers == 0)
                pNoOutliers = myepsilon;
            if (pNoOutliers > (1 - myepsilon))
                pNoOutliers = 1 - myepsilon;
            double tempest = std::log(1 - confidence) / std::log(pNoOutliers);
            N = int(std::round(tempest));
            trialcount = 0;
        }
        countit++;
    }

    return point3;
}

cv::Mat _solveP3PRansac(const std::vector<cv::Point3f> &objectPoints,
                        const std::vector<cv::Point2f> &imagePoints,
                        cv::InputArray cameraMatrix, cv::InputArray distortion,
                        cv::OutputArray _rvec, cv::OutputArray _tvec,
                        float threshold, int iterations, double confidence,
                        bool refine)
{
    // Init parameters
    int N = iterations;
    int trialcount = 0;
    int countit = 0;
    cv::Mat rvec(1, 3, CV_64F);
    cv::Mat tvec(1, 3, CV_64F);

    const auto numPoints = objectPoints.size();

    // Indices to shuffle
    std::vector<size_t> indices(numPoints);
    std::iota(indices.begin(), indices.end(), size_t(0));
    std::random_device rd{};
    std::mt19937_64 rng{rd()};

    // Ransac loop
    std::vector<int> _inliers;
    while (N > trialcount && countit < iterations) {
        // Randomly pick 4 point pairs
        std::ranges::shuffle(indices, rng);

        const std::vector objPoints{
            objectPoints[indices[0]], objectPoints[indices[1]],
            objectPoints[indices[2]], objectPoints[indices[3]]};
        const std::vector imgPoints{
            imagePoints[indices[0]], imagePoints[indices[1]],
            imagePoints[indices[2]], imagePoints[indices[3]]};

        cv::solvePnP(objPoints, imgPoints, cameraMatrix, distortion, rvec, tvec,
                     false, cv::SOLVEPNP_P3P);

        // Compute inliers
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortion,
                          projectedPoints);

        std::vector<int> inliers;
        for (size_t k{0}; k < numPoints; k++) {
            if (cv::norm(imagePoints[k] - projectedPoints[k]) < threshold) {
                inliers.push_back(static_cast<int>(k));
            }
        }
        trialcount++;

        // Keep the best one
        if (inliers.size() > _inliers.size()) {
            inliers.swap(_inliers);
            tvec.copyTo(_tvec);
            rvec.copyTo(_rvec);

            // with probability p, a data set with no outliers.
            constexpr auto myepsilon = 1e-5;

            double fracinliers =
                static_cast<double>(_inliers.size()) / numPoints;
            double pNoOutliers = 1 - std::pow(fracinliers, 3);
            if (pNoOutliers == 0)
                pNoOutliers = myepsilon;
            if (pNoOutliers > (1 - myepsilon))
                pNoOutliers = 1 - myepsilon;
            double tempest = std::log(1 - confidence) / std::log(pNoOutliers);
            N = int(std::round(tempest));
            trialcount = 0;
        }
        countit++;
    }

    if (refine && _inliers.size() >= 4) {
        std::vector<cv::Point3f> objPoints;
        std::vector<cv::Point2f> imgPoints;
        objPoints.reserve(_inliers.size());
        imgPoints.reserve(_inliers.size());
        for (const auto &inlier : _inliers) {
            imgPoints.push_back(imagePoints[inlier]);
            objPoints.push_back(objectPoints[inlier]);
        }
        cv::solvePnP(objPoints, imgPoints, cameraMatrix, distortion, _rvec,
                     _tvec, true);
    }

    return cv::Mat{_inliers, true};
}

cv::Mat solveP3PRansac(const std::vector<cv::Point3f> &objectPoints,
                       const std::vector<cv::Point2f> &imagePoints,
                       cv::InputArray cameraMatrix, cv::InputArray distortion,
                       cv::OutputArray rvec, cv::OutputArray tvec,
                       float threshold, int iterations, int type,
                       double confidence, bool refine)
{
    CV_Assert(objectPoints.size() >= size_t(4) &&
              objectPoints.size() == imagePoints.size());

    switch (type) {
        case 0: {
            return _solveP3PRansac(objectPoints, imagePoints, cameraMatrix,
                                   distortion, rvec, tvec, threshold,
                                   iterations, confidence, refine);
        }
        case 1: {
            std::vector<cv::Point2f> imagePoints_u;
            cv::fisheye::undistortPoints(imagePoints, imagePoints_u,
                                         cameraMatrix, distortion);

            {
                const auto K = cameraMatrix.getMat();
                const auto fx = float(K.at<double>(0, 0));
                const auto fy = float(K.at<double>(1, 1));
                const auto cx = float(K.at<double>(0, 2));
                const auto cy = float(K.at<double>(1, 2));
                for (auto &imagePoint_u : imagePoints_u) {
                    imagePoint_u.x = imagePoint_u.x * fx + cx;
                    imagePoint_u.y = imagePoint_u.y * fy + cy;
                }
            }

            return _solveP3PRansac(objectPoints, imagePoints_u, cameraMatrix,
                                   cv::Mat_<double>::zeros(1, 5), rvec, tvec,
                                   threshold, iterations, confidence, refine);
        }
        default:
            CV_Error(cv::Error::StsBadFlag, "Unsupported camera model!");
            break;
    }
}

cv::Mat solvePnPRansac(cv::InputArray _objectPoints,
                       cv::InputArray _imagePoints,
                       cv::InputArray _cameraMatrix, cv::InputArray distortion,
                       cv::OutputArray rvec, cv::OutputArray tvec,
                       int iteration, float threshold, int type,
                       double confidence, bool refine)
{
    cv::Mat inliers;
    switch (type) {
        case 0: {
            if (!cv::solvePnPRansac(_objectPoints, _imagePoints, _cameraMatrix,
                                    distortion, rvec, tvec, false, iteration,
                                    threshold, confidence, inliers,
                                    cv::SOLVEPNP_P3P)) {
                return {};
            }
            break;
        }
        case 1: {
            std::vector<cv::Point2f> imagePoints_u;
            cv::fisheye::undistortPoints(_imagePoints, imagePoints_u,
                                         _cameraMatrix, distortion);

            const auto cameraMatrix = _cameraMatrix.getMat();
            const auto fx = float(cameraMatrix.at<double>(0, 0));
            const auto fy = float(cameraMatrix.at<double>(1, 1));
            const auto cx = float(cameraMatrix.at<double>(0, 2));
            const auto cy = float(cameraMatrix.at<double>(1, 2));
            for (auto &imagePoint_u : imagePoints_u) {
                imagePoint_u.x = imagePoint_u.x * fx + cx;
                imagePoint_u.y = imagePoint_u.y * fy + cy;
            }

            if (!cv::solvePnPRansac(_objectPoints, imagePoints_u, cameraMatrix,
                                    cv::noArray(), rvec, tvec, false, iteration,
                                    threshold, confidence, inliers,
                                    cv::SOLVEPNP_P3P)) {
                return {};
            }
            break;
        }
        default:
            CV_Error(cv::Error::StsBadFlag, "Unsupported camera model!");
            break;
    }

    constexpr auto kMinInliers{4};
    if (const auto numInliers = inliers.rows;
        refine && numInliers >= kMinInliers) {
        const auto objectPoints = _objectPoints.getMat();
        const auto imagePoints = _imagePoints.getMat();
        std::vector<cv::Point3f> objectPointsInliers;
        std::vector<cv::Point2f> imagePointsInliers;
        objectPointsInliers.reserve(numInliers);
        imagePointsInliers.reserve(numInliers);
        for (auto i{0}; i < numInliers; ++i) {
            const auto ind = inliers.at<int>(i);
            objectPointsInliers.push_back(objectPoints.at<cv::Point3f>(ind));
            imagePointsInliers.push_back(imagePoints.at<cv::Point2f>(ind));
        }

        cv::solvePnP(objectPointsInliers, imagePointsInliers, _cameraMatrix,
                     distortion, rvec, tvec, true);
    }

    return inliers;
}

cv::Mat handeyeCalibration(const std::vector<cv::Mat> &poses1,
                           const std::vector<cv::Mat> &poses2)
{
    CV_Assert(!poses1.empty() && poses1.size() == poses2.size());

    // Prepare the poses for handeye calibration
    const auto numPoses = poses1.size();
    std::vector<cv::Mat> rmats(numPoses), tvecs1(numPoses), rmats2(numPoses),
        tvecs2(numPoses);
    for (size_t i{0}; i < numPoses; i++) {
        const cv::Mat pose1 = poses1[i].inv();
        const cv::Mat pose2 = poses2[i];

        // save in datastruct
        cv::Mat rvec1, tvec1, rvec2, tvec2;
        Proj2RT(pose1, rvec1, tvec1);
        Proj2RT(pose2, rvec2, tvec2);

        cv::Mat rmat1, rmat2;
        cv::Rodrigues(rvec1, rmat1);
        cv::Rodrigues(rvec2, rmat2);

        rmats[i] = rmat1;
        tvecs1[i] = tvec1;
        rmats2[i] = rmat2;
        tvecs2[i] = tvec2;
    }

    // Hand-eye calibration
    cv::Mat r_g1_g2, t_g1_g2;
    cv::calibrateHandEye(rmats, tvecs1, rmats2, tvecs2, r_g1_g2, t_g1_g2,
                         cv::CALIB_HAND_EYE_HORAUD);
    return RT2Proj(r_g1_g2, t_g1_g2);
}

cv::Mat handeyeBootstratpTranslationCalibration(
    unsigned int nb_cluster, unsigned int nb_it,
    const std::vector<cv::Mat> &pose_abs_1,
    const std::vector<cv::Mat> &pose_abs_2)
{
    // N clusters but less if less images available
    nb_cluster =
        (pose_abs_1.size() < nb_cluster) ? pose_abs_1.size() : nb_cluster;

    // Prepare the translational component of the cameras to be clustered
    // concatenation of the translation of pose 1 and 2 for clustering
    cv::Mat position_1_2;
    for (size_t i = 0; i < pose_abs_1.size(); i++) {
        cv::Mat rvec_1, tvec_1, rot_2, tvec_2;
        Proj2RT(pose_abs_1[i], rvec_1, tvec_1);
        Proj2RT(pose_abs_2[i], rot_2, tvec_2);
        cv::Mat concat_trans_1_2;
        cv::hconcat(tvec_1.t(), tvec_2.t(), concat_trans_1_2);
        position_1_2.push_back(concat_trans_1_2);
    }
    position_1_2.convertTo(position_1_2, CV_32F);

    // Cluster the observation to select the most diverse poses
    constexpr auto kkmeanIterations = 5;

    cv::Mat labels;
    cv::Mat centers;
    std::ignore = cv::kmeans(
        position_1_2, nb_cluster, labels,
        cv::TermCriteria{cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10,
                         0.01},
        kkmeanIterations, cv::KMEANS_PP_CENTERS, centers);
    labels.convertTo(labels, CV_32S);

    // Iterate n times the
    std::vector<double> r1_he, r2_he, r3_he; // structure to save valid rot
    std::vector<double> t1_he, t2_he, t3_he; // structure to save valid trans
    unsigned int nb_clust_pick = 6;
    unsigned int nb_success = 0;
    for (unsigned int iter = 0; iter < nb_it; iter++) {
        // pick from n of these clusters randomly
        std::vector<unsigned int> shuffled_ind(nb_cluster);
        std::iota(shuffled_ind.begin(), shuffled_ind.end(), 0);

        std::random_device rd;
        std::mt19937 rng{rd()};
        std::ranges::shuffle(shuffled_ind, rng);

        std::vector<unsigned int> cluster_select;
        cluster_select.reserve(nb_clust_pick);
        for (unsigned int k = 0; k < nb_clust_pick; ++k) {
            cluster_select.push_back(shuffled_ind[k]);
        }

        // Select one pair of pose for each cluster
        std::vector<unsigned int> pose_ind;
        pose_ind.reserve(cluster_select.size());
        for (const auto &clust_ind : cluster_select) {
            std::vector<unsigned int> idx;
            for (unsigned int j = 0; j < pose_abs_2.size(); j++) {
                if (labels.at<unsigned int>(j) == clust_ind) {
                    idx.push_back(j);
                }
            }

            // randomly select an index in the occurrences of the cluster
            srand(time(NULL));
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, idx.size() - 1);
            unsigned int cluster_idx = dis(gen);
            pose_ind.push_back(idx[cluster_idx]);
        }

        // Prepare the poses for handeye calibration
        std::vector<cv::Mat> r_cam_group_1, t_cam_group_1, r_cam_group_2,
            t_cam_group_2;
        r_cam_group_1.reserve(pose_ind.size());
        t_cam_group_1.reserve(pose_ind.size());
        r_cam_group_2.reserve(pose_ind.size());
        t_cam_group_2.reserve(pose_ind.size());
        for (const auto &pose_ind_i : pose_ind) {
            // get the poses
            cv::Mat pose_cam_group_1 = pose_abs_1[pose_ind_i].inv();
            cv::Mat pose_cam_group_2 = pose_abs_2[pose_ind_i];

            // save in datastruct
            cv::Mat r_1, r_2, t_1, t_2;
            Proj2RT(pose_cam_group_1, r_1, t_1);
            Proj2RT(pose_cam_group_2, r_2, t_2);
            cv::Mat r_1_mat, r_2_mat;
            cv::Rodrigues(r_1, r_1_mat);
            cv::Rodrigues(r_2, r_2_mat);
            r_cam_group_1.push_back(r_1_mat);
            t_cam_group_1.push_back(t_1);
            r_cam_group_2.push_back(r_2_mat);
            t_cam_group_2.push_back(t_2);
        }

        // Hand-eye calibration
        cv::Mat r_g1_g2, t_g1_g2;
        cv::calibrateHandEye(r_cam_group_1, t_cam_group_1, r_cam_group_2,
                             t_cam_group_2, r_g1_g2, t_g1_g2,
                             cv::CALIB_HAND_EYE_TSAI);
        // cv::CALIB_HAN
        cv::Mat pose_g1_g2 = RT2Proj(r_g1_g2, t_g1_g2);

        // Check the consistency of the set
        double max_error = 0;
        for (size_t i = 0; i < pose_ind.size(); i++) {
            cv::Mat pose_cam_group_1_1 = pose_abs_1[pose_ind[i]];
            cv::Mat pose_cam_group_2_1 = pose_abs_2[pose_ind[i]];
            for (size_t j = 0; j < pose_ind.size(); j++) {
                if (i == j) {
                    continue;
                }

                cv::Mat pose_cam_group_1_2 = pose_abs_1[pose_ind[i]];
                cv::Mat pose_cam_group_2_2 = pose_abs_2[pose_ind[i]];
                cv::Mat PP1 = pose_cam_group_1_2.inv() * pose_cam_group_1_1;
                cv::Mat PP2 = pose_cam_group_2_1.inv() * pose_cam_group_2_2;
                cv::Mat ErrMat = PP2.inv() * pose_g1_g2 * PP1 * pose_g1_g2;
                cv::Mat ErrRot, ErrTrans;
                Proj2RT(ErrMat, ErrRot, ErrTrans);
                cv::Mat ErrRotMat;
                cv::Rodrigues(ErrRot, ErrRotMat);
                double traceRot = cv::trace(ErrRotMat)[0] -
                                  std::numeric_limits<double>::epsilon();

                double err_degree =
                    std::acos(0.5 * (traceRot - 1.0)) * 180.0 / M_PI;
                if (err_degree > max_error) {
                    max_error = err_degree;
                }
            }
        }

        // if it is a sucess then add to our valid pose evector
        if (max_error < 15.) {
            nb_success++;
            cv::Mat rot_temp, trans_temp;
            Proj2RT(pose_g1_g2, rot_temp, trans_temp);
            r1_he.push_back(rot_temp.at<double>(0));
            r2_he.push_back(rot_temp.at<double>(1));
            r3_he.push_back(rot_temp.at<double>(2));
            t1_he.push_back(trans_temp.at<double>(0));
            t2_he.push_back(trans_temp.at<double>(1));
            t3_he.push_back(trans_temp.at<double>(2));
        }
    }

    // if enough sucess (at least 3) then compute median value
    if (nb_success > 3) {
        cv::Mat r_he = calcAverageRotation(r1_he, r2_he, r3_he);
        cv::Mat t_he = cv::Mat::zeros(3, 1, CV_64F);
        t_he.at<double>(0) = median(t1_he);
        t_he.at<double>(1) = median(t2_he);
        t_he.at<double>(2) = median(t3_he);
        return RVecT2Proj(r_he, t_he);
    }

    return handeyeCalibration(pose_abs_1, pose_abs_1);
}

cv::Mat convertRotationMatrixToQuaternion(cv::Mat R)
{
    // code is adapted from
    // https://gist.github.com/shubh-agrawal/76754b9bfb0f4143819dbd146d15d4c8

    cv::Mat Q(1, 4, CV_64F); // x y z w

    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0);
        Q.at<double>(0, 3) = (s * 0.5);
        s = 0.5 / s;
        Q.at<double>(0, 0) = ((R.at<double>(2, 1) - R.at<double>(1, 2)) * s);
        Q.at<double>(0, 1) = ((R.at<double>(0, 2) - R.at<double>(2, 0)) * s);
        Q.at<double>(0, 2) = ((R.at<double>(1, 0) - R.at<double>(0, 1)) * s);
    }

    else {
        int i = R.at<double>(0, 0) < R.at<double>(1, 1)
                    ? (R.at<double>(1, 1) < R.at<double>(2, 2) ? 2 : 1)
                    : (R.at<double>(0, 0) < R.at<double>(2, 2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        double s = std::sqrt(R.at<double>(i, i) - R.at<double>(j, j) -
                             R.at<double>(k, k) + 1.0);
        Q.at<double>(0, i) = s * 0.5;
        s = 0.5 / s;

        Q.at<double>(0, 3) = (R.at<double>(k, j) - R.at<double>(j, k)) * s;
        Q.at<double>(0, j) = (R.at<double>(j, i) + R.at<double>(i, j)) * s;
        Q.at<double>(0, k) = (R.at<double>(k, i) + R.at<double>(i, k)) * s;
    }

    return Q;
}

cv::Mat convertQuaternionToRotationMatrix(const std::array<double, 4> &q)
{
    // code adapted from
    // https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/

    const double q0 = q[3];
    const double q1 = q[0];
    const double q2 = q[1];
    const double q3 = q[2];

    cv::Mat rot_matrix(3, 3, CV_64F);
    rot_matrix.at<double>(0, 0) = 2 * (q0 * q0 + q1 * q1) - 1;
    rot_matrix.at<double>(0, 1) = 2 * (q1 * q2 - q0 * q3);
    rot_matrix.at<double>(0, 2) = 2 * (q1 * q3 + q0 * q2);

    rot_matrix.at<double>(1, 0) = 2 * (q1 * q2 + q0 * q3);
    rot_matrix.at<double>(1, 1) = 2 * (q0 * q0 + q2 * q2) - 1;
    rot_matrix.at<double>(1, 2) = 2 * (q2 * q3 - q0 * q1);

    rot_matrix.at<double>(2, 0) = 2 * (q1 * q3 - q0 * q2);
    rot_matrix.at<double>(2, 1) = 2 * (q2 * q3 + q0 * q1);
    rot_matrix.at<double>(2, 2) = 2 * (q0 * q0 + q3 * q3) - 1;

    return rot_matrix;
}

cv::Mat calcAverageRotation(std::vector<double> &r1, std::vector<double> &r2,
                            std::vector<double> &r3, bool useQuaternionAverage)
{
    cv::Mat average_rotation = cv::Mat::zeros(3, 1, CV_64F);
    if (useQuaternionAverage) {
        // The Quaternion Averaging algorithm is described in
        // https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
        // implementaion references:
        //  -
        //  https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
        //  -
        //  https://github.com/tolgabirdal/averaging_quaternions/blob/master/avg_quaternion_markley.m

        assert(r1.size() == r2.size() && r2.size() == r3.size());

        std::vector<cv::Mat> quaternions;
        // convert rotation vector to quaternion through rotation matrix
        for (unsigned int angle_idx = 0u; angle_idx < r1.size(); ++angle_idx) {
            std::array<double, 3> angles = {r1[angle_idx], r2[angle_idx],
                                            r3[angle_idx]};
            const cv::Mat rot_vec = cv::Mat(1, 3, CV_64F, angles.data());
            cv::Mat rot_matrix;
            cv::Rodrigues(rot_vec, rot_matrix);
            const cv::Mat quaternion =
                convertRotationMatrixToQuaternion(rot_matrix);
            quaternions.push_back(quaternion);
        }

        cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);
        for (cv::Mat &q : quaternions) {
            if (q.at<double>(0, 3) < 0) {
                // handle the antipodal configurations
                q = -q;
            }
            A += q.t() * q;
        }
        A /= quaternions.size();

        cv::SVD svd(A, cv::SVD::FULL_UV);
        cv::Mat U = svd.u;
        cv::Mat singularValues = svd.w;

        const unsigned int largestEigenValueIndex = 0u;
        std::array<double, 4> average_quaternion = {
            svd.u.at<double>(0, largestEigenValueIndex),
            svd.u.at<double>(1, largestEigenValueIndex),
            svd.u.at<double>(2, largestEigenValueIndex),
            svd.u.at<double>(3, largestEigenValueIndex)};

        cv::Mat rot_matrix =
            convertQuaternionToRotationMatrix(average_quaternion);
        cv::Rodrigues(rot_matrix, average_rotation);
    }
    else {
        average_rotation.at<double>(0) = median(r1);
        average_rotation.at<double>(1) = median(r2);
        average_rotation.at<double>(2) = median(r3);
    }

    return average_rotation;
}

void calcAverageRotation2(cv::InputArrayOfArrays _rvecs,
                          cv::OutputArray avgRvec, bool useQuaternionAverage)
{
    // Check type as well, support floating point only
    CV_Assert(_rvecs.isMatVector());

    std::vector<cv::Mat> rvecs;
    _rvecs.getMatVector(rvecs);

    if (rvecs.empty()) {
        return;
    }

    const auto total = rvecs.size();

    if (useQuaternionAverage) {
        auto A = cv::Matx44d::zeros();
        for (const auto &rvec : rvecs) {
            const auto quat = cv::Quatd::createFromRvec(rvec);
            const cv::Vec4d qvec{quat.x, quat.y, quat.z, quat.w};
            A += (qvec * qvec.t());
        }
        A /= static_cast<double>(total);

        const cv::SVD svd{A, cv::SVD::FULL_UV};
        const auto xyzw = svd.u.col(0);
        cv::Mat{cv::Quatd{xyzw.at<double>(3), xyzw.at<double>(0),
                          xyzw.at<double>(1), xyzw.at<double>(2)}
                    .toRotVec()}
            .copyTo(avgRvec);
        return;
    }

    // TODO: Finish median
    cv::Mat rvecs_mat;
    cv::hconcat(_rvecs, rvecs_mat);
}

cv::Mat calcAveragePose(const std::vector<cv::Mat> &poses,
                        bool useQuaternionAverage)
{
    // Median
    const auto numPoses = poses.size();
    std::vector<double> r1, r2, r3;
    std::vector<double> t1, t2, t3;
    r1.reserve(numPoses);
    r2.reserve(numPoses);
    r3.reserve(numPoses);
    t1.reserve(numPoses);
    t2.reserve(numPoses);
    t3.reserve(numPoses);
    for (const auto &pose : poses) {
        cv::Mat rvec, tvec;
        Proj2RT(pose, rvec, tvec);
        r1.push_back(rvec.at<double>(0));
        r2.push_back(rvec.at<double>(1));
        r3.push_back(rvec.at<double>(2));
        t1.push_back(tvec.at<double>(0));
        t2.push_back(tvec.at<double>(1));
        t3.push_back(tvec.at<double>(2));
    }

    cv::Mat rvec_avg = calcAverageRotation(r1, r2, r3, useQuaternionAverage);
    cv::Mat tvec_avg = cv::Mat::zeros(3, 1, CV_64F);
    tvec_avg.at<double>(0) = median(t1);
    tvec_avg.at<double>(1) = median(t2);
    tvec_avg.at<double>(2) = median(t3);

    return RVecT2Proj(rvec_avg, tvec_avg);
}

} // namespace tl::mcmb
