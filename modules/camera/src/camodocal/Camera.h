#pragma once

#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>

namespace camodocal {

class Camera
{
public:
    enum ModelType
    {
        KANNALA_BRANDT,
        MEI,
        PINHOLE,
        PINHOLE_FULL,
        SCARAMUZZA
    };

    class Parameters
    {
    public:
        explicit Parameters(ModelType type,
                            const std::string &cameraName = "camera", int w = 0,
                            int h = 0);

        inline static constexpr char keyModelType[11]{"model_type"};
        virtual const char *modelTypeName() const = 0;

        ModelType &modelType();
        std::string &cameraName();
        int &imageWidth();
        int &imageHeight();

        ModelType modelType() const;
        const std::string &cameraName() const;
        int imageWidth() const;
        int imageHeight() const;
        inline cv::Size imageSize() const
        {
            return {imageWidth(), imageHeight()};
        }

        int nIntrinsics() const;

        bool readFromYamlFile(const std::string &filename);
        void writeToYamlFile(const std::string &filename) const;

        // TODO: Add static read/write functions if we want these IO methods to
        // be compatible with OpenCV IO interface, e.g.
        // void write(FileStorage& fs, const std::string&, const MyData& x)
        // void read(const FileNode& node, MyData& x, const MyData& default_val)
        virtual bool read(const cv::FileNode &node);
        virtual void write(cv::FileStorage &fs) const;

    protected:
        ModelType m_modelType;
        int m_nIntrinsics;
        std::string m_cameraName;
        int m_imageWidth;
        int m_imageHeight;
    };

    using Ptr = std::shared_ptr<Camera>;
    using ConstPtr = std::shared_ptr<const Camera>;

    // FIXME: These are not virtual interfaces
    virtual ModelType modelType() const = 0;
    virtual const std::string &cameraName() const = 0;
    virtual int imageWidth() const = 0;
    virtual int imageHeight() const = 0;
    inline cv::Size imageSize() const { return {imageWidth(), imageHeight()}; }

    virtual cv::Mat &mask();
    virtual const cv::Mat &mask() const;

    virtual void estimateIntrinsics(
        const cv::Size &boardSize,
        const std::vector<std::vector<cv::Point3f>> &objectPoints,
        const std::vector<std::vector<cv::Point2f>> &imagePoints) = 0;
    virtual void estimateExtrinsics(
        const std::vector<cv::Point3f> &objectPoints,
        const std::vector<cv::Point2f> &imagePoints, cv::Mat &rvec,
        cv::Mat &tvec) const;

    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d &p,
                            Eigen::Vector3d &P) const = 0;
    //%output P

    // Lift points from the image plane to the projective space
    virtual void liftProjective(const Eigen::Vector2d &p,
                                Eigen::Vector3d &P) const = 0;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    virtual void spaceToPlane(const Eigen::Vector3d &P,
                              Eigen::Vector2d &p) const = 0;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    // virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    //                          Eigen::Matrix<double,2,3>& J) const = 0;
    //%output p
    //%output J

    virtual void undistToPlane(const Eigen::Vector2d &p_u,
                               Eigen::Vector2d &p) const = 0;
    //%output p

    // virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale
    // = 1.0) const = 0;
    virtual cv::Mat initUndistortRectifyMap(
        cv::Mat &map1, cv::Mat &map2, float fx = -1.0f, float fy = -1.0f,
        cv::Size imageSize = cv::Size(0, 0), float cx = -1.0f, float cy = -1.0f,
        cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const = 0;

    // FIXME: These should not be virtual interfaces. The reason they are here
    // is base class doesn't have Parameter
    virtual int parameterCount() const = 0;

    virtual void readParameters(const std::vector<double> &parameters) = 0;
    virtual void writeParameters(std::vector<double> &parameters) const = 0;

    virtual void writeParametersToYamlFile(
        const std::string &filename) const = 0;

    virtual std::string parametersToString() const = 0;

    /**
     * \brief Calculates the reprojection distance between points
     *
     * \param P1 first 3D point coordinates
     * \param P2 second 3D point coordinates
     * \return euclidean distance in the plane
     */
    double reprojectionDist(const Eigen::Vector3d &P1,
                            const Eigen::Vector3d &P2) const;

    double reprojectionError(
        const std::vector<std::vector<cv::Point3f>> &objectPoints,
        const std::vector<std::vector<cv::Point2f>> &imagePoints,
        const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
        cv::OutputArray perViewErrors = cv::noArray()) const;

    double reprojectionError(const Eigen::Vector3d &P,
                             const Eigen::Quaterniond &camera_q,
                             const Eigen::Vector3d &camera_t,
                             const Eigen::Vector2d &observed_p) const;

    void projectPoints(const std::vector<cv::Point3f> &objectPoints,
                       const cv::Mat &rvec, const cv::Mat &tvec,
                       std::vector<cv::Point2f> &imagePoints) const;

protected:
    cv::Mat m_mask;
};

} // namespace camodocal
