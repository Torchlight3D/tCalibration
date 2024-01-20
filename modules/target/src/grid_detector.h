// #pragma once

// #include "../camera/CameraModelBase.h"
// #include "../camera/simple/CameraBase.h"
// #include "CalibBoardBase.h"

// namespace tl {

// class CalibObservation;

// class GridDetector
//{
// public:
//     struct Options
//     {
//         float filterCornerSigmaThreshold{2.};
//         float filterCornerMinReprojError{0.2};
//         bool plotCornerReprojection{false};
//         bool imageStepping{false};
//         bool filterCornerOutliers{false};

//        Options() {}
//    };

//    // TODO: use ConstPtr
//    GridDetector(CameraBase::Ptr camera, CalibBoardBase::Ptr target,
//                 const Options &options = {});
//    GridDetector(CameraModelBase::Ptr geometry, CalibBoardBase::Ptr target,
//                 const Options &options = {});
//    virtual ~GridDetector();

//    bool findTarget(const cv::Mat &image, CalibObservation &observation,
//                    bool estimateTransformation = false) const;

// private:
//     CameraModelBase::Ptr _geometry;
//     CameraBase::Ptr _camera;
//     CalibBoardBase::Ptr _target;
//     GridDetector::Options _options;
// };

//} // namespace tl
