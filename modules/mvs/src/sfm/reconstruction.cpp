#include "reconstruction.h"

#include <glog/logging.h>

#include <tCore/ContainerUtils>
#include <tMath/Eigen/Utils>

// #include "globalreconstruction.h"
// #include "hybridreconstruction.h"
// #include "incrementalreconstruction.h"

namespace tl {

// Reconstruction* Reconstruction::Create(const Options&
// options)
// {
//     switch (options.reconstruction_estimator_type) {
//         case Type::GLOBAL:
//             return new GlobalReconstructionEstimator(options);
//         case Type::INCREMENTAL:
//             return new IncrementalReconstructionEstimator(options);
//         case Type::Hybrid:
//             return new HybridReconstructionEstimator(options);
//         default:
//             LOG(ERROR) << "Invalid reconstruction estimator specified.";
//     }
//     return nullptr;
// }

BundleAdjustment::Options ReconstructionOptions::toBundleAdjustmentOptions(
    int numViews) const
{
    constexpr int kMinViewsForSparseSchur = 150;

    BundleAdjustment::Options opts;
    opts.num_threads = num_threads;
    opts.loss_function_type = bundle_adjustment_loss_function_type;
    opts.robust_loss_width = bundle_adjustment_robust_loss_width;
    opts.use_inner_iterations = true;
    opts.intrinsics_to_optimize = intrinsics_to_optimize;

    switch (track_parametrization_type) {
        case TrackParametrizationType::HomoManifold:
            opts.use_homogeneous_local_point_parametrization = true;
            opts.use_inverse_depth_parametrization = false;
            break;
        case TrackParametrizationType::InverseDepth:
            opts.use_homogeneous_local_point_parametrization = false;
            opts.use_inverse_depth_parametrization = true;
            break;
        case TrackParametrizationType::Homo:
            opts.use_homogeneous_local_point_parametrization = false;
            opts.use_inverse_depth_parametrization = false;
            break;
        default:
            LOG(ERROR) << "Unknown track parametrization type.";
            break;
    }

    if (numViews >= min_cameras_for_iterative_solver) {
        opts.linear_solver_type = ceres::SPARSE_SCHUR;
        opts.preconditioner_type = ceres::JACOBI;
        // NOTE: this is an arbitrary scaling that was found to work well. It
        // may need to change depending on the application.
        opts.max_num_iterations *= 1.5;
    }
    else if (numViews >= kMinViewsForSparseSchur) {
        opts.linear_solver_type = ceres::SPARSE_SCHUR;
        opts.preconditioner_type = ceres::JACOBI;
    }
    else {
        opts.linear_solver_type = ceres::DENSE_SCHUR;
    }

    opts.verbose = VLOG_IS_ON(1);
    return opts;
}

void setCameraIntrinsicsFromMetaData(Scene* scene)
{
    if (!scene) {
        return;
    }

    for (const auto& camId : scene->cameraIds()) {
        const auto sharedCameraViewIds = scene->sharedCameraViewIds(camId);

        // Find the representative view from the estimated views in the scene.
        // If none of the views is estimated, then the choice of representative
        // view doesnt really matter much.
        auto repViewId = kInvalidViewId;
        for (const auto& viewId : sharedCameraViewIds) {
            if (const auto* view = scene->view(viewId); view) {
                repViewId = viewId;
                if (view->estimated()) {
                    break;
                }
            }
        }

        // Set the representative camera intrinsics.
        auto* repView = scene->rView(repViewId);
        auto& repCamera = repView->rCamera();
        repCamera.setFromMetaData(repView->cameraMetaData());

        // Update all other views shared the same camera.
        for (const auto& viewId : sharedCameraViewIds) {
            if (auto* view = scene->rView(viewId);
                view && repViewId != viewId) {
                // Set the view's intrinsics to point to the shared intrinsics.
                // This includes estimated views who may have estimated
                // intrinsics parameters that are different than the shared
                // intrinsics.
                view->rCamera().setCameraIntrinsics(
                    repCamera.cameraIntrinsics());
                view->rCamera().setImageSize(repCamera.imageWidth(),
                                             repCamera.imageHeight());
            }
        }
    }
}

} // namespace tl
