#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCamera/Camera>
#include <tCore/RandomGenerator>
#include <tMvs/BundleAdjustment>
#include <tMvs/Scene>

#include "test_utils.h"

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {
RandomNumberGenerator kRNG{52};
}

inline Camera RandomCamera()
{
    Camera camera;
    camera.setPosition(Vector3d::Random());
    camera.setOrientationFromAngleAxis(0.2 * Vector3d::Random());
    camera.setImageSize(1000, 1000);
    camera.setFocalLength(500);
    camera.setPrincipalPoint(500, 500);
    return camera;
}

inline void TestOptimizeView(int numPoints, double pixelNoise)
{
    // Set up random cameras
    const Camera camera = RandomCamera();
    Scene scene;
    const auto viewId = scene.addView("0", 0., CameraId{0});
    scene.rView(viewId)->rCamera().deepCopy(camera);
    scene.rView(viewId)->setEstimated(true);

    // Set up random points
    for (int i = 0; i < numPoints; i++) {
        const Vector3d point{kRNG.RandDouble(-5., 5.), kRNG.RandDouble(-5., 5.),
                             kRNG.RandDouble(4., 10.)};
        const auto trackId = scene.addTrack();
        scene.rTrack(trackId)->setPosition(point.homogeneous());
        scene.rTrack(trackId)->setEstimated(true);

        Vector2d pixel;
        const auto depth = scene.view(viewId)->camera().projectPoint(
            point.homogeneous(), pixel);
        if (pixelNoise > 0.) {
            AddNoiseToProjection(pixelNoise, &kRNG, &pixel);
        }
        if (depth > 0.) {
            scene.addFeature(viewId, trackId, Feature(pixel));
        }
    }

    BundleAdjustment::Options opts;
    opts.verbose = true;

    const auto summary = BundleAdjustView(opts, viewId, &scene);
    LOG(INFO) << "Success: " << summary.success;
    LOG(INFO) << "Final squared reprojection error: "
              << 2. * summary.final_cost;

    const auto noiseMean =
        2. * summary.final_cost / scene.view(viewId)->featureCount();

    if (pixelNoise == 0.) {
        EXPECT_TRUE(noiseMean < 1e-15);
    }
    else {
        EXPECT_TRUE(noiseMean < pixelNoise);
    }
}

TEST(OptimizeView, NoNoise)
{
    constexpr double kPixelNoise{0.0};
    constexpr int kNumPoints{100};
    TestOptimizeView(kNumPoints, kPixelNoise);
}

TEST(OptimizeView, Noise)
{
    constexpr double kPixelNoise{0.1};
    constexpr int kNumPoints{100};
    TestOptimizeView(kNumPoints, kPixelNoise);
}
