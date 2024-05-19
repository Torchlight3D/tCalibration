#include <Eigen/Core>
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
    // Set up random cameras.
    Camera camera1 = RandomCamera();
    Scene scene;
    ViewId vid = scene.addView("0", 0, 0.0);
    scene.rView(vid)->rCamera().deepCopy(camera1);
    scene.rView(vid)->setEstimated(true);
    // Set up random points.
    for (int i = 0; i < numPoints; i++) {
        Vector3d point(kRNG.RandDouble(-5., 5.), kRNG.RandDouble(-5., 5.),
                       kRNG.RandDouble(4., 10.));
        auto trackId = scene.addTrack();
        scene.rTrack(trackId)->setPosition(point.homogeneous());
        scene.rTrack(trackId)->setEstimated(true);

        Vector2d pixel;
        double depth =
            scene.view(vid)->camera().projectPoint(point.homogeneous(), pixel);
        if (pixelNoise > 0.0) {
            AddNoiseToProjection(pixelNoise, &kRNG, &pixel);
        }
        if (depth > 0.0) {
            scene.addFeature(vid, trackId, Feature(pixel));
        }
    }

    BundleAdjustmentOptions opts;
    opts.verbose = true;
    BundleAdjustmentSummary sum = BundleAdjustView(opts, vid, &scene);
    std::cout << "Success: " << sum.success << "\n";
    std::cout << "Final squared reprojection error: " << 2.0 * sum.final_cost
              << "\n";
    if (pixelNoise == 0.0) {
        EXPECT_TRUE(2.0 * sum.final_cost / scene.view(vid)->featureCount() <
                    1e-15);
    }
    else {
        EXPECT_TRUE(2.0 * sum.final_cost / scene.view(vid)->featureCount() <
                    pixelNoise);
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
