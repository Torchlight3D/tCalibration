#include <gtest/gtest.h>
// #include <glog/logging.h>

#include <tCore/ContainerUtils>
#include <tMvs/Scene>

using namespace tl;

namespace {
const std::vector<std::string> kViewNames{"1", "2", "3"};
const std::vector<Feature> kFeatures{Feature(1., 1.), Feature(2., 2.),
                                     Feature(3., 3.)};
} // namespace

TEST(Reconstruction, ViewIdFromNameValid)
{
    Scene scene;
    const auto gtViewId = scene.addView(kViewNames[0], 0.);
    const auto fromNameViewId = scene.viewIdFromName(kViewNames[0]);
    EXPECT_EQ(gtViewId, fromNameViewId);
}

TEST(Reconstruction, ViewIdFromNameInvalid)
{
    Scene scene;
    EXPECT_EQ(scene.viewIdFromName(kViewNames[0]), kInvalidViewId);
}

TEST(Reconstruction, addView)
{
    Scene scene;
    const auto viewId = scene.addView(kViewNames[0], 0.0);
    EXPECT_NE(viewId, kInvalidViewId);
    EXPECT_EQ(scene.viewCount(), 1);
    EXPECT_EQ(scene.trackCount(), 0);
    EXPECT_EQ(scene.addView(kViewNames[0], 0.), kInvalidViewId);
    EXPECT_EQ(scene.cameraId(viewId), 0);
}

TEST(Reconstruction, addViewWithCameraIntrinsicsGroup)
{
    Scene scene;
    const CameraId camGroupId = 1;
    const auto viewId = scene.addView(kViewNames[0], camGroupId, 0.);
    EXPECT_NE(viewId, kInvalidViewId);
    EXPECT_EQ(scene.viewCount(), 1);
    EXPECT_EQ(scene.trackCount(), 0);
    EXPECT_EQ(scene.cameraCount(), 1);
    EXPECT_EQ(scene.cameraId(viewId), camGroupId);
    EXPECT_EQ(scene.addView(kViewNames[0], 0.0), kInvalidViewId);
}

TEST(Reconstruction, RemoveView)
{
    Scene scene;
    const auto viewId1 = scene.addView(kViewNames[0], 0.);
    const auto viewId2 = scene.addView(kViewNames[1], 1.);
    EXPECT_EQ(scene.viewCount(), 2);
    EXPECT_EQ(scene.cameraCount(), 2);

    const auto view1_group = scene.cameraId(viewId1);
    const auto view2_group = scene.cameraId(viewId2);
    EXPECT_TRUE(scene.removeView(viewId1));
    EXPECT_EQ(scene.viewCount(), 1);
    EXPECT_EQ(scene.viewIdFromName(kViewNames[0]), kInvalidViewId);
    EXPECT_EQ(scene.view(viewId1), nullptr);
    EXPECT_EQ(scene.cameraId(viewId1), kInvalidCameraId);
    EXPECT_EQ(scene.cameraCount(), 1);

    const auto viewIds = scene.sharedCameraViewIds(view1_group);
    EXPECT_FALSE(std::find(viewIds.cbegin(), viewIds.cend(), viewId1) !=
                 viewIds.cend());

    EXPECT_TRUE(scene.removeView(viewId2));
    EXPECT_EQ(scene.viewCount(), 0);
    EXPECT_EQ(scene.viewIdFromName(kViewNames[1]), kInvalidViewId);
    EXPECT_EQ(scene.view(viewId2), nullptr);
    EXPECT_EQ(scene.cameraId(viewId2), kInvalidCameraId);
    EXPECT_EQ(scene.cameraCount(), 0);

    const auto view2_camera_intrinsics_group =
        scene.sharedCameraViewIds(view2_group);
    EXPECT_FALSE(std::find(viewIds.cbegin(), viewIds.cend(), viewId2) !=
                 viewIds.cend());

    EXPECT_FALSE(scene.removeView(kInvalidViewId));
    EXPECT_FALSE(scene.removeView(viewId1));
}

TEST(Reconstruction, GetViewValid)
{
    Scene scene;
    const auto viewId = scene.addView(kViewNames[0], 0.);
    EXPECT_NE(viewId, kInvalidViewId);

    const auto constView = scene.view(viewId);
    EXPECT_NE(constView, nullptr);

    auto mutableView = scene.rView(viewId);
    EXPECT_NE(mutableView, nullptr);
}

TEST(Reconstruction, GetViewValidInvalid)
{
    constexpr ViewId invalidViewId = 0;

    Scene scene;
    const auto constView = scene.view(invalidViewId);
    EXPECT_EQ(constView, nullptr);

    auto mutableView = scene.rView(invalidViewId);
    EXPECT_EQ(mutableView, nullptr);
}

TEST(Reconstruction, GetViewsInCameraIntrinsicGroup)
{
    constexpr double kFocalLength1{800.0};
    constexpr double kFocalLength2{1200.0};

    Scene scene;
    const auto viewId1 = scene.addView(kViewNames[0], 0, 0.);
    const auto camGroupId1 = scene.cameraId(viewId1);

    // Add a second view with to the same camera Id.
    const auto viewId2 = scene.addView(kViewNames[1], camGroupId1, 1.);
    const auto camGroupId2 = scene.cameraId(viewId2);
    EXPECT_EQ(camGroupId1, camGroupId2);

    // Add a third view this is with a different camera Id.
    const auto viewId3 = scene.addView(kViewNames[2], 2, 2.);
    const auto camGroupId3 = scene.cameraId(viewId3);
    EXPECT_NE(camGroupId1, camGroupId3);
    EXPECT_EQ(scene.cameraCount(), 2);

    // Change a value in view 1's camera intrinsics and ensure that it
    // propagates to view 2.
    auto& camera1 = scene.rView(viewId1)->rCamera();
    auto& camera2 = scene.rView(viewId2)->rCamera();
    auto& camera3 = scene.rView(viewId3)->rCamera();
    camera1.setFocalLength(kFocalLength1);
    EXPECT_EQ(camera1.focalLength(), camera2.focalLength());
    EXPECT_NE(camera1.focalLength(), camera3.focalLength());

    // Alter the intrinsics through camera 2 and ensure that camera 1 is
    //    updated and camera 3 is not.
    camera2.setFocalLength(kFocalLength2);
    EXPECT_EQ(camera1.focalLength(), camera2.focalLength());
    EXPECT_NE(camera2.focalLength(), camera3.focalLength());
}

TEST(Reconstruction, CameraIntrinsicsGroupIds)
{
    Scene scene;
    const auto viewId1 = scene.addView(kViewNames[0], 0.);
    const auto camId1 = scene.cameraId(viewId1);

    // Add a second view with to the same cameraId.
    const auto viewId2 = scene.addView(kViewNames[1], camId1, 1.);
    const auto camId2 = scene.cameraId(viewId2);
    EXPECT_EQ(camId1, camId2);

    // Add a third view that is with a different cameraId.
    const auto viewId3 = scene.addView(kViewNames[2], 2.);
    const auto camId3 = scene.cameraId(viewId3);
    EXPECT_NE(camId1, camId3);
    EXPECT_EQ(scene.cameraCount(), 2);

    // Ensure that the previous cameras are added correctly.
    const auto camIds = scene.cameraIds();
    EXPECT_EQ(camIds.size(), 2);

    EXPECT_TRUE(std::find(camIds.cbegin(), camIds.cend(), camId1) !=
                camIds.cend());
    EXPECT_TRUE(std::find(camIds.cbegin(), camIds.cend(), camId3) !=
                camIds.cend());
}

TEST(Reconstruction, AddEmptyTrack)
{
    Scene scene;
    const auto trackId = scene.addTrack();
    EXPECT_NE(trackId, kInvalidTrackId);
}

TEST(Reconstruction, addFeatureValid)
{
    Scene scene;

    const auto viewId1 = scene.addView(kViewNames[0], 0.0);
    const auto viewId2 = scene.addView(kViewNames[1], 1.0);
    EXPECT_NE(viewId1, kInvalidViewId);
    EXPECT_NE(viewId2, kInvalidViewId);

    const auto trackId = scene.addTrack();
    EXPECT_NE(trackId, kInvalidTrackId);

    EXPECT_TRUE(scene.addFeature(viewId1, trackId, kFeatures[0]));

    // Ensure that the observation adds the correct information to the view.
    const auto view1 = scene.view(viewId1);
    const auto view2 = scene.view(viewId2);
    EXPECT_EQ(view1->featureCount(), 1);
    EXPECT_EQ(view2->featureCount(), 0);

    const auto feature1 = view1->featureOf(trackId);
    EXPECT_NE(feature1, nullptr);
    EXPECT_EQ(feature1->point_.x(), kFeatures[0].point_.x());
    EXPECT_EQ(feature1->point_.y(), kFeatures[0].point_.y());

    const auto feature2 = view2->featureOf(trackId);
    EXPECT_EQ(feature2, nullptr);

    // Ensure that the observation adds the correct information to the track.
    const auto track = scene.track(trackId);
    EXPECT_EQ(track->viewCount(), 1);
    EXPECT_TRUE(track->viewIds().contains(viewId1));
}

TEST(Reconstruction, addFeatureInvalid)
{
    Scene scene;

    const auto viewId1 = scene.addView(kViewNames[0], 0.0);
    const auto viewId2 = scene.addView(kViewNames[1], 1.0);
    EXPECT_NE(viewId1, kInvalidViewId);
    EXPECT_NE(viewId2, kInvalidViewId);

    const auto trackId = scene.addTrack();
    EXPECT_NE(trackId, kInvalidTrackId);

    EXPECT_TRUE(scene.addFeature(viewId1, trackId, kFeatures[0]));
    EXPECT_TRUE(scene.addFeature(viewId2, trackId, kFeatures[0]));
    EXPECT_FALSE(scene.addFeature(viewId1, trackId, kFeatures[0]));
    EXPECT_FALSE(scene.addFeature(viewId2, trackId, kFeatures[0]));
    EXPECT_FALSE(scene.addFeature(viewId1, trackId, kFeatures[1]));
    EXPECT_FALSE(scene.addFeature(viewId2, trackId, kFeatures[1]));
}

TEST(Reconstruction, addTrackValid)
{
    Scene scene;

    const std::vector<std::pair<ViewId, Feature>> track{{0, kFeatures[0]},
                                                        {1, kFeatures[1]}};
    EXPECT_NE(scene.addView(kViewNames[0], 0.), kInvalidViewId);
    EXPECT_NE(scene.addView(kViewNames[1], 1.), kInvalidViewId);

    const auto trackId = scene.addTrack(track);
    EXPECT_NE(trackId, kInvalidTrackId);
    EXPECT_TRUE(scene.track(trackId));
    EXPECT_EQ(scene.trackCount(), 1);
}

TEST(Reconstruction, addTrackInvalid)
{
    Scene scene;

    // Should fail with less than two views.
    const std::vector<std::pair<ViewId, Feature>> invalidObs{{0, kFeatures[0]}};
    EXPECT_NE(scene.addView(kViewNames[0], 0.0), kInvalidViewId);
    EXPECT_EQ(scene.addTrack(invalidObs), kInvalidTrackId);
    EXPECT_EQ(scene.trackCount(), 0);
}

TEST(Reconstruction, RemoveTrackValid)
{
    Scene scene;

    const std::vector<std::pair<ViewId, Feature>> track{{0, kFeatures[0]},
                                                        {1, kFeatures[1]}};

    // Should be able to successfully remove the track.
    EXPECT_NE(scene.addView(kViewNames[0], 0.), kInvalidViewId);
    EXPECT_NE(scene.addView(kViewNames[1], 1.), kInvalidViewId);
    const auto trackId = scene.addTrack(track);
    EXPECT_TRUE(scene.removeTrack(trackId));
}

TEST(Reconstruction, RemoveTrackInvalid)
{
    Scene scene;

    // Should return false when trying to remove a track not in the scene.
    EXPECT_FALSE(scene.removeTrack(kInvalidTrackId));
}

TEST(Reconstruction, GetTrackValid)
{
    Scene scene;
    const std::vector<std::pair<ViewId, Feature>> track{{0, kFeatures[0]},
                                                        {1, kFeatures[1]}};
    EXPECT_NE(scene.addView(kViewNames[0], 0.0), kInvalidViewId);
    EXPECT_NE(scene.addView(kViewNames[1], 1.0), kInvalidViewId);

    const auto trackId = scene.addTrack(track);
    EXPECT_NE(trackId, kInvalidTrackId);

    const auto constTrack = scene.track(trackId);
    EXPECT_NE(constTrack, nullptr);

    auto mutableTrack = scene.rTrack(trackId);
    EXPECT_NE(mutableTrack, nullptr);
}

TEST(Reconstruction, GetTrackInvalid)
{
    Scene scene;
    const std::vector<std::pair<ViewId, Feature>> track = {};
    const auto trackId = scene.addTrack(track);
    EXPECT_EQ(trackId, kInvalidTrackId);

    const auto constTrack = scene.track(trackId);
    EXPECT_EQ(constTrack, nullptr);

    auto mutableTrack = scene.rTrack(trackId);
    EXPECT_EQ(mutableTrack, nullptr);
}

TEST(Reconstruction, GetSubReconstruction)
{
    constexpr int kViewCount{100};
    constexpr int kTrackCount{1000};
    constexpr int kNumObservationsPerTrack{10};

    Scene scene;
    for (int i{0}; i < kViewCount; i++) {
        const auto viewId =
            scene.addView(std::to_string(i), static_cast<double>(i));
        EXPECT_NE(viewId, kInvalidViewId);
    }

    for (int i{0}; i < kTrackCount; i++) {
        std::vector<std::pair<ViewId, Feature>> track;
        for (int j{0}; j < kNumObservationsPerTrack; j++) {
            track.emplace_back((i + j) % kViewCount, Feature{});
        }
        const auto trackId = scene.addTrack(track);
        EXPECT_NE(trackId, kInvalidTrackId);
    }

    // Test subset extraction with a fixed subset size. We trivially take
    // consecutive view ids to choose the subset.
    constexpr int kViewCountInSubset{25};
    for (int i = 0; i < kViewCount - kViewCountInSubset; i++) {
        std::unordered_set<ViewId> subViewIds;
        for (int j{0}; j < kViewCountInSubset; j++) {
            subViewIds.emplace(i + j);
        }

        Scene subScene;
        scene.extractSubScene(subViewIds, &subScene);

        // The sub-scene contains only the specified views.
        EXPECT_EQ(subScene.viewCount(), kViewCountInSubset);
        const auto& viewIdsInSubScene = subScene.viewIds();
        // Verify that all views in the subset are in the main scene and
        // in the input views for the subset.
        for (const auto& viewId : viewIdsInSubScene) {
            EXPECT_TRUE(subViewIds.contains(viewId));

            // Ensure equality of the view objects.
            const auto viewInMain = scene.view(viewId);
            const auto viewInSub = subScene.view(viewId);
            EXPECT_NE(viewInMain, nullptr);
            EXPECT_NE(viewInSub, nullptr);
            EXPECT_EQ(viewInMain->estimated(), viewInSub->estimated());
            // We only check the focal length in order to verify that the
            // Camera object was copied correctly.
            EXPECT_EQ(viewInMain->camera().focalLength(),
                      viewInSub->camera().focalLength());

            // Verify that the tracks exist in both the main-scene and sub-scene
            const auto trackIdsInViewInSub = viewInSub->trackIds();
            for (const auto& trackId : trackIdsInViewInSub) {
                const auto featureInSub = viewInSub->featureOf(trackId);
                const auto featureInMain = viewInMain->featureOf(trackId);
                EXPECT_NE(featureInSub, nullptr);
                EXPECT_NE(featureInMain, nullptr);
                EXPECT_EQ(featureInSub->x(), featureInMain->x());
                EXPECT_EQ((*featureInSub).y(), (*featureInMain).y());
            }
        }

        // Verify that all tracks in sub-scene are valid.
        const auto trackIdsInSub = subScene.trackIds();
        for (const auto& trackId : trackIdsInSub) {
            const auto trackInMain = scene.track(trackId);
            const auto trackInSub = subScene.track(trackId);
            EXPECT_NE(trackInMain, nullptr);
            EXPECT_NE(trackInSub, nullptr);
            EXPECT_EQ((trackInSub->position() - trackInMain->position()).norm(),
                      0.);

            // Ensure that all views observing the subset's track are actually
            // in the subset.
            const auto& viewIdsObservingTrackInSub = trackInSub->viewIds();
            for (const auto& viewId : viewIdsObservingTrackInSub) {
                EXPECT_TRUE(subViewIds.contains(viewId));
            }
        }

        // Ensure that RemoveView works properly.
        for (const auto& viewId : subViewIds) {
            ASSERT_TRUE(subScene.removeView(viewId));
        }
    }
}
