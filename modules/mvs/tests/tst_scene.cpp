#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tMvs/Scene>

using namespace tl;

namespace {
const std::vector<std::string> kViewNames{"1", "2", "3"};
const std::vector<Feature> kFeatures{Feature(1., 1.), Feature(2., 2.),
                                     Feature(3., 3.)};
} // namespace

TEST(Scene, ViewIdFromNameValid)
{
    Scene scene;
    const auto viewId = scene.addView(kViewNames[0], 0.);
    const auto viewIdFromName = scene.viewIdFromName(kViewNames[0]);
    EXPECT_EQ(viewId, viewIdFromName);
}

TEST(Scene, ViewIdFromNameInvalid)
{
    Scene scene;
    EXPECT_EQ(scene.viewIdFromName(kViewNames[0]), kInvalidViewId);
}

TEST(Scene, AddView)
{
    Scene scene;
    const auto viewId = scene.addView(kViewNames[0], 0.);
    EXPECT_NE(viewId, kInvalidViewId);
    EXPECT_EQ(scene.viewCount(), 1);
    EXPECT_EQ(scene.trackCount(), 0);
    // FIXME:
    // The intension is to test "Add view with same name and timestamp",
    // however the logic is not correct for now.
    EXPECT_EQ(scene.addView(kViewNames[0], 0.), kInvalidViewId);
    // The first default cameraId is 0
    EXPECT_EQ(scene.cameraId(viewId), 0);
}

TEST(Scene, AddViewWithSpecificCameraId)
{
    Scene scene;
    constexpr CameraId camId{1};
    const auto viewId = scene.addView(kViewNames[0], 0., camId);
    EXPECT_NE(viewId, kInvalidViewId);
    EXPECT_EQ(scene.viewCount(), 1);
    EXPECT_EQ(scene.trackCount(), 0);
    EXPECT_EQ(scene.cameraCount(), 1);
    EXPECT_EQ(scene.cameraId(viewId), camId);

    // FIXME:
    // The intension is to test "Add view with same name and timestamp",
    // however the logic is not correct for now.
    EXPECT_EQ(scene.addView(kViewNames[0], 0.), kInvalidViewId);
}

TEST(Scene, RemoveView)
{
    Scene scene;
    // Add 2 views with default camId parameter (different cameraId)
    const auto viewId1 = scene.addView(kViewNames[0], 0.);
    const auto viewId2 = scene.addView(kViewNames[1], 1.);
    EXPECT_EQ(scene.viewCount(), 2);
    EXPECT_EQ(scene.cameraCount(), 2);

    // Remove the first cameraId
    const auto camId1 = scene.cameraId(viewId1);
    EXPECT_TRUE(scene.removeView(viewId1));
    EXPECT_EQ(scene.viewCount(), 1);
    EXPECT_EQ(scene.viewIdFromName(kViewNames[0]), kInvalidViewId);
    EXPECT_EQ(scene.view(viewId1), nullptr);
    EXPECT_EQ(scene.cameraId(viewId1), kInvalidCameraId);
    EXPECT_EQ(scene.cameraCount(), 1);

    const auto viewIds1 = scene.sharedCameraViewIds(camId1);
    EXPECT_FALSE(con::Contains(viewIds1, viewId1));

    // Remove the second cameraId
    const auto camId2 = scene.cameraId(viewId2);
    EXPECT_TRUE(scene.removeView(viewId2));
    EXPECT_EQ(scene.viewCount(), 0);
    EXPECT_EQ(scene.viewIdFromName(kViewNames[1]), kInvalidViewId);
    EXPECT_EQ(scene.view(viewId2), nullptr);
    EXPECT_EQ(scene.cameraId(viewId2), kInvalidCameraId);
    EXPECT_EQ(scene.cameraCount(), 0);

    const auto viewIds2 = scene.sharedCameraViewIds(camId2);
    EXPECT_FALSE(con::Contains(viewIds2, viewId2));

    EXPECT_FALSE(scene.removeView(kInvalidViewId));
    EXPECT_FALSE(scene.removeView(viewId1));
}

TEST(Scene, GetViewValid)
{
    Scene scene;
    const auto viewId = scene.addView(kViewNames[0], 0.);
    EXPECT_NE(viewId, kInvalidViewId);

    const auto constView = scene.view(viewId);
    EXPECT_NE(constView, nullptr);

    auto mutableView = scene.rView(viewId);
    EXPECT_NE(mutableView, nullptr);
}

TEST(Scene, GetViewValidInvalid)
{
    constexpr ViewId invalidViewId{0};

    Scene scene;
    const auto constView = scene.view(invalidViewId);
    EXPECT_EQ(constView, nullptr);

    auto mutableView = scene.rView(invalidViewId);
    EXPECT_EQ(mutableView, nullptr);
}

TEST(Scene, GetViewsFromCameraId)
{
    constexpr double kFocalLength1{800.0};
    constexpr double kFocalLength2{1200.0};

    Scene scene;
    const auto viewId1 = scene.addView(kViewNames[0], 0., CameraId{0});
    const auto camId1 = scene.cameraId(viewId1);

    // Add a second view with to the same camera Id.
    const auto viewId2 = scene.addView(kViewNames[1], 1., camId1);
    const auto camId2 = scene.cameraId(viewId2);
    EXPECT_EQ(camId1, camId2);

    // Add a third view this is with a different camera Id.
    const auto viewId3 = scene.addView(kViewNames[2], 2., CameraId{2});
    const auto camId3 = scene.cameraId(viewId3);
    EXPECT_NE(camId1, camId3);
    EXPECT_EQ(scene.cameraCount(), 2);

    // Change a value in view 1's camera intrinsics and ensure that it
    // propagates to view 2.
    auto& camera1 = scene.rView(viewId1)->rCamera();
    auto& camera2 = scene.rView(viewId2)->rCamera();
    auto& camera3 = scene.rView(viewId3)->rCamera();
    camera1.setFocalLength(kFocalLength1);
    EXPECT_EQ(camera1.focalLength(), camera2.focalLength());
    EXPECT_NE(camera1.focalLength(), camera3.focalLength());

    // Alter the intrinsics through camera 2 and ensure that camera 1 is updated
    // and camera 3 is not.
    camera2.setFocalLength(kFocalLength2);
    EXPECT_EQ(camera1.focalLength(), camera2.focalLength());
    EXPECT_NE(camera2.focalLength(), camera3.focalLength());
}

TEST(Scene, CameraIds)
{
    Scene scene;
    const auto viewId1 = scene.addView(kViewNames[0], 0.);
    const auto camId1 = scene.cameraId(viewId1);

    // Add a second view with to the same cameraId.
    const auto viewId2 = scene.addView(kViewNames[1], 1., camId1);
    const auto camId2 = scene.cameraId(viewId2);
    EXPECT_EQ(camId1, camId2);

    // Add a third view with default cameraId, which would use a new camId
    const auto viewId3 = scene.addView(kViewNames[2], 2.);
    const auto camId3 = scene.cameraId(viewId3);
    EXPECT_NE(camId1, camId3);
    EXPECT_EQ(scene.cameraCount(), 2);

    // Ensure that the previous cameras are added correctly.
    const auto camIds = scene.cameraIds();
    EXPECT_EQ(camIds.size(), 2);

    EXPECT_TRUE(con::Contains(camIds, camId1));
    EXPECT_TRUE(con::Contains(camIds, camId3));
}

TEST(Scene, AddFeatureValid)
{
    Scene scene;

    const auto viewId1 = scene.addView(kViewNames[0], 0.);
    const auto viewId2 = scene.addView(kViewNames[1], 1.);
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
    EXPECT_EQ(feature1->pos.x(), kFeatures[0].pos.x());
    EXPECT_EQ(feature1->pos.y(), kFeatures[0].pos.y());

    const auto feature2 = view2->featureOf(trackId);
    EXPECT_EQ(feature2, nullptr);

    // Ensure that the observation adds the correct information to the track.
    const auto track = scene.track(trackId);
    EXPECT_EQ(track->viewCount(), 1);
    EXPECT_TRUE(track->viewIds().contains(viewId1));
}

TEST(Scene, AddFeatureInvalid)
{
    Scene scene;

    const auto viewId1 = scene.addView(kViewNames[0], 0.);
    const auto viewId2 = scene.addView(kViewNames[1], 1.);
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

TEST(Scene, AddTrackEmpty)
{
    Scene scene;
    const auto trackId = scene.addTrack();
    EXPECT_NE(trackId, kInvalidTrackId);
}

TEST(Scene, AddTrackValid)
{
    Scene scene;

    const Scene::TrackObservation obs{{0, kFeatures[0]}, {1, kFeatures[1]}};
    EXPECT_NE(scene.addView(kViewNames[0], 0.), kInvalidViewId);
    EXPECT_NE(scene.addView(kViewNames[1], 1.), kInvalidViewId);

    const auto trackId = scene.addTrack(obs);
    EXPECT_NE(trackId, kInvalidTrackId);
    EXPECT_TRUE(scene.track(trackId));
    EXPECT_EQ(scene.trackCount(), 1);
}

TEST(Scene, AddTrackInvalid)
{
    Scene scene;

    // Should fail with less than two views.
    const Scene::TrackObservation invalidObs{{0, kFeatures[0]}};
    EXPECT_NE(scene.addView(kViewNames[0], 0.), kInvalidViewId);
    EXPECT_EQ(scene.addTrack(invalidObs), kInvalidTrackId);
    EXPECT_EQ(scene.trackCount(), 0);
}

TEST(Scene, RemoveTrackValid)
{
    Scene scene;

    const Scene::TrackObservation track{{0, kFeatures[0]}, {1, kFeatures[1]}};

    // Should be able to successfully remove the track.
    EXPECT_NE(scene.addView(kViewNames[0], 0.), kInvalidViewId);
    EXPECT_NE(scene.addView(kViewNames[1], 1.), kInvalidViewId);
    const auto trackId = scene.addTrack(track);
    EXPECT_TRUE(scene.removeTrack(trackId));
}

TEST(Scene, RemoveTrackInvalid)
{
    Scene scene;

    // Should return false when trying to remove a track not in the scene.
    EXPECT_FALSE(scene.removeTrack(kInvalidTrackId));
}

TEST(Scene, GetTrackValid)
{
    Scene scene;

    const Scene::TrackObservation track{{0, kFeatures[0]}, {1, kFeatures[1]}};
    EXPECT_NE(scene.addView(kViewNames[0], 0.), kInvalidViewId);
    EXPECT_NE(scene.addView(kViewNames[1], 1.), kInvalidViewId);

    const auto trackId = scene.addTrack(track);
    EXPECT_NE(trackId, kInvalidTrackId);

    const auto constTrack = scene.track(trackId);
    EXPECT_NE(constTrack, nullptr);

    auto mutableTrack = scene.rTrack(trackId);
    EXPECT_NE(mutableTrack, nullptr);
}

TEST(Scene, GetTrackInvalid)
{
    Scene scene;

    const Scene::TrackObservation track = {};
    const auto trackId = scene.addTrack(track);
    EXPECT_EQ(trackId, kInvalidTrackId);

    const auto constTrack = scene.track(trackId);
    EXPECT_EQ(constTrack, nullptr);

    auto mutableTrack = scene.rTrack(trackId);
    EXPECT_EQ(mutableTrack, nullptr);
}

TEST(Scene, GetSubScene)
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
        Scene::TrackObservation track;
        for (int j{0}; j < kNumObservationsPerTrack; j++) {
            track.insert({(i + j) % kViewCount, Feature{}});
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
