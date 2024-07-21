#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tMvs/Scene>
#include <tMvs/SfM/TrackBuilder>

using namespace tl;

namespace {
constexpr size_t kMinTrackLength = 2;
}

// Ensure that each track has been added to every view.
void VerifyTracks(const Scene& scene)
{
    for (const auto& trackId : scene.trackIds()) {
        const auto* track = CHECK_NOTNULL(scene.track(trackId));
        for (const auto& viewId : track->viewIds()) {
            const auto* view = CHECK_NOTNULL(scene.view(viewId));
            EXPECT_NE(view->featureOf(trackId), nullptr);
        }
    }
}

TEST(TrackBuilder, ConsistentTracks)
{
    constexpr size_t kMaxTrackLength = 10;

    const ViewId viewIdMatches[][2]{
        {ViewId{0}, ViewId{1}}, {0, 1}, {1, 2}, {1, 2}};

    TrackBuilder builder{kMinTrackLength, kMaxTrackLength};
    for (size_t i{0}; i < std::size(viewIdMatches); ++i) {
        builder.addFeatureMatch(
            viewIdMatches[i][0],
            Feature{static_cast<double>(i), static_cast<double>(i)},
            viewIdMatches[i][1],
            Feature{static_cast<double>(i), static_cast<double>(i)});
    }

    Scene scene;
    scene.addView("0", 0.);
    scene.addView("1", 1.);
    scene.addView("2", 2.);
    builder.buildTracks(&scene);
    VerifyTracks(scene);
    EXPECT_EQ(scene.trackCount(), std::size(viewIdMatches));
}

TEST(TrackBuilder, SingletonTracks)
{
    // Having a small max track length will force a singleton track.
    constexpr size_t kMaxTrackLength = 2;

    const ViewId viewIdMatches[][2]{{ViewId{0}, ViewId{1}}, {1, 2}};

    TrackBuilder builder{kMinTrackLength, kMaxTrackLength};
    for (const auto& match : viewIdMatches) {
        builder.addFeatureMatch(match[0], Feature{0., 0.}, match[1],
                                Feature{0., 0.});
    }

    Scene scene;
    scene.addView("0", 0.);
    scene.addView("1", 1.);
    scene.addView("2", 2.);

    builder.buildTracks(&scene);
    VerifyTracks(scene);
    EXPECT_EQ(scene.trackCount(), 1);
}

TEST(TrackBuilder, InconsistentTracks)
{
    constexpr size_t kMaxTrackLength = 10;

    const ViewId viewIdMatches[][2]{
        {ViewId{0}, ViewId{1}}, {0, 1}, {1, 2}, {1, 2}};

    TrackBuilder builder{kMinTrackLength, kMaxTrackLength};
    for (size_t i{0}; i < std::size(viewIdMatches); i++) {
        builder.addFeatureMatch(viewIdMatches[i][0], Feature{0., 0.},
                                viewIdMatches[i][1], Feature{i + 1., i + 1.});
    }

    Scene scene;
    scene.addView("0", 0.);
    scene.addView("1", 1.);
    scene.addView("2", 2.);

    builder.buildTracks(&scene);

    VerifyTracks(scene);
    EXPECT_EQ(scene.trackCount(), 2);
}

TEST(TrackBuilder, MaxTrackLength)
{
    constexpr size_t kMaxTrackLength = 2;

    const std::array viewIds{ViewId{0}, 1, 2, 3, 4, 5};

    TrackBuilder builder{kMinTrackLength, kMaxTrackLength};
    for (size_t i{0}; i < viewIds.size() - 1; i++) {
        builder.addFeatureMatch(viewIds[i], Feature{0., 0.}, viewIds[i + 1],
                                Feature{0., 0.});
    }

    Scene scene;
    scene.addView("0", 0.);
    scene.addView("1", 1.);
    scene.addView("2", 2.);
    scene.addView("3", 3.);
    scene.addView("4", 4.);
    scene.addView("5", 5.);

    builder.buildTracks(&scene);

    VerifyTracks(scene);
    EXPECT_EQ(scene.trackCount(), 3);
}

TEST(TrackBuilder, MinTrackLength)
{
    constexpr size_t kMaxTrackLength = 10;
    constexpr size_t min_track_length = 3;
    constexpr std::array viewIds{ViewId{0}, 1, 2, 3, 4, 5};

    TrackBuilder builder{min_track_length, kMaxTrackLength};

    // Add one track that is larger than the min track length.
    for (size_t i{0}; i < viewIds.size() - 1; ++i) {
        builder.addFeatureMatch(viewIds[i], Feature{0., 0.}, viewIds[i + 1],
                                Feature{0., 0.});
    }

    // Add another track that is smaller than the min track length.
    builder.addFeatureMatch(viewIds[0], Feature{1., 1.}, viewIds[1],
                            Feature{1., 1.});

    Scene scene;
    scene.addView("0", 0.);
    scene.addView("1", 1.);
    scene.addView("2", 2.);
    scene.addView("3", 3.);
    scene.addView("4", 4.);
    scene.addView("5", 5.);

    builder.buildTracks(&scene);
    VerifyTracks(scene);
    EXPECT_EQ(scene.trackCount(), 1);
}
