#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <tMvs/View>

using namespace tl;

TEST(View, Name)
{
    const std::string kName{"0"};
    View view(kName);
    EXPECT_EQ(view.name(), kName);
}

TEST(View, Estimated)
{
    View view;
    EXPECT_TRUE(!view.estimated());
    view.setEstimated(true);
    EXPECT_TRUE(view.estimated());
    view.setEstimated(false);
    EXPECT_TRUE(!view.estimated());
}

TEST(View, Features)
{
    View view;
    const std::vector<TrackId> trackIds = {0, 1, 2};
    const std::vector features = {Feature(0, 0), Feature(1, 1), Feature(2, 2)};
    // Test that no features exist.
    EXPECT_EQ(view.featureCount(), 0);

    // Add features.
    for (size_t i{0}; i < trackIds.size(); i++) {
        view.addFeature(trackIds[i], features[i]);
        const Feature* feature = view.featureOf(trackIds[i]);
        EXPECT_NE(feature, nullptr);
        EXPECT_EQ(*feature, features[i]);
        EXPECT_EQ(view.featureCount(), i + 1);
    }

    for (size_t i{0}; i < trackIds.size(); i++) {
        view.removeFeature(trackIds[i]);
        const Feature* feature = view.featureOf(trackIds[i]);
        EXPECT_EQ(feature, nullptr);
        EXPECT_EQ(view.featureCount(), 3 - i - 1);
    }
}

TEST(View, TrackIds)
{
    View view;
    const std::vector<TrackId> trackIds = {0, 1, 2};
    const std::vector features = {Feature(0, 0), Feature(1, 1), Feature(2, 2)};
    // Add features.
    for (size_t i{0}; i < trackIds.size(); i++) {
        view.addFeature(trackIds[i], features[i]);
    }

    // Make sure the track ids are equivalent.
    std::vector<TrackId> temp_track_ids = view.trackIds();
    std::sort(temp_track_ids.begin(), temp_track_ids.end());
    for (size_t i{0}; i < trackIds.size(); i++) {
        EXPECT_EQ(trackIds[i], temp_track_ids[i]);
    }
}
