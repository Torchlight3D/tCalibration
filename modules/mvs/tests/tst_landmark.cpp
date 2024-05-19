#include <gtest/gtest.h>

#include <vector>
#include <Eigen/Core>

#include <tCore/ContainerUtils>
#include <tMvs/Landmark>

using namespace tl;

TEST(Landmark, Default)
{
    Track track;
    EXPECT_EQ(track.viewCount(), 0);
    EXPECT_EQ(track.position(), Eigen::Vector4d::Zero());
}

TEST(Landmark, Estimated)
{
    Track track;
    EXPECT_TRUE(!track.estimated());
    track.setEstimated(true);
    EXPECT_TRUE(track.estimated());
    track.setEstimated(false);
    EXPECT_TRUE(!track.estimated());
}

TEST(Landmark, Views)
{
    Track track;
    const std::vector<ViewId> viewIds = {0, 1, 2};
    // Test that no views exist.
    EXPECT_EQ(track.viewCount(), 0);

    // Add views.
    for (size_t i{0}; i < viewIds.size(); i++) {
        track.addView(viewIds[i]);
        EXPECT_EQ(track.viewCount(), i + 1);
    }

    for (size_t i{0}; i < viewIds.size(); i++) {
        track.removeView(viewIds[i]);
        EXPECT_EQ(track.viewCount(), 3 - i - 1);
    }
}

TEST(Landmark, ViewIds)
{
    Track track;
    const std::vector<ViewId> viewIds = {0, 1, 2};

    // Add views.
    for (const auto& viewId : viewIds) {
        track.addView(viewId);
    }

    // Make sure the track ids are equivalent.
    std::unordered_set<ViewId> viewIdsInTrack = track.viewIds();
    EXPECT_EQ(viewIdsInTrack.size(), 3);
    for (size_t i{0}; i < viewIds.size(); i++) {
        EXPECT_TRUE(viewIdsInTrack.contains(viewIds[i]));
    }
}

TEST(Landmark, InverseDepth)
{
    Track track;
    const std::vector<ViewId> viewIds = {0, 1, 2};
    // Test that no views exist.
    EXPECT_EQ(track.viewCount(), 0);
    EXPECT_TRUE(track.referenceViewId() == kInvalidViewId);

    // Add views.
    for (size_t i{0}; i < viewIds.size(); i++) {
        track.addView(viewIds[i]);
        EXPECT_EQ(track.viewCount(), i + 1);
    }

    EXPECT_TRUE(track.referenceViewId() == viewIds[0]);

    *track.rInverseDepth() = 1.0;
    EXPECT_TRUE(*track.inverseDepth() == 1.0);

    for (size_t i{0}; i < viewIds.size(); i++) {
        track.removeView(viewIds[i]);
        EXPECT_EQ(track.viewCount(), 3 - i - 1);
        if (i == viewIds.size() - 1) {
            EXPECT_TRUE(track.referenceViewId() == kInvalidViewId);
        }
        else {
            EXPECT_TRUE(track.referenceViewId() != kInvalidViewId);
        }
    }
}

TEST(Landmark, Descriptor)
{
    Track track;
    // Add views.
    track.addView(0);
    Eigen::Vector3f descVec{1.f, 2.f, 3.f};
    track.setReferenceDescriptor(descVec);
    for (int i{0}; i < 3; ++i) {
        EXPECT_FLOAT_EQ(track.referenceDescriptor()[i], descVec[i]);
    }

    Eigen::VectorXf descMat = Eigen::VectorXf::Random(128);
    track.setReferenceDescriptor(descMat);
    for (int i{0}; i < 3; ++i) {
        EXPECT_FLOAT_EQ(track.referenceDescriptor()[i], descMat[i]);
    }
}
