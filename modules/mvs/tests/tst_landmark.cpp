#include <array>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tMvs/Landmark>

using namespace tl;

using Eigen::Vector3f;
using Eigen::VectorXf;

namespace {
constexpr std::array<ViewId, 3> kViewIds = {0, 1, 2};
}

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
    // Test that no views exist.
    EXPECT_EQ(track.viewCount(), 0);

    // Add views.
    for (size_t i{0}; i < kViewIds.size(); i++) {
        track.addView(kViewIds[i]);
        EXPECT_EQ(track.viewCount(), i + 1);
    }

    for (size_t i{0}; i < kViewIds.size(); i++) {
        track.removeView(kViewIds[i]);
        EXPECT_EQ(track.viewCount(), 3 - i - 1);
    }
}

TEST(Landmark, ViewIds)
{
    Track track;

    // Add views
    for (const auto& viewId : kViewIds) {
        track.addView(viewId);
    }

    // Make sure the track ids in view are equivalent
    const auto& viewIdsInTrack = track.viewIds();
    EXPECT_EQ(viewIdsInTrack.size(), kViewIds.size());
    for (const auto& viewId : kViewIds) {
        EXPECT_TRUE(viewIdsInTrack.contains(viewId));
    }
}

TEST(Landmark, InverseDepth)
{
    Track track;
    // Test that no views exist
    EXPECT_EQ(track.viewCount(), 0);
    EXPECT_TRUE(track.referenceViewId() == kInvalidViewId);

    // Add views
    for (size_t i{0}; i < kViewIds.size(); i++) {
        track.addView(kViewIds[i]);
        EXPECT_EQ(track.viewCount(), i + 1);
    }

    // This test is weird???
    EXPECT_TRUE(track.referenceViewId() == kViewIds[0]);

    // Set inverse depth
    constexpr auto kInverseDepth{1.};
    *track.rInverseDepth() = kInverseDepth;
    EXPECT_TRUE(*track.inverseDepth() == kInverseDepth);

    for (size_t i{0}; i < kViewIds.size(); i++) {
        track.removeView(kViewIds[i]);
        EXPECT_EQ(track.viewCount(), 3 - i - 1);
        if (i == kViewIds.size() - 1) {
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
    const Vector3f kFixDescriptor{1.f, 2.f, 3.f};
    track.setReferenceDescriptor(kFixDescriptor);
    for (int i{0}; i < kFixDescriptor.size(); ++i) {
        EXPECT_FLOAT_EQ(track.referenceDescriptor()[i], kFixDescriptor[i]);
    }

    const VectorXf kRandomDescriptor = VectorXf::Random(128);
    track.setReferenceDescriptor(kRandomDescriptor);
    for (int i{0}; i < kRandomDescriptor.size(); ++i) {
        EXPECT_FLOAT_EQ(track.referenceDescriptor()[i], kRandomDescriptor[i]);
    }
}
