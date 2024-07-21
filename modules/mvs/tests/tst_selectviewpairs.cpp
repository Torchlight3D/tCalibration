#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/RandomGenerator>
#include <tMvs/ViewGraph>
#include <tMvs/ViewPairInfo>
#include <tMvs/SfM/SelectViewPairs>

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {
RandomNumberGenerator kRNG{55};
}

class FilterViewPairsTest : public ::testing::Test
{
protected:
    void SetUp() override {}

    void TearDown() override {}

    void FilterByOrientation()
    {
        constexpr double kMaxRelativeRotationDiff = 2.; // deg

        const auto orientations =
            createViewsOfRandomOrientations(_opts.numViews);

        ViewGraph viewGraph;
        createValidViewPairs(_opts.numValidViewPairs, orientations, &viewGraph);
        createInvalidViewPairs(_opts.numInvalidViewPairs, orientations,
                               &viewGraph);

        FilterViewPairs::ByOrientations(orientations, kMaxRelativeRotationDiff,
                                        &viewGraph);

        EXPECT_EQ(viewGraph.numEdges(), _opts.numValidViewPairs);
    }

    void FilterByRelativeTranslation()
    {
        std::unordered_map<ViewId, Vector3d> orientations;
        std::unordered_map<ViewId, Vector3d> positions;
        createViewsOfRandomPoses(_opts.numViews, &orientations, &positions);

        ViewGraph viewGraph;
        createValidViewPairs(_opts.numValidViewPairs, orientations, positions,
                             &viewGraph);
        createInvalidViewPairs(_opts.numInvalidViewPairs, orientations,
                               positions, &viewGraph);

        FilterViewPairs::ByRelativeTranslationOptions options;
        options.rng = std::make_shared<RandomNumberGenerator>(169);
        FilterViewPairs::ByRelativeTranslations(options, orientations,
                                                &viewGraph);

        EXPECT_GE(viewGraph.numEdges(), _opts.numValidViewPairs);
    }

    void FilterByRelativeTranslationInLine()
    {
        constexpr int kNumViews = 4;
        constexpr int kNumValidViewPairs = 3;

        std::unordered_map<ViewId, Vector3d> orientations;
        std::unordered_map<ViewId, Vector3d> positions;
        for (int i = 0; i < kNumViews; i++) {
            orientations[i] = Vector3d::Zero();
            positions[i] = Vector3d{static_cast<double>(i), 0., 0.};
        }

        ViewGraph viewGraph;
        createValidViewPairs(kNumValidViewPairs, orientations, positions,
                             &viewGraph);

        // Add invalid view pairs.
        {
            const ViewIdPair viewPairId{0, 3};
            ViewPairInfo info;
            // Force the bad translations to be really bad.
            info.position = Vector3d{-1., -1., -1.}.normalized();
            viewGraph.addEdge(0, 3, info);
        }

        FilterViewPairs::ByRelativeTranslationOptions options;
        options.rng = std::make_shared<RandomNumberGenerator>(199);
        options.translation_projection_tolerance = 0.1;
        FilterViewPairs::ByRelativeTranslations(options, orientations,
                                                &viewGraph);

        EXPECT_EQ(viewGraph.numEdges(), kNumValidViewPairs);
    }

    void ExtractMaximalParallelSubgraph()
    {
        std::unordered_map<ViewId, Vector3d> orientations;
        std::unordered_map<ViewId, Vector3d> positions;
        createViewsOfRandomPoses(_opts.numViews, &orientations, &positions);

        ViewGraph viewGraph;
        createValidViewPairs(_opts.numValidViewPairs, orientations, positions,
                             &viewGraph);
        createInvalidViewPairs(_opts.numInvalidViewPairs, orientations,
                               positions, &viewGraph);

        ExtractMaximallyParallelRigidSubgraph(orientations, &viewGraph);

        EXPECT_GE(viewGraph.numEdges(), _opts.numValidViewPairs);
        EXPECT_EQ(viewGraph.numViews(), _opts.numViews);
    }

protected:
    struct
    {
        int numViews = 10;
        int numValidViewPairs = 5;
        int numInvalidViewPairs = 10;
    } _opts;

private:
    std::unordered_map<ViewId, Eigen::Vector3d> createViewsOfRandomOrientations(
        int numViews) const
    {
        std::unordered_map<ViewId, Vector3d> orientations;
        orientations.reserve(numViews);
        orientations.emplace(0, Vector3d::Zero());
        for (auto i{1}; i < numViews; ++i) {
            orientations.emplace(i, Vector3d::Random());
        }

        return orientations;
    }

    void createViewsOfRandomPoses(
        int numViews, std::unordered_map<ViewId, Eigen::Vector3d>* orientations,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions) const
    {
        orientations->clear();
        orientations->reserve(numViews);
        positions->clear();
        positions->reserve(numViews);
        orientations->emplace(0, Vector3d::Zero());
        positions->emplace(0, Vector3d::Zero());
        for (auto i{1}; i < numViews; ++i) {
            orientations->emplace(i, Vector3d::Random());
            positions->emplace(i, Vector3d::Random());
        }
    }

    // FIXME: Maybe duplicated
    void RelativeRotationFromOrientations(const Eigen::Vector3d& rotation1,
                                          const Eigen::Vector3d& rotation2,
                                          Eigen::Vector3d* rotation_21) const
    {
        Matrix3d R_1, R_2;
        ceres::AngleAxisToRotationMatrix(rotation1.data(), R_1.data());
        ceres::AngleAxisToRotationMatrix(rotation2.data(), R_2.data());
        const Matrix3d R_21 = R_2 * R_1.transpose();
        ceres::RotationMatrixToAngleAxis(R_21.data(), rotation_21->data());
    }

    ViewPairInfo createViewPairInfo(
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        const ViewIdPair& viewPairId) const
    {
        ViewPairInfo info;
        RelativeRotationFromOrientations(
            con::FindOrDie(orientations, viewPairId.first),
            con::FindOrDie(orientations, viewPairId.second), &info.rotation);
        return info;
    }

    ViewPairInfo createViewPairInfo(
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        const std::unordered_map<ViewId, Eigen::Vector3d>& positions,
        const ViewIdPair& viewPairId) const
    {
        const auto& viewId1 = viewPairId.first;
        const auto& viewId2 = viewPairId.second;

        ViewPairInfo info;

        Matrix3d R_1, R_2;
        ceres::AngleAxisToRotationMatrix(
            con::FindOrDie(orientations, viewId1).data(), R_1.data());
        ceres::AngleAxisToRotationMatrix(
            con::FindOrDie(orientations, viewId2).data(), R_2.data());
        const Matrix3d R_21 = R_2 * R_1.transpose();
        ceres::RotationMatrixToAngleAxis(R_21.data(), info.rotation.data());

        const Vector3d t_12 = (con::FindOrDie(positions, viewId2) -
                               con::FindOrDie(positions, viewId1))
                                  .normalized();
        info.position = R_1 * t_12;

        return info;
    }

    void createValidViewPairs(
        int numValidViewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        ViewGraph* viewGraph) const
    {
        // Add a skeletal graph.
        std::vector<ViewId> viewIds;
        viewIds.push_back(0);
        for (int i = 1; i < orientations.size(); i++) {
            const ViewIdPair viewPairId{i - 1, i};
            const auto info = createViewPairInfo(orientations, viewPairId);
            viewGraph->addEdge(i - 1, i, info);
            viewIds.push_back(i);
        }

        // Add extra edges.
        std::random_device rd;
        std::mt19937 rng{rd()};
        while (viewGraph->numEdges() < numValidViewPairs) {
            std::shuffle(viewIds.begin(), viewIds.end(), rng);
            const ViewIdPair viewPairId{viewIds[0], viewIds[1]};
            if (viewGraph->hasEdge(viewPairId)) {
                continue;
            }

            const auto info = createViewPairInfo(orientations, viewPairId);
            viewGraph->addEdge(viewPairId.first, viewPairId.second, info);
        }
    }

    void createValidViewPairs(
        int numValidViewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        const std::unordered_map<ViewId, Eigen::Vector3d>& positions,
        ViewGraph* viewGraph) const
    {
        // Add a skeletal graph.
        std::vector<ViewId> viewIds;
        viewIds.push_back(0);
        for (int i = 1; i < orientations.size(); i++) {
            const ViewIdPair viewPairId{i - 1, i};
            const auto info =
                createViewPairInfo(orientations, positions, viewPairId);
            viewGraph->addEdge(i - 1, i, info);
            viewIds.push_back(i);
        }

        // Add extra edges.
        std::random_device rd;
        std::mt19937 rng{rd()};
        while (viewGraph->numEdges() < numValidViewPairs) {
            std::shuffle(viewIds.begin(), viewIds.end(), rng);
            const ViewIdPair viewPairId{viewIds[0], viewIds[1]};
            if (viewGraph->hasEdge(viewPairId)) {
                continue;
            }

            const auto info =
                createViewPairInfo(orientations, positions, viewPairId);
            viewGraph->addEdge(viewPairId.first, viewPairId.second, info);
        }
    }

    void createInvalidViewPairs(
        int numInvalidViewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        ViewGraph* viewGraph) const
    {
        const int total = viewGraph->numEdges() + numInvalidViewPairs;

        const ViewId maxViewId = static_cast<int>(orientations.size()) - 1;
        while (viewGraph->numEdges() < total) {
            const ViewIdPair viewPairId{kRNG.randInt(0, maxViewId),
                                        kRNG.randInt(0, maxViewId)};

            if (!viewPairId.isValid() || viewGraph->hasEdge(viewPairId)) {
                continue;
            }

            // Create a valid view pair.
            auto info = createViewPairInfo(orientations, viewPairId);
            // Add a lot of noise to it.
            info.rotation += Vector3d::Ones();
            viewGraph->addEdge(viewPairId.first, viewPairId.second, info);
        }
    }

    void createInvalidViewPairs(
        int numInvalidViewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
        const std::unordered_map<ViewId, Eigen::Vector3d>& positions,
        ViewGraph* viewGraph) const
    {
        const int total = viewGraph->numEdges() + numInvalidViewPairs;
        const ViewId maxViewId = static_cast<int>(orientations.size()) - 1;
        while (viewGraph->numEdges() < total) {
            const ViewIdPair viewPairId{kRNG.randInt(0, maxViewId),
                                        kRNG.randInt(0, maxViewId)};

            if (!viewPairId.isValid() || viewGraph->hasEdge(viewPairId)) {
                continue;
            }

            // Create a valid view pair.
            auto info = createViewPairInfo(orientations, positions, viewPairId);
            // Add a lot of noise to it.
            info.rotation += Vector3d::Ones();
            info.position = Vector3d::Random().normalized();
            viewGraph->addEdge(viewPairId.first, viewPairId.second, info);
        }
    }
};

TEST_F(FilterViewPairsTest, ByOrientation_NoBadRotations)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 0;

    FilterByOrientation();
}

TEST_F(FilterViewPairsTest, ByOrientation_FewBadRotations)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 5;

    FilterByOrientation();
}

TEST_F(FilterViewPairsTest, ByOrientation_ManyBadRotations)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 15;

    FilterByOrientation();
}

TEST_F(FilterViewPairsTest, ByRelativeTranslation_NoBadPoses)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 0;

    FilterByRelativeTranslation();
}

TEST_F(FilterViewPairsTest, ByRelativeTranslation_FewBadPoses)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 5;

    FilterByRelativeTranslation();
}

TEST_F(FilterViewPairsTest, ByRelativeTranslation_ManyBadPoses)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 15;

    FilterByRelativeTranslation();
}

TEST_F(FilterViewPairsTest, ByRelativeTranslation_PositionsInLine)
{
    FilterByRelativeTranslationInLine();
}

TEST_F(FilterViewPairsTest, ExtractMaximalParallelSubgraph_NoBadPoses)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 0;

    ExtractMaximalParallelSubgraph();
}

TEST_F(FilterViewPairsTest, ExtractMaximalParallelSubgraph_FewBadPoses)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 5;

    ExtractMaximalParallelSubgraph();
}

TEST_F(FilterViewPairsTest, ExtractMaximalParallelSubgraph_ManyBadPoses)
{
    _opts.numViews = 10;
    _opts.numValidViewPairs = 30;
    _opts.numInvalidViewPairs = 15;

    ExtractMaximalParallelSubgraph();
}
