#include <random>

#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/RandomGenerator>
#include <tMvs/ViewGraph>

namespace tl {
inline bool operator==(const ViewPairInfo& lhs, const ViewPairInfo& rhs)
{
    return lhs.position == rhs.position && lhs.rotation == rhs.rotation &&
           lhs.num_verified_matches == rhs.num_verified_matches;
}
} // namespace tl

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector3d;

TEST(ViewGraph, Constructor)
{
    ViewGraph viewGraph;
    EXPECT_EQ(viewGraph.numViews(), 0);
    EXPECT_EQ(viewGraph.numEdges(), 0);
}

TEST(ViewGraph, RemoveView)
{
    ViewGraph viewGraph;

    const ViewId viewId1 = 0;
    const ViewId viewId2 = 1;
    const ViewPairInfo edge;

    viewGraph.addEdge(viewId1, viewId2, edge);
    EXPECT_TRUE(viewGraph.hasView(viewId1));
    EXPECT_TRUE(viewGraph.hasView(viewId2));
    EXPECT_EQ(viewGraph.numViews(), 2);
    EXPECT_EQ(viewGraph.numEdges(), 1);

    viewGraph.removeView(viewId1);
    EXPECT_TRUE(!viewGraph.hasView(viewId1));
    EXPECT_EQ(viewGraph.numViews(), 1);
    EXPECT_EQ(viewGraph.numEdges(), 0);
}

TEST(ViewGraph, AddEdge)
{
    ViewPairInfo info_01;
    info_01.num_verified_matches = 1;
    ViewPairInfo info_02;
    info_02.num_verified_matches = 2;

    ViewGraph viewGraph;
    viewGraph.addEdge(0, 1, info_01);
    viewGraph.addEdge(0, 2, info_02);

    EXPECT_EQ(viewGraph.numViews(), 3);
    EXPECT_EQ(viewGraph.numEdges(), 2);

    const std::vector<int> expected_ids = {0, 1, 2};
    std::unordered_set<ViewId> view_ids = viewGraph.viewIds();

    const auto* edge_01 = viewGraph.edge(0, 1);
    const auto* edge_02 = viewGraph.edge(0, 2);
    EXPECT_TRUE(edge_01);
    EXPECT_EQ(*edge_01, info_01);
    EXPECT_TRUE(edge_02);
    EXPECT_EQ(*edge_02, info_02);
    EXPECT_TRUE(!viewGraph.edge(1, 2));

    // Interanally always save as [smaller, bigger]
    const auto* edge_10 = viewGraph.edge(1, 0);
    const auto* edge_20 = viewGraph.edge(2, 0);
    EXPECT_TRUE(edge_10);
    EXPECT_EQ(*edge_10, info_01);
    EXPECT_EQ(*edge_20, info_02);
    EXPECT_TRUE(!viewGraph.edge(2, 1));

    const auto neighborIds = *viewGraph.neighborViewIds(0);
    EXPECT_TRUE(neighborIds.contains(1));
    EXPECT_TRUE(neighborIds.contains(2));

    ViewPairInfo info_12;
    info_12.num_verified_matches = 3;
    viewGraph.addEdge(1, 2, info_12);
    EXPECT_EQ(viewGraph.numViews(), 3);
    EXPECT_EQ(viewGraph.numEdges(), 3);
    EXPECT_TRUE(viewGraph.edge(1, 2));
    EXPECT_EQ(*viewGraph.edge(1, 2), info_12);
}

TEST(ViewGraph, RemoveEdge)
{
    ViewPairInfo info_01;
    info_01.num_verified_matches = 1;
    ViewPairInfo info_02;
    info_02.num_verified_matches = 2;

    ViewGraph viewGraph;
    viewGraph.addEdge(0, 1, info_01);
    viewGraph.addEdge(0, 2, info_02);

    EXPECT_TRUE(viewGraph.edge(0, 1));
    EXPECT_TRUE(viewGraph.edge(0, 2));

    EXPECT_EQ(viewGraph.numViews(), 3);
    EXPECT_EQ(viewGraph.numEdges(), 2);

    EXPECT_FALSE(viewGraph.removeEdge(1, 2));

    EXPECT_TRUE(viewGraph.removeEdge(0, 2));
    EXPECT_EQ(viewGraph.numEdges(), 1);
    EXPECT_TRUE(!viewGraph.edge(0, 2));
}

TEST(ViewGraph, ExtractSubgraph_Basic)
{
    // Create known graph that is a square with all edges connected.
    ViewPairInfo info;
    ViewGraph viewGraph;
    viewGraph.addEdge(0, 1, info);
    viewGraph.addEdge(0, 2, info);
    viewGraph.addEdge(0, 3, info);
    viewGraph.addEdge(1, 2, info);
    viewGraph.addEdge(1, 3, info);
    viewGraph.addEdge(2, 3, info);

    // Extract subgraph from 3 of the nodes.
    const std::unordered_set<ViewId> sub = {1, 2, 3};
    ViewGraph subgraph;
    viewGraph.ExtractSubgraph(sub, &subgraph);

    // Ensure correct edges.
    EXPECT_EQ(subgraph.numViews(), 3);
    EXPECT_EQ(subgraph.numEdges(), 3);
    EXPECT_TRUE(subgraph.hasView(1));
    EXPECT_TRUE(subgraph.hasView(2));
    EXPECT_TRUE(subgraph.hasView(3));
    EXPECT_TRUE(subgraph.hasEdge(1, 2));
    EXPECT_TRUE(subgraph.hasEdge(1, 3));
    EXPECT_TRUE(subgraph.hasEdge(2, 3));
}

TEST(ViewGraph, ExtractSubgraph_Large)
{
    constexpr int kNumViews = 100;
    constexpr int kNumSubgraphViews = 100;
    constexpr int kNumEdgesPerView = 20;

    // Create a subgraph by adding K random edges for each node.
    ViewPairInfo info;
    ViewGraph viewGraph;
    RandomNumberGenerator rng(181);
    for (int i = 0; i < kNumViews; i++) {
        // Add random edges for each node, ensuring that no self-edges are
        // added.
        for (int j = 0; j < kNumEdgesPerView; j++) {
            const int random_view = rng.randInt(0, kNumViews - 1);
            if (i != random_view) {
                viewGraph.addEdge(i, random_view, info);
            }
        }
    }

    // Extract subgraph from some of the nodes.
    std::unordered_set<ViewId> subgraph_nodes;
    for (int i = 0; i < kNumSubgraphViews; i++) {
        subgraph_nodes.emplace(i);
    }

    ViewGraph subgraph;
    viewGraph.ExtractSubgraph(subgraph_nodes, &subgraph);

    // Ensure correct edges.
    EXPECT_EQ(subgraph.numViews(), subgraph_nodes.size());
    const auto& subgraph_view_ids = subgraph.viewIds();
    for (const auto& viewId : subgraph_view_ids) {
        // Ensure the graph view is actually in the input subgraph.
        EXPECT_TRUE(subgraph_nodes.contains(viewId));
        // Ensure all edges to the node are correct.
        const auto& edges_in_graph = *viewGraph.neighborViewIds(viewId);
        int num_edges_to_view_in_subgraph = 0;
        for (const auto& neighbor_in_graph : edges_in_graph) {
            // If both views are in the subgraph, ensure the edge exists in the
            // subgraph.
            if (subgraph_nodes.contains(neighbor_in_graph)) {
                ++num_edges_to_view_in_subgraph;
                EXPECT_TRUE(subgraph.hasEdge(viewId, neighbor_in_graph));
            }
        }

        // Ensure that the number of subgraph edges counted in the graph is
        // the same as the number of edges present in the subgraph.
        const auto& edges_to_view_in_subgraph =
            *subgraph.neighborViewIds(viewId);
        EXPECT_EQ(num_edges_to_view_in_subgraph,
                  edges_to_view_in_subgraph.size());
    }
}

TEST(RemoveDisconnectedViewPairs, SingleConnectedComponent)
{
    ViewPairInfo info;

    ViewGraph view_graph;
    view_graph.addEdge(0, 1, info);
    view_graph.addEdge(1, 2, info);
    view_graph.addEdge(2, 3, info);

    const auto removed_views = view_graph.removeDisconnectedViewPairs();
    EXPECT_EQ(view_graph.numEdges(), 3);
    EXPECT_EQ(removed_views.size(), 0);
}

TEST(ViewGraph, RemoveDisconnectedViewPairs_TwoConnectedComponents)
{
    const ViewPairInfo info;

    ViewGraph viewGraph;
    viewGraph.addEdge(0, 1, info);
    viewGraph.addEdge(1, 2, info);
    viewGraph.addEdge(2, 3, info);
    viewGraph.addEdge(5, 6, info);
    viewGraph.addEdge(6, 7, info);

    const auto removedViewIds = viewGraph.removeDisconnectedViewPairs();
    EXPECT_EQ(viewGraph.numEdges(), 3);
    EXPECT_EQ(removedViewIds.size(), 3);
    EXPECT_TRUE(con::Contains(removedViewIds, 5));
    EXPECT_TRUE(con::Contains(removedViewIds, 6));
    EXPECT_TRUE(con::Contains(removedViewIds, 7));
}

TEST(ViewGraph, RemoveDisconnectedViewPairs_ThreeConnectedComponents)
{
    const ViewPairInfo info;

    ViewGraph viewGraph;
    viewGraph.addEdge(0, 1, info);
    viewGraph.addEdge(1, 2, info);
    viewGraph.addEdge(2, 3, info);
    viewGraph.addEdge(5, 6, info);
    viewGraph.addEdge(7, 8, info);

    const auto removedViewIds = viewGraph.removeDisconnectedViewPairs();
    EXPECT_EQ(viewGraph.numEdges(), 3);
    EXPECT_EQ(removedViewIds.size(), 4);
    EXPECT_TRUE(con::Contains(removedViewIds, 5));
    EXPECT_TRUE(con::Contains(removedViewIds, 6));
    EXPECT_TRUE(con::Contains(removedViewIds, 7));
    EXPECT_TRUE(con::Contains(removedViewIds, 8));
}

class ViewGraphAlgo : public ::testing::Test
{
protected:
    void SetUp() override {}

    void TearDown() override {}

    void TestOrientationFromMST()
    {
        const auto orientations = createViewIdWithOrientations(_opts.numViews);

        const auto viewGraph = createViewGraph(_opts.numEdges, orientations);

        std::unordered_map<ViewId, Vector3d> orientations_est;
        viewGraph.OrientationsFromMaximumSpanningTree(&orientations_est);
        verifyOrientations(viewGraph, orientations, orientations_est);
    }

protected:
    struct
    {
        int numViews = 5;
        int numEdges = 10;
    } _opts;

private:
    std::unordered_map<ViewId, Vector3d> createViewIdWithOrientations(
        int numViews) const
    {
        std::unordered_map<ViewId, Vector3d> orientations;
        orientations.reserve(numViews);
        orientations.emplace(0, Vector3d::Zero());
        for (int i = 1; i < numViews; i++) {
            orientations.emplace(i, Vector3d::Random());
        }
        return orientations;
    }

    Eigen::Vector3d relativeRotationFromOrientations(
        const Eigen::Vector3d& rvec1, const Eigen::Vector3d& rvec2) const
    {
        Matrix3d R_1, R_2;
        ceres::AngleAxisToRotationMatrix(rvec1.data(), R_1.data());
        ceres::AngleAxisToRotationMatrix(rvec2.data(), R_2.data());
        const Matrix3d R_21 = R_2 * R_1.transpose();

        Vector3d rvec_21;
        ceres::RotationMatrixToAngleAxis(R_21.data(), rvec_21.data());

        return rvec_21;
    }

    ViewPairInfo createViewPairInfo(
        const std::unordered_map<ViewId, Vector3d>& orientations,
        ViewId viewId1, ViewId viewId2) const
    {
        ViewPairInfo info;
        info.rotation = relativeRotationFromOrientations(
            con::FindOrDie(orientations, viewId1),
            con::FindOrDie(orientations, viewId2));

        if (viewId1 > viewId2) {
            info.rotation *= -1.;
        }

        info.num_verified_matches = 100;
        return info;
    }

    ViewGraph createViewGraph(
        int numEdges,
        const std::unordered_map<ViewId, Vector3d>& orientations) const
    {
        CHECK_GE(numEdges, orientations.size() - 1);

        ViewGraph viewGraph;

        // Add a skeletal graph.
        std::vector<ViewId> viewIds;
        viewIds.push_back(0);
        for (int i = 1; i < orientations.size(); i++) {
            const auto info = createViewPairInfo(orientations, i - 1, i);
            viewGraph.addEdge(i - 1, i, info);
            viewIds.push_back(i);
        }

        // Add extra edges.
        std::random_device rd;
        std::mt19937 rng{rd()};
        while (viewGraph.numEdges() < numEdges) {
            std::shuffle(viewIds.begin(), viewIds.end(), rng);
            if (viewGraph.edge(viewIds[0], viewIds[1])) {
                continue;
            }

            const auto info =
                createViewPairInfo(orientations, viewIds[0], viewIds[1]);
            viewGraph.addEdge(viewIds[0], viewIds[1], info);
        }

        return viewGraph;
    }

    void verifyOrientations(
        const ViewGraph& viewGraph,
        const std::unordered_map<ViewId, Vector3d>& gt_orientations,
        const std::unordered_map<ViewId, Vector3d>& orientations) const
    {
        constexpr double kTolerance = 1e-12;
        for (const auto& [viewPairId, viewPairInfo] : viewGraph.viewPairs()) {
            const auto& gt_orientation1 =
                con::FindOrDie(gt_orientations, viewPairId.first);
            const auto& gt_orientation2 =
                con::FindOrDie(gt_orientations, viewPairId.second);
            const auto gt_relative_rotation = relativeRotationFromOrientations(
                gt_orientation1, gt_orientation2);

            const auto& orientation1 =
                con::FindOrDie(orientations, viewPairId.first);
            const auto& orientation2 =
                con::FindOrDie(orientations, viewPairId.second);
            const auto relative_rotation =
                relativeRotationFromOrientations(orientation1, orientation2);

            EXPECT_LT((gt_relative_rotation - relative_rotation).norm(),
                      kTolerance);
        }
    }
};

TEST_F(ViewGraphAlgo, OrientationFromMST_SmallScale)
{
    _opts.numViews = 4;
    _opts.numEdges = 6;

    TestOrientationFromMST();
}

TEST_F(ViewGraphAlgo, OrientationFromMST_LargeScale)
{
    _opts.numViews = 50;
    _opts.numEdges = 100;

    TestOrientationFromMST();
}
