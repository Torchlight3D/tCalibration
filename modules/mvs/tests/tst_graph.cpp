#include <gtest/gtest.h>

#include <tCore/HashUtils>
#include <tCore/RandomGenerator>
#include <tMvs/Graph/ConnectedComponents>
#include <tMvs/Graph/MinimumSpanningTree>
#include <tMvs/Graph/TripletExtractor>

using namespace tl;

using IntPair = std::pair<int, int>;
using IntTriplet = std::tuple<int, int, int>;

namespace {
RandomNumberGenerator kRNG{49};
}

TEST(Graph, ConnectedComponents_FullyConnectedGraph)
{
    ConnectedComponents<int> cc;
    for (int i = 0; i < 9; i++) {
        cc.AddEdge(i, i + 1);
    }

    std::unordered_map<int, std::unordered_set<int>> disjoint_sets;
    cc.Extract(&disjoint_sets);

    EXPECT_EQ(disjoint_sets.size(), 1);
    EXPECT_EQ(disjoint_sets.begin()->second.size(), 10);
}

TEST(Graph, ConnectedComponents_FullyDisconnectedGraph)
{
    ConnectedComponents<int> cc;
    for (int i = 0; i < 9; i++) {
        cc.AddEdge(i, i + 100);
    }

    std::unordered_map<int, std::unordered_set<int>> disjoint_sets;
    cc.Extract(&disjoint_sets);

    EXPECT_EQ(disjoint_sets.size(), 9);
    for (const auto& [_, neighbors] : disjoint_sets) {
        EXPECT_EQ(neighbors.size(), 2);
    }
}

TEST(Graph, ConnectedComponents_PartiallyConnectedGraph)
{
    ConnectedComponents<int> cc;
    for (int i = 0; i < 9; i++) {
        cc.AddEdge(i, i + 5);
    }

    std::unordered_map<int, std::unordered_set<int>> disjoint_sets;
    cc.Extract(&disjoint_sets);

    EXPECT_EQ(disjoint_sets.size(), 5);
    for (const auto& [_, neighbors] : disjoint_sets) {
        EXPECT_GE(neighbors.size(), 2);
    }
}

TEST(Graph, ConnectedComponents_LimitComponentSize)
{
    ConnectedComponents<int> connected_components(2);
    for (int i = 0; i < 9; i++) {
        connected_components.AddEdge(i, i + 1);
    }

    std::unordered_map<int, std::unordered_set<int>> disjoint_sets;
    connected_components.Extract(&disjoint_sets);

    EXPECT_EQ(disjoint_sets.size(), 5);
    for (const auto& [_, neighbors] : disjoint_sets) {
        EXPECT_LE(neighbors.size(), 2);
    }
}

TEST(Graph, ConnectedComponents_RandomOrder)
{
    std::vector<IntPair> intPairsToAdd;
    for (int i = 0; i < 10; i++) {
        intPairsToAdd.emplace_back(i, i + 1);
    }

    std::random_device rd;
    std::mt19937 rng{rd()};
    for (int i = 0; i < 25; i++) {
        std::shuffle(intPairsToAdd.begin(), intPairsToAdd.end(), rng);

        ConnectedComponents<int> cc;
        for (const auto& intPair : intPairsToAdd) {
            cc.AddEdge(intPair.first, intPair.second);
        }

        std::unordered_map<int, std::unordered_set<int>> disjoint_sets;
        cc.Extract(&disjoint_sets);

        EXPECT_EQ(disjoint_sets.size(), 1);
    }
}

TEST(Graph, ExtractTriplet_NoTriplets)
{
    std::unordered_set<IntPair> edges;
    edges.emplace(0, 1);
    edges.emplace(1, 2);
    edges.emplace(2, 3);

    TripletExtractor<int> extractor;
    std::vector<std::vector<IntTriplet>> triplets;
    extractor.ExtractTriplets(edges, &triplets);
    EXPECT_EQ(triplets.size(), 0);
}

TEST(Graph, ExtractTriplet_TwoTripletsOneSet)
{
    std::unordered_set<IntPair> edges;
    edges.emplace(0, 1);
    edges.emplace(1, 2);
    edges.emplace(2, 3);
    edges.emplace(0, 3);
    edges.emplace(1, 3);
    // Add an edge to the view graph that will not be part of any triplets.
    edges.emplace(3, 4);

    TripletExtractor<int> triplet_extractor;
    std::vector<std::vector<IntTriplet>> triplets;
    triplet_extractor.ExtractTriplets(edges, &triplets);

    EXPECT_EQ(triplets.size(), 1);
    EXPECT_EQ(triplets.at(0).size(), 2);
}

TEST(Graph, ViewTriplet_DisconnectedSets)
{
    std::unordered_set<IntPair> edges;
    edges.emplace(0, 1);
    edges.emplace(1, 2);
    edges.emplace(0, 2);
    edges.emplace(0, 3);
    edges.emplace(0, 4);
    edges.emplace(3, 4);

    TripletExtractor<int> extractor;
    std::vector<std::vector<IntTriplet>> triplets;
    extractor.ExtractTriplets(edges, &triplets);

    EXPECT_EQ(triplets.size(), 2);
    EXPECT_EQ(triplets[0].size(), 1);
    EXPECT_EQ(triplets[1].size(), 1);
}

void VerifyMST(const std::unordered_map<IntPair, double>& edges,
               const std::unordered_set<IntPair>& mst, double mst_cost)
{
    // Check that all nodes were added using connected component.
    std::unordered_set<int> nodes;
    for (const auto& [edge, _] : edges) {
        nodes.emplace(edge.first);
        nodes.emplace(edge.second);
    }

    ConnectedComponents<int> cc_mst;
    for (const auto& edge : mst) {
        cc_mst.AddEdge(edge.first, edge.second);
    }

    std::unordered_map<int, std::unordered_set<int>> components_mst;
    cc_mst.Extract(&components_mst);

    // Verify that there is only 1 connected component in the MST.
    EXPECT_EQ(components_mst.size(), 1);

    // Verify that the MST covers all nodes.
    const auto& cc_iterator = components_mst.begin();
    EXPECT_EQ(nodes.size(), cc_iterator->second.size());
    for (const auto& node : cc_iterator->second) {
        EXPECT_TRUE(nodes.contains(node));
    }

    // Verify it is the MST.
    double cost = 0;
    for (const auto& edge : mst) {
        cost += con::FindOrDieNoPrint(edges, edge);
    }

    EXPECT_EQ(cost, mst_cost);
}

TEST(Graph, MinimumSpanningTree_Basic)
{
    MinimumSpanningTree<int, double> mst;
    std::unordered_map<IntPair, double> edges;
    for (int i = 0; i < 9; i++) {
        mst.AddEdge(i, i + 1, 1.);
        edges.emplace(IntPair{i, i + 1}, 1.);
    }

    const double mst_cost = edges.size() * 1.;

    std::unordered_set<IntPair> spanning_tree;
    EXPECT_TRUE(mst.Extract(&spanning_tree));
    VerifyMST(edges, spanning_tree, mst_cost);
}

TEST(Graph, MinimumSpanningTree_RandomGraph)
{
    const int num_vertices = 10;
    const int num_edges = 30;

    MinimumSpanningTree<int, double> mst;
    std::unordered_map<IntPair, double> edges;

    // Create the MST first.
    for (int i = 0; i < num_vertices - 1; i++) {
        mst.AddEdge(i, i + 1, 1.0);
        edges.emplace(IntPair(i, i + 1), 1.0);
    }
    const double mst_cost = edges.size() * 1.0;

    // Add extra edges with a higher edge weight.
    while (edges.size() != num_edges) {
        const int random_node1 = kRNG.randInt(0, num_vertices - 1);
        const int random_node2 = kRNG.randInt(0, num_vertices - 1);
        if (random_node1 == random_node2 ||
            edges.contains(IntPair(random_node1, random_node2)) ||
            edges.contains(IntPair(random_node2, random_node1))) {
            continue;
        }

        const double edge_weight = kRNG.randFloat(2.0, 10.0);
        mst.AddEdge(random_node1, random_node2, edge_weight);
        edges.emplace(IntPair(random_node1, random_node2), edge_weight);
    }

    std::unordered_set<IntPair> spanning_tree;
    EXPECT_TRUE(mst.Extract(&spanning_tree));
    VerifyMST(edges, spanning_tree, mst_cost);
}

TEST(Graph, MinimumSpanningTree_NoEdges)
{
    MinimumSpanningTree<int, double> mst;
    std::unordered_set<IntPair> spanning_tree;
    EXPECT_FALSE(mst.Extract(&spanning_tree));
}

TEST(Graph, MinimumSpanningTree_DisconnectedGraph)
{
    // Create a graph: 0 -> 1 -> 2  and 3 -> 4 -> 5
    MinimumSpanningTree<int, double> mst;
    mst.AddEdge(0, 1, 1.0);
    mst.AddEdge(1, 2, 1.0);
    mst.AddEdge(3, 4, 1.0);
    mst.AddEdge(4, 5, 1.0);
    std::unordered_set<IntPair> spanning_tree;
    EXPECT_FALSE(mst.Extract(&spanning_tree));
}
