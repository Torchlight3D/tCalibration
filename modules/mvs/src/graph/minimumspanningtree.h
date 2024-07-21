#pragma once

#include "connectedcomponents.h"

namespace tl {

// A class for extracting the minimum spanning tree of a graph using Kruskal's
// greedy algorithm. The minimum spanning tree is a subgraph that contains all
// nodes in the graph and only the edges that connect these nodes with a minimum
// edge weight summation. The algorithm runs in O(E * log (V) ) where E is the
// number of edges and V is the number of nodes in the graph. For more details
// on the algorithm please see:
//   https://en.wikipedia.org/wiki/Kruskal%27s_algorithm
template <typename T, typename V>
class MinimumSpanningTree
{
public:
    MinimumSpanningTree() = default;

    // Add an edge in the graph.
    void AddEdge(const T& node1, const T& node2, const V& weight)
    {
        edges_.emplace_back(weight, std::pair<T, T>(node1, node2));
    }

    // Extracts the minimum spanning tree. Returns true on success and false
    // upon failure. If true is returned, the output variable contains the edge
    // list of the minimum spanning tree.
    bool Extract(std::unordered_set<std::pair<T, T>>* minimum_spanning_tree)
    {
        if (edges_.empty()) {
            VLOG(2) << "No edges were passed to the minimum spanning tree "
                       "extractor!";
            return false;
        }

        // Determine the number of nodes in the graph.
        const int num_nodes = CountNodesInGraph();

        // Reserve space in the MST since we know it will have exactly N - 1
        // edges.
        minimum_spanning_tree->reserve(num_nodes - 1);

        // Order all edges by their weights.
        std::sort(edges_.begin(), edges_.end());

        // For each edge in the graph, add it to the minimum spanning tree if it
        // does not create a cycle.
        ConnectedComponents<T> cc;
        for (int i = 0;
             i < edges_.size() && minimum_spanning_tree->size() < num_nodes - 1;
             i++) {
            const auto& [_, pair] = edges_[i];
            const auto& [node1, node2] = pair;
            if (!cc.NodesInSameConnectedComponent(node1, node2)) {
                cc.AddEdge(node1, node2);
                minimum_spanning_tree->emplace(node1, node2);
            }
        }

        return minimum_spanning_tree->size() == num_nodes - 1;
    }

private:
    // Counts the number of nodes in the graph by counting the number of unique
    // node values we have received from AddEdge.
    int CountNodesInGraph() const
    {
        std::vector<T> nodes;
        nodes.reserve(edges_.size() * 2);
        for (const auto& [_, pair] : edges_) {
            nodes.emplace_back(pair.first);
            nodes.emplace_back(pair.second);
        }

        std::sort(nodes.begin(), nodes.end());

        return std::distance(nodes.begin(),
                             std::unique(nodes.begin(), nodes.end()));
    }

private:
    std::vector<std::pair<V, std::pair<T, T>>> edges_;

    // Each node is mapped to a Root node. If the node is equal to the root id
    // then the node is a root and the size of the root is the size of the
    // connected component.
    std::unordered_map<T, T> disjoint_set_;

    // DISALLOW_COPY_AND_ASSIGN(MinimumSpanningTree);
};

} // namespace tl
