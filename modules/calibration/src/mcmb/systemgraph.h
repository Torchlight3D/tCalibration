#pragma once

#include <graaflib/graph.h>

namespace tl::mcmb {

// Node type: Id (int), Edge: double
class Graph
{
public:
    void addVertex(int id);
    size_t vertexCount() const;

    void addEdge(int id1, int id2, double weight);
    size_t edgeCount() const;

    std::vector<std::vector<int>> connectedComponents() const;
    std::vector<int> shortestPathBetween(int id1, int id2) const;

private:
    using graph_t = graaf::undirected_graph<int, double>;
    graph_t _g;
    std::unordered_map<int, graaf::vertex_id_t> _idToIndex;
};

} // namespace tl::mcmb
