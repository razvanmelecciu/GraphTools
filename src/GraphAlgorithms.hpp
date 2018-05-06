#ifndef _graph_algorithms_h_
#define _graph_algorithms_h_

#include "GraphBase.hpp"
#include <deque>
#include <unordered_set>

GRAPH_START
ALGO_START

/// Default trait class that defines the vertex list (deque, vector, list) and the search structure used (set, unordered_set)
template <class vertex_list_ = std::deque<std::size_t> ,
          class search_structure_ = std::unordered_set<std::size_t> >
struct Traits
{
  typedef vertex_list_ vertices_list_trait;
  typedef search_structure_ search_structure_trait;
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct Traversal
{
  typedef typename graph_type::equivalence_cmp equivalent_elements;
  typedef typename traits::vertices_list_trait vertices_list;
  typedef typename traits::search_structure_trait search_structure;

  /// Breadth first graph traversal (only explores the current connected component in which crt_vertex resides)
  static void BreadthFirst(const graph_type& my_graph, std::size_t crt_vertex, vertices_list& node_labels)
  {
    typename graph_type::neighbors_list vertex_queue;
    search_structure visited_nodes;
    typename search_structure::_Pairib inserted_it;

    vertex_queue.push_back(crt_vertex);
    my_graph.GetLinks(crt_vertex, vertex_queue);

    while (vertex_queue.size() > 0)
    {
      inserted_it = visited_nodes.insert(vertex_queue.front());
      if (!inserted_it.second)
        vertex_queue.pop_front();
      else
      {
        node_labels.push_back(vertex_queue.front());
        vertex_queue.pop_front();
        if (vertex_queue.size() > 0)
          my_graph.GetLinks(vertex_queue.front(), vertex_queue);
      }
    }
  }

  /// Depth first graph traversal (only explores the current connected component in which crt_vertex resides)
  static void DepthFirst(const graph_type& my_graph, std::size_t crt_vertex, vertices_list& node_labels)
  {
    std::unordered_set<std::size_t> visited_nodes;
    DFRecursive(my_graph, crt_vertex, node_labels, visited_nodes);
  }

private :

  static void DFRecursive(const graph_type& my_graph, std::size_t crt_vertex, vertices_list& node_labels, search_structure& visited_nodes)
  {
    typename graph_type::neighbors_list crt_vertex_queue;
    typename search_structure::_Pairib inserted_it;

    inserted_it = visited_nodes.insert(crt_vertex);
    if (inserted_it.second)
      node_labels.push_back(crt_vertex);

    my_graph.GetLinks(crt_vertex, crt_vertex_queue);

    typename graph_type::neighbors_list::const_iterator crt_vertex_it(crt_vertex_queue.begin());
    typename graph_type::neighbors_list::const_iterator end(crt_vertex_queue.end());

    for (; crt_vertex_it != end; ++crt_vertex_it)
    {
      if (visited_nodes.find(*crt_vertex_it) == visited_nodes.end())
        DFRecursive(my_graph, *crt_vertex_it, node_labels, visited_nodes);
    }
  }
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct Features
{
  /// Extract Connected components
  //TODO

  /// Extract disjoint cycles
  //TODO

  /// Check bipartite
  //TODO

  /// Check Complete graph
  //TODO

  /// Compute Tranzitive closure (Roy-Warshall)
  //TODO

  /// Check Euler graph
  //TODO

  /// Check Hamilton graph
  //TODO
};


template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct MSP
{
  static_assert(graph_type::directed_graph == 0, "Invalid graph type");
  /// Kruskal
  //TODO

  /// Prim
  //TODO
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct Paths
{
  /// Djikstra
  //TODO

  /// Floyd-Warshall
  //TODO

  /// Bellman-Ford
  //TODO
};

ALGO_END
GRAPH_END

#endif // !_graph_algorithms_h_