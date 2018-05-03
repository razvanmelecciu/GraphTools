#ifndef _graph_algorithms_h_
#define _graph_algorithms_h_

#include "GraphBase.hpp"
#include <queue>
#include <set>

GRAPH_START
ALGO_START

template <StorageMethod storage_method = ADJACENCY_MATRIX,
          class distance_type = int,
          class eq_elems = graph::equivalent_elements<distance_type>,
          class graph_type = GraphContainer<storage_method, distance_type, eq_elems> >
struct Traversal
{
  typedef eq_elems equivalent_elements;
  typedef std::set<std::size_t> vertex_list;

  /// Breadth first graph traversal (only explores the current connected component in which crt_vertex resides)
  static void BreadthFirst(const graph_type& my_graph, std::size_t crt_vertex, vertex_list& node_labels)
  {
    typename graph_type::neighbors_list crt_vertex_queue;

    node_labels.insert(crt_vertex);
    my_graph.GetLinks(crt_vertex, crt_vertex_queue);

    while (crt_vertex_queue.size() > 0)
    {
      node_labels.insert(crt_vertex_queue.front());
      my_graph.GetLinks(crt_vertex_queue.front(), crt_vertex_queue);
      crt_vertex_queue.pop();
    }
  }

  /// Depth first graph traversal (only explores the current connected component in which crt_vertex resides)
  static void DepthFirst(const graph_type& my_graph, std::size_t crt_vertex, vertex_list& node_labels)
  {
    typename graph_type::neighbors_list crt_vertex_queue;
    my_graph.GetLinks(crt_vertex, crt_vertex_queue);

    if (crt_vertex_queue.size() == 0)
    {
      node_labels.insert(crt_vertex);
      return;
    }

    while (crt_vertex_queue.size() > 0)
    {
      DepthFirst(my_graph, crt_vertex_queue.front(), node_labels);
      crt_vertex_queue.pop();
    }
  }
};


ALGO_END
GRAPH_END

#endif // !_graph_algorithms_h_