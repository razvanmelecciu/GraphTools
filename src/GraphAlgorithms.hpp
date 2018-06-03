#ifndef _graph_algorithms_h_
#define _graph_algorithms_h_

#include "GraphBase.hpp"
#include "GraphMisc.hpp"
#include <deque>
#include <unordered_set>
#include <limits>

GRAPH_START
ALGO_START

enum DegreeType : unsigned char { EXTERNAL = 0x00, INTERNAL };

/// Default trait class that defines the vertex list (deque, vector, list) and the search structure used (set, unordered_set)
template <class vertex_list_ = std::deque<int> ,
          class search_structure_ = std::unordered_set<int> >
struct Traits
{
  typedef vertex_list_ vertices_list_trait;
  typedef search_structure_ search_structure_trait;
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<> >
struct Traversal
{
  typedef typename graph_type::equivalence_cmp equivalent_elements;
  typedef typename traits::vertices_list_trait vertices_list;
  typedef typename traits::search_structure_trait search_structure;

  /// Breadth first graph traversal (only explores the current connected component in which crt_vertex resides)
  static void BreadthFirst(const graph_type& my_graph, int crt_vertex, vertices_list& node_labels)
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
  static void DepthFirst(const graph_type& my_graph, int crt_vertex, vertices_list& node_labels)
  {
    std::unordered_set<int> visited_nodes;
    DFRecursive(my_graph, crt_vertex, node_labels, visited_nodes);
  }

private :

  static void DFRecursive(const graph_type& my_graph, int crt_vertex, vertices_list& node_labels, search_structure& visited_nodes)
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
          class traits = Traits<> >
struct Features
{
  /// Compute the internal/external degree of a node (for undirected graphs the internal and external degree are the same)
  static unsigned int ComputeDegree(const graph_type& my_input_graph, int input_vertex, DegreeType degr_option = EXTERNAL)
  {
    unsigned int vertices_nb = my_input_graph.NoVertices();

    if (input_vertex >= static_cast<int>(vertices_nb) || input_vertex < 0)
    {
      assert(false && "Invalid vertex specified");
      return false;
    }

    unsigned int comp_degree = 0;

    if (graph_type::directed_graph)
    {
      switch (degr_option)
      {
      case graph::EXTERNAL:
      default:
        {
          for (int i = 0; i < static_cast<int>(no_vertices_); ++i)
            if (my_input_graph.HasLink(input_vertex, i))
              ++comp_degree;
        }
        break;
      case graph::INTERNAL:
        {
          for (int i = 0; i < static_cast<int>(no_vertices_); ++i)
            if (my_input_graph.HasLink(i, input_vertex))
              ++comp_degree;
        }
        break;
      }
    }
    else
    {
      for (int i = 0; i < static_cast<int>(no_vertices_); ++i)
        if (my_input_graph.HasLink(input_vertex, i))
          ++comp_degree;
    }

    return comp_degree;
  }

  /// Extract Connected components
  //TODO

  /// Extract disjoint cycles
  //TODO

  /// Check bipartite
  //TODO

  /// Check Complete graph (basically verifies if the number of edges has reached the maximum)
  static bool CompleteGraph(const graph_type& my_input_graph)
  {
    unsigned int maxEdges = my_input_graph.MaxNoEdges();
    unsigned int noEdges = my_input_graph.NoEdges();

    return (noEdges == maxEdges);
  }

  /// Compute Transitive closure (Roy-Warshall -> places 1 as the cost for the vertex connections)
  static void TransitiveClosure(const graph_type& my_input_graph, graph::common::SquareMatrix<bool>& transitive_closure)
  {
    unsigned int vertices_nb = my_input_graph.NoVertices();
    transitive_closure.SetSize(vertices_nb, false);
    int i = 0, j = 0, k = 0;

    for (i = 0; i < vertices_nb; ++i)
    {
      for (j = 0; j < vertices_nb; ++j)
        transitive_closure(i, j) = my_input_graph.HasLink(i, j);
    }

    for (k = 0; k < vertices_nb; ++k)
    {
      for (i = 0; i < vertices_nb; ++i)
      {
        for (j = 0; j < vertices_nb; ++j)
          transitive_closure(i, j) = transitive_closure(i, j) || (transitive_closure(i, k) && transitive_closure(k, j));
      }
    }
  }
};


template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct MSP
{
  static_assert(graph_type::directed_graph == 0, "Invalid graph type");

  /// Determines the minimum spanning tree for the input graph along with a total connection cost
  static typename graph_type::weight_element_type Kruskal(const graph_type& my_graph, graph_type& output_msp)
  {

  }

  /// Prim
  //TODO
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<> >
struct Paths
{
  typedef typename graph_type::weight_element_type dist_type;
  typedef std::pair<dist_type, int> dist_nxt;
  enum { null_vertex = -1 };

  /// Djikstra
  //TODO

  /// Compute all the minimum paths (Floyd-Warshall -> places the distance instead of the edge cost)
  static void ComputePaths(const graph_type& my_input_graph, graph::common::SquareMatrix<dist_type>& cost_output)
  {
    dist_type max_val = std::numeric_limits<dist_type>::max() / 1000;
    auto vertices_nb = my_input_graph.NoVertices();
    cost_output.SetSize(vertices_nb, 0);
    typename graph_type::weight_element_type crt_dist = 0;

    int i = 0, j = 0, k = 0;

    for (i = 0; i < vertices_nb; ++i)
    {
      for (j = 0; j < vertices_nb; ++j)
      {
        if (i != j)
        {
          if (my_input_graph.HasLink(i, j))
            cost_output(i, j) = my_input_graph.GetWeight(i, j);
          else
            cost_output(i, j) = max_val;
        }
      }
    }

    for (k = 0; k < vertices_nb; ++k)
    {
      for (i = 0; i < vertices_nb; ++i)
      {
        for (j = 0; j < vertices_nb; ++j)
        {
          crt_dist = cost_output(i, k) + cost_output(k, j);
          if (crt_dist < cost_output(i, j))
            cost_output(i, j) = crt_dist;
        }       
      }
    }
  }

  /// Compute all the minimum paths (Floyd-Warshall -> places the distance instead of the edge cost)
  static void ComputePathsTrails(const graph_type& my_input_graph, graph::common::SquareMatrix<dist_type>& cost_output,
                                 graph::common::SquareMatrix<int>& nxt_matrix)
  {
    dist_type max_val = std::numeric_limits<dist_type>::max() / 1000;
    auto vertices_nb = my_input_graph.NoVertices();

    cost_output.SetSize(vertices_nb, 0);
    nxt_matrix.SetSize(vertices_nb, null_vertex);

    typename graph_type::weight_element_type crt_dist = 0;

    unsigned int i = 0, j = 0, k = 0;

    for (i = 0; i < vertices_nb; ++i)
    {
      for (j = 0; j < vertices_nb; ++j)
      {
        if (i != j)
        {
          if (my_input_graph.HasLink(i, j))
          {
            cost_output(i, j) = my_input_graph.GetWeight(i, j);
            nxt_matrix(i, j) = j;
          }
          else
            cost_output(i, j) = max_val;
        }
      }
    }

    for (k = 0; k < vertices_nb; ++k)
    {
      for (i = 0; i < vertices_nb; ++i)
      {
        for (j = 0; j < vertices_nb; ++j)
        {
          crt_dist = cost_output(i, k) + cost_output(k, j);
          if (crt_dist < cost_output(i, j))
          {
            cost_output(i, j) = crt_dist;
            nxt_matrix(i, j) = nxt_matrix(i, k);
          }
        }
      }
    }
  }

  /// Extract a path from the specified square matrix (after calling ComputePathsTrails)
  static bool ExtractPath(const graph::common::SquareMatrix<int>& mat_paths, int i, int j, std::deque<int>& sequence_trail)
  {
    sequence_trail.clear();

    auto vertices_nb = mat_paths.GetSize();
    if (i >= static_cast<int>(vertices_nb) || j >= static_cast<int>(vertices_nb) || i < 0 || j < 0)
    {
      assert(false && "Invalid vertices specified");
      return false;
    }

    auto nxt_vertex = mat_paths.AccessElementCst(i, j);
    if (nxt_vertex == null_vertex)
      return false;

    sequence_trail.push_back(i);
    while (nxt_vertex != null_vertex)
    {
      sequence_trail.push_back(nxt_vertex);
      nxt_vertex = mat_paths.AccessElementCst(nxt_vertex, j);
    }

    return true;
  }

  /// Bellman-Ford
  //TODO
};

ALGO_END
GRAPH_END

#endif // !_graph_algorithms_h_