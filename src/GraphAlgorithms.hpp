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
template <class vertex_list_ = std::deque<int>,
          class search_structure_ = std::unordered_set<int>,
          class vertex_list_list = std::deque<std::deque<int>>,
          class vertex_list_category = std::deque<std::pair<int, int>>>
struct Traits
{
  typedef vertex_list_ vertices_list_trait;
  typedef search_structure_ search_structure_trait;
  typedef vertex_list_list vertices_list_list_trait;
  typedef vertex_list_category vertex_list_categ;

  // static_assert(vertex_list_list::value_type == )
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct Traversal
{
  typedef typename graph_type::equivalence_cmp equivalent_elements;
  typedef typename traits::vertices_list_trait vertices_list;
  typedef typename traits::search_structure_trait search_structure;

  /// Breadth first graph traversal (only explores the current connected component in which crt_vertex resides)
  static void breadthFirst(const graph_type& my_graph, int crt_vertex, vertices_list& node_labels)
  {
    typename graph_type::neighbors_list vertex_queue;
    search_structure visited_nodes;
    std::pair<typename search_structure::iterator, bool> inserted_it;

    vertex_queue.push_back(crt_vertex);
    my_graph.getLinks(crt_vertex, vertex_queue);

    while (vertex_queue.size() > 0)
    {
      inserted_it = visited_nodes.insert(vertex_queue.front());
      if (!inserted_it.second)
        vertex_queue.pop_front();
      else
      {
        auto front_elem = vertex_queue.front();
        node_labels.push_back(front_elem);
        vertex_queue.pop_front();
        if (vertex_queue.size() > 0)
          my_graph.getLinks(front_elem, vertex_queue);
      }
    }
  }

  /// Depth first graph traversal (only explores the current connected component in which crt_vertex resides)
  static void depthFirst(const graph_type& my_graph, int crt_vertex, vertices_list& node_labels)
  {
    search_structure visited_nodes;
    dfRecursive(my_graph, crt_vertex, node_labels, visited_nodes);
  }

private :

  static void dfRecursive(const graph_type& my_graph, int crt_vertex, 
                          vertices_list& node_labels, search_structure& visited_nodes)
  {
    typename graph_type::neighbors_list crt_vertex_queue;
    std::pair<typename search_structure::iterator, bool> inserted_it;

    inserted_it = visited_nodes.insert(crt_vertex);
    if (inserted_it.second)
      node_labels.push_back(crt_vertex);

    my_graph.getLinks(crt_vertex, crt_vertex_queue);

    typename graph_type::neighbors_list::const_iterator crt_vertex_it(crt_vertex_queue.begin());
    typename graph_type::neighbors_list::const_iterator end(crt_vertex_queue.end());

    for (; crt_vertex_it != end; ++crt_vertex_it)
    {
      if (visited_nodes.find(*crt_vertex_it) == visited_nodes.end())
        dfRecursive(my_graph, *crt_vertex_it, node_labels, visited_nodes);
    }
  }
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct Features
{
  typedef typename traits::vertices_list_trait vertices_list;
  typedef typename traits::search_structure_trait search_structure;
  typedef typename traits::vertices_list_list_trait vertex_lists;
  typedef typename traits::vertex_list_categ vertex_categ;

  /// Compute the internal/external degree of a node (for undirected graphs the internal and external degree are the same)
  static unsigned int computeDegree(const graph_type& my_input_graph, 
                                    int input_vertex, DegreeType degr_option = EXTERNAL)
  {
    unsigned int vertices_nb = my_input_graph.noVertices();

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
        case graph::algorithms::EXTERNAL:
        default:
        {
          for (int i = 0; i < static_cast<int>(vertices_nb); ++i)
            if (my_input_graph.hasLink(input_vertex, i))
              ++comp_degree;
        }
        break;
        case graph::algorithms::INTERNAL:
        {
          for (int i = 0; i < static_cast<int>(vertices_nb); ++i)
            if (my_input_graph.hasLink(i, input_vertex))
              ++comp_degree;
        }
        break;
      }
    }
    else
    {
      for (int i = 0; i < static_cast<int>(vertices_nb); ++i)
        if (my_input_graph.hasLink(input_vertex, i))
          ++comp_degree;
    }

    return comp_degree;
  }

  /// Extract Connected components (returns the nb of connected components and the lists of vertices)
  static unsigned int connectedComponents(const graph_type& my_input_graph, 
                                          vertex_lists& node_labels)
  {
    return connectedCompsHelper(my_input_graph, &node_labels);
  }

  /// Checks if the graph is connected
  static bool isConnected(const graph_type& my_input_graph)
  {
    auto nb_connected_comps = connectedCompsHelper(my_input_graph);
    return (nb_connected_comps == 1);
  }

  /// Check if the graph is bipartite (returns a vertex list with the category)
  static bool isBipartite(const graph_type& my_input_graph, vertex_categ& vertex_list)
  {
    /*typename graph_type::neighbors_list vertex_queue;
    search_structure visited_nodes;
    std::pair<typename search_structure::iterator, bool> inserted_it;

    vertex_queue.push_back(crt_vertex);
    my_graph.getLinks(crt_vertex, vertex_queue);

    auto categ_left  = 0;
    auto categ_right = 1;
    auto s_categ = 1;
    auto crt_categ = categ_left;

    while (vertex_queue.size() > 0)
    {
      inserted_it = visited_nodes.insert(vertex_queue.front());
      if (!inserted_it.second)
      {
        vertex_queue.pop_front();
      }
      else
      {
        auto front_elem = vertex_queue.front();
        vertex_list.push_back(std::make_pair(front_elem, crt_categ));
        vertex_queue.pop_front();
        if (vertex_queue.size() > 0)
        {
          my_graph.getLinks(front_elem, vertex_queue);
        }
      }
    }*/
  }

  /// Checks if the there is at least a cycle in the subgraph that contains the specified vertex
  static bool hasCycles(const graph_type& my_input_graph, int crt_vertex)
  {
    search_structure visited_nodes;
    if (graph_type::directed_graph)
    {
      search_structure rec_chain_visited;
      return dfCycleDetectedD(my_input_graph, crt_vertex, visited_nodes, rec_chain_visited);
    }
    else
    {
      return dfCycleDetectedU(my_input_graph, crt_vertex, visited_nodes);
    }
  }

  /// Check Complete graph (basically verifies if the number of edges has reached the maximum possible)
  static bool completeGraph(const graph_type& my_input_graph)
  {
    unsigned int maxEdges = my_input_graph.maxNoEdges();
    unsigned int noEdges = my_input_graph.noEdges();

    return (noEdges == maxEdges);
  }

  /// Compute Transitive closure (Roy-Warshall -> places 1 as the cost for the vertex connections)
  static void transitiveClosure(const graph_type& my_input_graph, graph::common::SquareMatrix<bool>& transitive_closure)
  {
    unsigned int vertices_nb = my_input_graph.noVertices();
    transitive_closure.setSize(vertices_nb, false);
    int i = 0, j = 0, k = 0;

    for (i = 0; i < vertices_nb; ++i)
    {
      for (j = 0; j < vertices_nb; ++j)
        transitive_closure(i, j) = my_input_graph.hasLink(i, j);
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

private :

  static unsigned int connectedCompsHelper(const graph_type& my_input_graph, 
                                           vertex_lists* node_labels=nullptr)
  {
    search_structure visitedVertices;
    vertices_list crtNodeLabels;
    unsigned int nbConnComponents = 0;
    
    auto nbVertices = my_input_graph.noVertices();
    for (auto i = 0 ; i < nbVertices; ++i)
    {
      if (visitedVertices.find(i) == visitedVertices.end())
      {
        ++nbConnComponents;
        Traversal<graph_type, traits>::breadthFirst(my_input_graph, i, crtNodeLabels);
        if (node_labels)
        {
          node_labels->push_back(crtNodeLabels);
        }
        for (const auto& elem : crtNodeLabels)
        {
          visitedVertices.insert(elem);
        }
        crtNodeLabels.clear();
      }
    }

    return nbConnComponents;
  }

  static bool dfCycleDetectedU(const graph_type& my_graph, int crt_vertex, 
                               search_structure& visited_nodes, int parent_vertex = -1)
  {
    // static_assert(graph_type::directed_graph == 0, "Invalid graph type");

    typename graph_type::neighbors_list crt_vertex_queue;
    std::pair<typename search_structure::iterator, bool> inserted_it;

    inserted_it = visited_nodes.insert(crt_vertex);
    my_graph.getLinks(crt_vertex, crt_vertex_queue);

    typename graph_type::neighbors_list::const_iterator crt_vertex_it(crt_vertex_queue.begin());
    typename graph_type::neighbors_list::const_iterator end(crt_vertex_queue.end());

    if (parent_vertex < 0)
    {
      parent_vertex = crt_vertex;
    }

    for (; crt_vertex_it != end; ++crt_vertex_it)
    {
      // exclude the parent from the list of neighbors
      if (*crt_vertex_it == parent_vertex)
      {
        continue;
      }

      if (visited_nodes.find(*crt_vertex_it) == visited_nodes.end())
      {
        bool result = dfCycleDetectedU(my_graph, *crt_vertex_it, visited_nodes, crt_vertex);
        // end exploration once a cycle has been found
        if (result)
        {
          return result;
        }
      }
      else
      {
        return true;
      }
    }

    // case for an isolated node
    return false;
  }

  static bool dfCycleDetectedD(const graph_type& my_graph, int crt_vertex, 
                               search_structure& visited_nodes, 
                               search_structure& rec_chain_visited)
  {
    // static_assert(graph_type::directed_graph == 1, "Invalid graph type");

    typename graph_type::neighbors_list crt_vertex_queue;
    std::pair<typename search_structure::iterator, bool> inserted_it;

    inserted_it = visited_nodes.insert(crt_vertex);
    my_graph.getLinks(crt_vertex, crt_vertex_queue);

    // keep track of the vertexes that are in the crt recursion chain
    rec_chain_visited.insert(crt_vertex);

    typename graph_type::neighbors_list::const_iterator crt_vertex_it(crt_vertex_queue.begin());
    typename graph_type::neighbors_list::const_iterator end(crt_vertex_queue.end());

    for (; crt_vertex_it != end; ++crt_vertex_it)
    {
      if (rec_chain_visited.find(*crt_vertex_it) != rec_chain_visited.end())
      {
        // vertex found in the crt recursive chain - cycle detected
        return true;
      }

      if (visited_nodes.find(*crt_vertex_it) == visited_nodes.end())
      {
        bool cycleDetected = dfCycleDetectedD(my_graph, *crt_vertex_it, visited_nodes, rec_chain_visited);
        // end exploration once a cycle has been found
        if (cycleDetected)
        {
          rec_chain_visited.erase(crt_vertex);
          return cycleDetected;
        }
      }
    }

    rec_chain_visited.erase(crt_vertex);
    return false;
  }
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct MSP
{
  static_assert(graph_type::directed_graph == 0, "Invalid graph type");

  /// Determines the minimum spanning tree for the input graph along with a total connection cost
  static typename graph_type::weight_element_type kruskal(const graph_type& my_graph, graph_type& output_msp)
  {

  }

  /// Prim
  //TODO
};

template <class graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          class traits = Traits<>>
struct Paths
{
  typedef typename graph_type::weight_element_type dist_type;
  typedef typename traits::vertices_list_trait vertices_list;
  typedef std::pair<dist_type, int> dist_nxt;
  enum { null_vertex = -1 };

  /// Djikstra
  //TODO

  /// Compute all the minimum paths (Floyd-Warshall-returns a square matrix with the associated cost)
  static void computePaths(const graph_type& my_input_graph, graph::common::SquareMatrix<dist_type>& cost_output)
  {
    dist_type max_val = std::numeric_limits<dist_type>::max() / 1000;
    auto vertices_nb = my_input_graph.noVertices();
    cost_output.setSize(vertices_nb, 0);
    typename graph_type::weight_element_type crt_dist = 0;

    int i = 0, j = 0, k = 0;

    for (i = 0; i < vertices_nb; ++i)
    {
      for (j = 0; j < vertices_nb; ++j)
      {
        if (i != j)
        {
          if (my_input_graph.hasLink(i, j))
            cost_output(i, j) = my_input_graph.getWeight(i, j);
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

  /// Compute all the minimum paths (Floyd-Warshall-returns a square matrix with the associated cost and a matrix with
  /// the next vertex for reconstructing the minimum paths)
  static void computePathsTrails(const graph_type& my_input_graph, graph::common::SquareMatrix<dist_type>& cost_output,
                                 graph::common::SquareMatrix<int>& nxt_matrix)
  {
    dist_type max_val = std::numeric_limits<dist_type>::max() / 1000;
    auto vertices_nb = my_input_graph.noVertices();

    cost_output.setSize(vertices_nb, 0);
    nxt_matrix.setSize(vertices_nb, null_vertex);

    typename graph_type::weight_element_type crt_dist = 0;

    unsigned int i = 0, j = 0, k = 0;

    for (i = 0; i < vertices_nb; ++i)
    {
      for (j = 0; j < vertices_nb; ++j)
      {
        if (i != j)
        {
          if (my_input_graph.hasLink(i, j))
          {
            cost_output(i, j) = my_input_graph.getWeight(i, j);
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

  /// Extract a path from the specified square matrix (after calling computePathsTrails)
  static bool extractPath(const graph::common::SquareMatrix<int>& mat_paths, int i, int j,
                          vertices_list& sequence_trail)
  {
    sequence_trail.clear();

    auto vertices_nb = mat_paths.getSize();
    if (i >= static_cast<int>(vertices_nb) || j >= static_cast<int>(vertices_nb) || i < 0 || j < 0)
    {
      assert(false && "Invalid vertices specified");
      return false;
    }

    auto nxt_vertex = mat_paths.accessElementCst(i, j);
    if (nxt_vertex == null_vertex)
      return false;

    sequence_trail.push_back(i);
    while (nxt_vertex != null_vertex)
    {
      sequence_trail.push_back(nxt_vertex);
      nxt_vertex = mat_paths.accessElementCst(nxt_vertex, j);
    }

    return true;
  }

  /// Bellman-Ford
  //TODO
};

ALGO_END
GRAPH_END

#endif // !_graph_algorithms_h_