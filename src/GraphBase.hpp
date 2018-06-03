#ifndef _graph_base_h_
#define _graph_base_h_

#include <deque>
#include <utility>
#include <cassert>
#include "GraphCommon.hpp"

GRAPH_START

enum StoragePolicy : unsigned short { ADJACENCY_MATRIX = 0x00, ADJACENCY_LIST };
enum DegreeType : unsigned char { EXTERNAL = 0x00, INTERNAL };
enum LinkType : unsigned short { DIRECTED, UNDIRECTED };

template
<class elem_type>
struct EquivalentElements
{
  bool operator () (elem_type a, elem_type b)
  {
    return (a == b);
  }
};

template
<LinkType link_type>
struct IsDirectedGraph
{
  enum { value = 0 };
};

template
<>
struct IsDirectedGraph<DIRECTED>
{
  enum { value = 1 };
};

/// A graph container class meant for keeping the nodes of a graph and the associated edge costs(must be positive), no algorithms provided here
template <StoragePolicy storage_method,
          LinkType link_type = UNDIRECTED,
          class distance_type = int,
          class eq_elems = EquivalentElements<distance_type> >
class GraphContainer
{
};

/// A partial specialization meant to deal with dense graphs
template <class distance_type,
          LinkType link_type,
          class eq_elems>
class GraphContainer<ADJACENCY_MATRIX, link_type, distance_type, eq_elems>
{
public :

  typedef distance_type weight_element_type;
  typedef std::pair<int, distance_type> vertex_cost;
  typedef std::deque<int> neighbors_list;
  typedef eq_elems equivalence_cmp;

  enum { crt_storage_type = ADJACENCY_MATRIX };
  enum { directed_graph = IsDirectedGraph<link_type>::value };
  enum { undefined_weight = 0 };

  struct Edge
  {
    Edge() : start_(0), end_(0), edge_cost_(0)
    {
    }

    Edge(int i, int j, distance_type cost) : start_(i), end_(j), edge_cost_(cost)
    {
    }

    int start_, end_;
    distance_type edge_cost_;
  };

  /// Ctor
  GraphContainer(unsigned int no_vertices) : no_vertices_(no_vertices)
  {
    unsigned int alloc_size = AllocSize();
    adjacency_matrix_ = new weight_element_type[alloc_size];
    for (unsigned int i = 0; i < alloc_size; ++i)
      *(adjacency_matrix_ + i) = undefined_weight;
  }

  /// Ctor (practically creates a complete graph with n(n-1) edges for digraphs or n(n-1)/2 edges for undirected graphs with the specified weight)
  GraphContainer(unsigned int no_vertices, const distance_type& initial_weight) : no_vertices_(no_vertices)
  {
    unsigned int alloc_size = AllocSize();
    adjacency_matrix_ = new weight_element_type[alloc_size];
    for (unsigned int i = 0; i < alloc_size; ++i)                // safe to put since the vertices don't generally have links to themselves
      *(adjacency_matrix_ + i) = initial_weight;
    for (unsigned int i = 0; i < no_vertices_; ++i)
      MatrixElem(i, i) = undefined_weight;
  }

  /// Copy ctor
  GraphContainer(const GraphContainer& rhs_elem) : no_vertices_(rhs_elem.no_vertices_)
  {
    unsigned int alloc_size = AllocSize();
    adjacency_matrix_ = new weight_element_type[alloc_size];
    for (unsigned int i = 0; i < alloc_size; ++i)
      *(adjacency_matrix_ + i) = *(rhs_elem.adjacency_matrix_ + i);
  }

#ifndef _MOVE_SEMANTICS_OFF
  /// Move ctor
  GraphContainer(GraphContainer&& rhs_elem) : no_vertices_(rhs_elem.no_vertices_)
  {
    if (rhs_elem.adjacency_matrix_)
    {
      adjacency_matrix_ = rhs_elem.adjacency_matrix_;
      rhs_elem.adjacency_matrix_ = nullptr;
    }
  }
#endif

  /// Copy assignment
  GraphContainer& operator = (const GraphContainer& rhs_elem)
  {
    if (this != &rhs_elem)
    {
      if (adjacency_matrix_)
        delete[] adjacency_matrix_; 

      no_vertices_ = rhs_elem.no_vertices;
      unsigned int alloc_size = AllocSize();

      adjacency_matrix_ = new weight_element_type[alloc_size];
      for (unsigned int i = 0; i < alloc_size; ++i)
        *(adjacency_matrix_ + i) = *(rhs_elem.adjacency_matrix_ + i);
    }

    return *this;
  }

#ifndef _MOVE_SEMANTICS_OFF
  /// Move assignment
  GraphContainer& operator = (GraphContainer&& rhs_elem)
  {
    if (this != &rhs_elem)
    {
      if (adjacency_matrix_)
        delete[] adjacency_matrix_;

      no_vertices_ = rhs_elem.no_vertices;
      adjacency_matrix_ = rhs_elem.adjacency_matrix_;
      rhs_elem.adjacency_matrix_ = nullptr;
    }

    return *this;
  }
#endif

  // - Accessors

  /// Get the number of vertices
  unsigned int NoVertices() const
  {
    return no_vertices_;
  }

  /// Count the number of defined edges
  unsigned int NoEdges() const
  {
    unsigned int alloc_size = AllocSize();
    unsigned int no_edges = 0;
    equivalence_cmp equiv;

    for (auto i = 0; i < alloc_size; ++i)
    {
      if (equiv(*(adjacency_matrix_ + i), static_cast<distance_type>(undefined_weight)))
        ++no_edges;
    }

    return no_edges;
  }

  /// Compute the internal/external degree of a node (for undirected graphs the internal and external degree are the same)
  unsigned int ComputeDegree(int input_vertex, DegreeType degr_option = EXTERNAL) const
  {
    if (input_vertex >= static_cast<int>(no_vertices_) || input_vertex < 0)
    {
      assert(false && "Invalid vertex specified");
      return false;
    }

    unsigned int comp_degree = 0;

    if (directed_graph)
    {
      switch (degr_option)
      {
      case graph::EXTERNAL :
      default :
        {
          for (int i = 0; i < static_cast<int>(no_vertices_); ++i)
            if (HasLink(input_vertex, i))
              ++comp_degree;
        }
        break;
      case graph::INTERNAL :
        {
          for (int i = 0; i < static_cast<int>(no_vertices_); ++i)
            if (HasLink(i, input_vertex))
              ++comp_degree;
        }
        break;
      }
    }
    else
    {
      for (int i = 0; i < static_cast<int>(no_vertices_); ++i)
        if (HasLink(input_vertex, i))
          ++comp_degree;
    }

    return comp_degree;
  }

  /// Get the maximum number of edges (refflexivity not included e.g. xRx)
  unsigned int MaxNoEdges() const
  {
    unsigned int max_nb_edge = no_vertices_ * (no_vertices_ - 1);  // digraph
    if (!directed_graph)                                          // graphs (undirected graphs)
      max_nb_edge /= 2;

    return max_nb_edge;
  }

  /// Has link between i and j
  bool HasLink(int i, int j) const
  {
    if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) || i < 0 || j < 0)
    {
      assert(false && "Invalid vertices specified");
      return false;
    }

    equivalence_cmp equiv;

    if (equiv(MatrixElemCst(i, j), static_cast<distance_type>(undefined_weight)))
      return false;
    return true;
  }

  /// Get the weight from node i to node j
  distance_type GetWeight(int i, int j) const
  {
    if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return undefined_weight;
    }

    return static_cast<distance_type>(MatrixElemCst(i, j));
  }

  /// Check if the relation is symmetric relation iRj and jRi (not that the cost is the same)
  bool SymmetricRelation(int i, int j) const
  {
    if (i >= no_vertices_ || j >= no_vertices_ || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return false;
    }

    if (directed_graph)
    {
      equivalence_cmp equiv;

      if (equiv(MatrixElemCst(i, j), undefined_weight) || equiv(MatrixElemCst(j, i), undefined_weight))
        return false;
    }

    return true;
  }

  /// Get the list of links for the current node
  void GetLinks(int vertex, neighbors_list& vertex_list) const
  {
    if (vertex >= static_cast<int>(no_vertices_) || vertex < 0)
    {
      assert(false && "Invalid vertex specified");
      return;
    }

    equivalence_cmp equiv;
    if (directed_graph)
    {
      unsigned int st_offset = vertex * no_vertices_;
      unsigned int end_offset = st_offset + no_vertices_;

      for (unsigned int i = st_offset; i < end_offset; ++i)
      {
        if (!equiv(adjacency_matrix_[i], undefined_weight))
          vertex_list.push_back(i % no_vertices_);
      }
    }
    else
    {
      for (unsigned int i = 0; i < no_vertices_; ++i)
      {
        if (!equiv(MatrixElemCst(vertex, i), undefined_weight))
          vertex_list.push_back(i);
      }
    }
    
  }

  /// Get a list with all the links defined
  void GetAllLinks(std::deque<Edge>& links_list) const
  {
    links_list.clear();
    Edge crtEdge;

    if (directed_graph)
    {
      for (unsigned int i = 0; i < no_vertices_; ++i)
      {
        crtEdge.start_ = i;
        for (unsigned int j = 0; j < no_vertices_; ++j)
        {
          crtEdge.end_ = j;
          crtEdge.edge_cost_ = MatrixElemCst(i, j);
          links_list.push_back(crtEdge);
        }
      }
    }
    else
    {
      for (unsigned int i = 0; i < no_vertices_; ++i)
      {
        crtEdge.start_ = i;
        for (unsigned int j = i; j < no_vertices_; ++j)
        {
          crtEdge.end_ = j;
          crtEdge.edge_cost_ = MatrixElemCst(i, j);
          links_list.push_back(crtEdge);
        }
      }
    }
  }

  // - Mutators

  /// Set a node path from vertex i to vertex j (or both vertices if the relation is symmetric)
  void InsertLink(int i, int j, const distance_type& weight, bool symmetric = false)
  {
    if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return;
    }

    if (weight < 0)
    {
      assert(false && "Invalid weight (must be a positive value)");
      return;
    }

    MatrixElem(i, j) = weight;
    if (directed_graph && symmetric)
      MatrixElem(j, i) = weight;
  }

  /// Remove an edge from vertex i to vertex j
  distance_type ClearLink(int i, int j)
  {
    if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return undefined_weight;
    }

    distance_type prev_value;
    prev_value = MatrixElem(i, j);
    MatrixElem(i, j) = undefined_weight;
    return prev_value;
  }

  /// Clear all edges
  void ClearAllLinks()
  {
    unsigned int alloc_size = AllocSize();
    for (unsigned int i = 0; i < alloc_size; ++i)
      *(adjacency_matrix_ + i) = undefined_weight;
  }
  
protected :

  // - Internals

  /// Get the corresponding matrix element
  weight_element_type& MatrixElem(int i, int j)
  {
    unsigned int offset = 0;
    if (!directed_graph)
    {
      if (i <= j)
      {
        if (i > 0)
          offset = (i * no_vertices_ - (i * (i - 1) / 2));
        offset += (j - i);
      }
      else
      {
        if (j > 0)
          offset = (j * no_vertices_ - (j * (j - 1) / 2));
        offset += (i - j);
      }
    }
    else
      offset = i * no_vertices_ + j;

    return *(adjacency_matrix_ + offset);
  }

  /// Get the corresponding matrix element
  const weight_element_type& MatrixElemCst(int i, int j) const
  {
    unsigned int offset = 0;
    if (!directed_graph)
    {
      if (i <= j)
      {
        if (i > 0)
          offset = (i * no_vertices_ - (i * (i - 1) / 2));
        offset += (j - i);
      }
      else
      {
        if (j > 0)
          offset = (j * no_vertices_ - (j * (j - 1) / 2));
        offset += (i - j);
      }
    }
    else
      offset = i * no_vertices_ + j;

      return *(adjacency_matrix_ + offset);
  }

  /// Linear array size
  unsigned int AllocSize() const
  {
    return directed_graph ? (no_vertices_ * no_vertices_) : (no_vertices_ * (no_vertices_ + 1) / 2);
  }

  // - Members

  weight_element_type* adjacency_matrix_;
  unsigned int no_vertices_;
};

GRAPH_END

#endif // !_graph_base_h_