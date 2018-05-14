#ifndef _graph_base_h_
#define _graph_base_h_

#include <deque>
#include <utility>
#include <cassert>
#include "GraphCommon.hpp"

GRAPH_START

enum StoragePolicy : unsigned short { ADJACENCY_MATRIX = 0x00, ADJACENCY_LIST };
enum LinkType : unsigned short { DIRECTED, UNDIRECTED };

template
<class elem_type>
struct equivalent_elements
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

/// A graph container class meant for keeping the nodes of a graph, no algorithms provided here
template <StoragePolicy storage_method,
          LinkType link_type = UNDIRECTED,
          class distance_type = int,
          class eq_elems = equivalent_elements<distance_type> >
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
  typedef std::pair<std::size_t, distance_type> vertex_cost;
  typedef std::deque<std::size_t> neighbors_list;
  typedef eq_elems equivalence_cmp;

  enum { crt_storage_type = ADJACENCY_MATRIX };
  enum { directed_graph = IsDirectedGraph<link_type>::value };
  enum { undefined_weight = 0 };

  /// Ctor
  GraphContainer(std::size_t no_vertices) : no_vertices_(no_vertices)
  {
    std::size_t alloc_size = AllocSize();
    adjacency_matrix_ = new weight_element_type[alloc_size];
    for (std::size_t i = 0; i < alloc_size; ++i)
      *(adjacency_matrix_ + i) = undefined_weight;
  }

  /// Copy ctor
  GraphContainer(const GraphContainer& rhs_elem) : no_vertices_(rhs_elem.no_vertices_)
  {
    std::size_t alloc_size = AllocSize();
    adjacency_matrix_ = new weight_element_type[alloc_size];
    for (std::size_t i = 0; i < alloc_size; ++i)
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
      std::size_t alloc_size = AllocSize();

      adjacency_matrix_ = new weight_element_type[alloc_size];
      for (std::size_t i = 0; i < alloc_size; ++i)
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
  std::size_t NoVertices() const
  {
    return no_vertices_;
  }

  /// Has link between i and j
  bool HasLink(std::size_t i, std::size_t j) const
  {
    if (i >= no_vertices_ || j >= no_vertices_)
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
  distance_type GetWeight(std::size_t i, std::size_t j) const
  {
    if (i >= no_vertices_ || j >= no_vertices_)
    {
      assert(false && "Invalid vertices specified");
      return undefined_weight;
    }

    return static_cast<distance_type>(MatrixElem(i, j));
  }

  /// Check if the relation is symmetric relation iRj and jRi (not that the cost is the same)
  bool SymmetricRelation(std::size_t i, std::size_t j) const
  {
    if (i >= no_vertices_ || j >= no_vertices_)
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
  void GetLinks(std::size_t vertex, neighbors_list& vertex_list) const
  {
    if (vertex >= no_vertices_)
    {
      assert(false && "Invalid vertex specified");
      return;
    }


    equivalence_cmp equiv;
    if (directed_graph)
    {
      std::size_t st_offset = vertex * no_vertices_;
      std::size_t end_offset = st_offset + no_vertices_;

      for (std::size_t i = st_offset; i < end_offset; ++i)
      {
        if (!equiv(adjacency_matrix_[i], undefined_weight))
          vertex_list.push_back(i % no_vertices_);
      }
    }
    else
    {
      for (std::size_t i = 0; i < no_vertices_; ++i)
      {
        if (!equiv(MatrixElemCst(vertex, i), undefined_weight))
          vertex_list.push_back(i);
      }
    }
    
  }

  // - Mutators

  /// Set a node path from vertex i to vertex j (or both vertices if the relation is symmetric)
  void InsertLink(std::size_t i, std::size_t j, const distance_type& weight, bool symmetric = true)
  {
    if (i >= no_vertices_ || j >= no_vertices_)
    {
      assert(false && "Invalid vertices specified");
      return;
    }

    MatrixElem(i, j) = weight;
    if (directed_graph && symmetric)
      MatrixElem(j, i) = weight;
  }

  /// Remove a node path from vertex i to vertex j
  distance_type ClearLink(std::size_t i, std::size_t j)
  {
    if (i >= no_vertices_ || j >= no_vertices_)
    {
      assert(false && "Invalid vertices specified");
      return undefined_weight;
    }

    distance_type prev_value;
    prev_value = MatrixElem(i, j);
    MatrixElem(i, j) = undefined_weight;
    return prev_value;
  }

  /// Clear all links
  void ClearAllLinks()
  {
    std::size_t alloc_size = AllocSize();
    for (std::size_t i = 0; i < alloc_size; ++i)
      *(adjacency_matrix_ + i) = undefined_weight;
  }
  
protected :

  // - Internals

  /// Get the corresponding matrix element
  weight_element_type& MatrixElem(std::size_t i, std::size_t j)
  {
    std::size_t offset = 0;
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
  const weight_element_type& MatrixElemCst(std::size_t i, std::size_t j) const
  {
    std::size_t offset = 0;
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
  std::size_t AllocSize() const
  {
    return directed_graph ? (no_vertices_ * no_vertices_) : (no_vertices_ * (no_vertices_ + 1) / 2);
  }

  // - Members

  weight_element_type* adjacency_matrix_;
  std::size_t no_vertices_;
};

GRAPH_END

#endif // !_graph_base_h_