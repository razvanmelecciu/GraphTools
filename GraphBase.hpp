#ifndef _graph_base_h_
#define _graph_base_h_

#include <queue>
#include <utility>
#include <cassert>
#include "GraphCommon.hpp"

GRAPH_START

enum StorageMethod : unsigned short { ADJACENCY_MATRIX = 0x00, ADJACENCY_LIST };

template
<class elem_type>
struct equivalent_elements
{
  bool operator () (elem_type a, elem_type b)
  {
    return (a == b);
  }
};

/// A graph container class meant for keeping the nodes of a graph, no algorithms provided here
template <StorageMethod storage_method, 
          class distance_type = int,
          class eq_elems = equivalent_elements<distance_type> >
class GraphContainer
{
};

/// A partial specialization meant to deal with dense graphs
template <class distance_type,
          class eq_elems>
class GraphContainer<ADJACENCY_MATRIX, distance_type, eq_elems>
{
public :

  typedef distance_type weight_element_type;
  typedef std::pair<std::size_t, distance_type> vertex_cost;
  typedef std::queue<std::size_t> neighbors_list;
  typedef eq_elems equivalence_cmp;

  enum { crt_storage_type = ADJACENCY_MATRIX };
  enum { undefined_weight = 0 };

  /// Ctor
  GraphContainer(std::size_t no_vertices) : no_vertices_(no_vertices), adjacency_matrix_(new weight_element_type[no_vertices * no_vertices])
  {
    std::size_t alloc_size = AllocSize();
    for (std::size_t i = 0; i < alloc_size; ++i)
      *(adjacency_matrix_ + i) = undefined_weight;
  }

  /// Copy ctor
  GraphContainer(const GraphContainer& rhs_elem) : no_vertices_(rhs_elem.no_vertices_), 
                                                   adjacency_matrix_(new weight_element_type[rhs_elem.no_vertices_ * rhs_elem.no_vertices_])
  {
    std::size_t alloc_size = AllocSize();
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

  /// Check if the relation is symmetric relation iRj and jRi
  bool SymmetricRelation(std::size_t i, std::size_t j) const
  {
    if (i >= no_vertices_ || j >= no_vertices_)
    {
      assert(false && "Invalid vertices specified");
      return false;
    }

    equivalence_cmp equiv;

    if (equiv(MatrixElemCst(i, j), undefined_weight) || equiv(MatrixElemCst(i, j), undefined_weight))
      return false;

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

    std::size_t st_offset  = vertex * no_vertices_;
    std::size_t end_offset = st_offset + no_vertices_;
    equivalence_cmp equiv;

    for (std::size_t i = st_offset; i < end_offset; ++i)
    {
      if (!equiv(adjacency_matrix_[i], undefined_weight))
        vertex_list.push(i % no_vertices_);
    }
  }

  // - Mutators

  /// Set a node path from vertex i to vertex j
  distance_type InsertLink(std::size_t i, std::size_t j, const distance_type& weight)
  {
    if (i >= no_vertices_ || j >= no_vertices_)
    {
      assert(false && "Invalid vertices specified");
      return undefined_weight;
    }

    distance_type prev_value;
    prev_value = MatrixElem(i, j);
    MatrixElem(i, j) = weight;
    return prev_value;
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
    for (std::size_t i = 0; i < no_vertices_; ++i)
      *(adjacency_matrix_ + i) = undefined_weight;
  }
  
protected :

  // - Internals

  /// Get the corresponding matrix element
  weight_element_type& MatrixElem(std::size_t i, std::size_t j)
  {
    std::size_t offset = i * no_vertices_ + j;
    return *(adjacency_matrix_ + offset);
  }

  /// Get the corresponding matrix element
  const weight_element_type& MatrixElemCst(std::size_t i, std::size_t j) const
  {
    std::size_t offset = i * no_vertices_ + j;
    return *(adjacency_matrix_ + offset);
  }

  /// Linear array size
  std::size_t AllocSize() const
  {
    return no_vertices_ * no_vertices_;
  }

  // - Members

  weight_element_type* adjacency_matrix_;
  std::size_t no_vertices_;
};

GRAPH_END

#endif // !_graph_base_h_