#ifndef _graph_base_h_
#define _graph_base_h_

#include <deque>
#include <list>
#include <utility>
#include <cassert>
#include "GraphCommon.hpp"

GRAPH_START

enum StoragePolicy : unsigned short { ADJACENCY_MATRIX = 0x00, ADJACENCY_LIST };
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

/// A Base structure for the graph specializations
template <StoragePolicy storage_method,
          LinkType link_type = UNDIRECTED,
          class distance_type = int,
          class eq_elems = EquivalentElements<distance_type> >
struct GraphBase
{
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
};


/// A graph container class meant for keeping the nodes of a graph and the associated edge costs(must be positive), no algorithms provided here
template <StoragePolicy storage_method,
          LinkType link_type = UNDIRECTED,
          class distance_type = int,
          class eq_elems = EquivalentElements<distance_type> >
class GraphContainer : public GraphBase<storage_method, link_type, distance_type, eq_elems>
{
};

/// A partial specialization menat to deal with sparse graphs
template <class distance_type,
          LinkType link_type,
          class eq_elems>
class GraphContainer<ADJACENCY_LIST, link_type, distance_type, eq_elems> : public GraphBase<ADJACENCY_LIST, link_type, distance_type, eq_elems>
{
public :

  typedef GraphBase<ADJACENCY_LIST, link_type, distance_type, eq_elems> base_type;

  /// Ctor
  GraphContainer(unsigned int no_vertices) : no_vertices_(no_vertices)
  {
    collection_node_edge_lists_ = new node_edge_list[no_vertices_];
  }

  /// Copy ctor
  GraphContainer(const GraphContainer& rhs_elem) : no_vertices_(rhs_elem.no_vertices_)
  {
    collection_node_edge_lists_ = new node_edge_list[no_vertices_];

    for (unsigned int i = 0; i < no_vertices_; ++i)
      *(collection_node_edge_lists_ + i) = *(rhs_elem.collection_node_edge_lists_+ i);
  }
  
#ifndef _MOVE_SEMANTICS_OFF
  /// Move ctor
  GraphContainer(GraphContainer&& rhs_elem) : no_vertices_(rhs_elem.no_vertices_)
  {
    if (rhs_elem.collection_node_edge_lists_)
    {
      collection_node_edge_lists_ = rhs_elem.collection_node_edge_lists_;
      rhs_elem.collection_node_edge_lists_ = nullptr;
    }
  }
#endif

  /// Copy assignment
  GraphContainer& operator = (const GraphContainer& rhs_elem)
  {
    if (this != &rhs_elem)
    {
      if (collection_node_edge_lists_)
        delete[] collection_node_edge_lists_;

      no_vertices_ = rhs_elem.no_vertices_;
      collection_node_edge_lists_ = new node_edge_list[no_vertices_];

      for (unsigned int i = 0; i < no_vertices_; ++i)
        *(collection_node_edge_lists_ + i) = *(rhs_elem.collection_node_edge_lists_ + i);
    }

    return *this;
  }

#ifndef _MOVE_SEMANTICS_OFF
  /// Move assignment
  GraphContainer& operator = (GraphContainer&& rhs_elem)
  {
    if (this != &rhs_elem)
    {
      if (collection_node_edge_lists_)
        delete[] collection_node_edge_lists_;

      no_vertices_ = rhs_elem.no_vertices_;
      collection_node_edge_lists_ = rhs_elem.collection_node_edge_lists_;
      rhs_elem.collection_node_edge_lists_ = nullptr;
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
    unsigned int no_links = 0;
    for (unsigned int i = 0; i < no_vertices_; ++i)
      no_links += static_cast<unsigned int>(collection_node_edge_lists_[i].size());

    if (!base_type::directed_graph)
      no_links /= 2;

    return no_links;
  }

  /// Get the maximum number of edges (refflexivity not included e.g. xRx)
  unsigned int MaxNoEdges() const
  {
    unsigned int max_nb_edge = no_vertices_ * (no_vertices_ - 1);  // digraph
    if (!base_type::directed_graph)                                           // graphs (undirected graphs)
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

    typename node_edge_list::const_iterator itCrt(collection_node_edge_lists_[i].begin()), itEnd(collection_node_edge_lists_[i].end());

    for (; itCrt != itEnd; ++itCrt)
    {
      if (itCrt->first == j)
        return true;
    }

    return false;
  }

  /// Get the weight from node i to node j
  distance_type GetWeight(int i, int j) const
  {
    distance_type crt_distance = base_type::undefined_weight;
    if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return crt_distance;
    }

    typename node_edge_list::const_iterator itCrt(collection_node_edge_lists_[i].begin()), itEnd(collection_node_edge_lists_[i].end());

    for (; itCrt != itEnd; ++itCrt)
    {
      if (itCrt->first == j)
        return static_cast<distance_type>(itCrt->second);
    }

    return crt_distance;
  }

  /// Check if the relation is symmetric relation iRj and jRi (not that the cost is the same)
  bool SymmetricRelation(int i, int j) const
  {
    if (i >= no_vertices_ || j >= no_vertices_ || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return false;
    }

    if (base_type::directed_graph)
    {
      if (HasLink(i, j) && HasLink(j, i))
        return true;
    }

    return false;
  }

  /// Get the list of links for the current node
  void GetLinks(int vertex, base_type::neighbors_list& vertex_list) const
  {
    if (vertex >= static_cast<int>(no_vertices_) || vertex < 0)
    {
      assert(false && "Invalid vertex specified");
      return;
    }

    for (const auto& elem : collection_node_edge_lists_[i])
      vertex_list.push_back(elem.first);
  }

  /// Get a list with all the links defined
  void GetAllLinks(std::deque<base_type::Edge>& links_list) const
  {
    links_list.clear();
    typename base_type::Edge crtEdge;

    for (unsigned int i = 0; i < no_vertices_; ++i)
    {
      crtEdge.start_ = i;
      for (const auto& elem : collection_node_edge_lists_[i])
      {
        crtEdge.end_       = elem->first;
        crtEdge.edge_cost_ = elem->second;
        links_list.push_back(crtEdge);
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

    collection_node_edge_lists_[i].push_back(list_node(j, weight));
    if ((base_type::directed_graph && symmetric) || !base_type::directed_graph)
      collection_node_edge_lists_[j].push_back(list_node(i, weight));
  }

  /// Remove an edge from vertex i to vertex j
  distance_type ClearLink(int i, int j)
  {
    if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return base_type::undefined_weight;
    }

    typename node_edge_list::const_iterator itCrt(collection_node_edge_lists_[i].begin()), itEnd(collection_node_edge_lists_[i].end());

    for (; itCrt != itEnd; ++itCrt)
    {
      if (itCrt->first == j)
      {
        distance_type prev_value = static_cast<distance_type>(itCrt->second);
        collection_node_edge_lists_[i].erase(itCrt);
        return prev_value;
      }
    }

    return base_type::undefined_weight;
  }

  /// Clear all edges
  void ClearAllLinks()
  {
    for (unsigned int i = 0; i < alloc_size; ++i)
      collection_node_edge_lists_[i].clear();
  }

protected :

  // - Members

  typedef std::pair<int, typename base_type::weight_element_type> list_node;
  typedef std::list<list_node> node_edge_list;
  
  node_edge_list* collection_node_edge_lists_;
  unsigned int no_vertices_;
};

/// A partial specialization meant to deal with dense graphs
template <class distance_type,
          LinkType link_type,
          class eq_elems>
class GraphContainer<ADJACENCY_MATRIX, link_type, distance_type, eq_elems> : public GraphBase<ADJACENCY_MATRIX, link_type, distance_type, eq_elems>
{
public :

  typedef GraphBase<ADJACENCY_MATRIX, link_type, distance_type, eq_elems> base_type;

  /// Ctor
  GraphContainer(unsigned int no_vertices) : no_vertices_(no_vertices)
  {
    unsigned int alloc_size = AllocSize();
    adjacency_matrix_ = new typename base_type::weight_element_type[alloc_size];
    for (unsigned int i = 0; i < alloc_size; ++i)
      *(adjacency_matrix_ + i) = base_type::undefined_weight;
  }

  /// Ctor (practically creates a complete graph with n(n-1) edges for digraphs or n(n-1)/2 edges for undirected graphs with the specified weight)
  GraphContainer(unsigned int no_vertices, const distance_type& initial_weight) : no_vertices_(no_vertices)
  {
    unsigned int alloc_size = AllocSize();
    adjacency_matrix_ = new typename base_type::weight_element_type[alloc_size];
    for (unsigned int i = 0; i < alloc_size; ++i)                // safe to put since the vertices don't generally have links to themselves
      *(adjacency_matrix_ + i) = initial_weight;
    for (unsigned int i = 0; i < no_vertices_; ++i)
      MatrixElem(i, i) = base_type::undefined_weight;
  }

  /// Copy ctor
  GraphContainer(const GraphContainer& rhs_elem) : no_vertices_(rhs_elem.no_vertices_)
  {
    unsigned int alloc_size = AllocSize();
    adjacency_matrix_ = new typename base_type::weight_element_type[alloc_size];
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

      adjacency_matrix_ = new typename base_type::weight_element_type[alloc_size];
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
    typename base_type::equivalence_cmp equiv;

    for (unsigned int i = 0; i < alloc_size; ++i)
    {
      if (!equiv(*(adjacency_matrix_ + i), static_cast<distance_type>(base_type::undefined_weight)))
        ++no_edges;
    }

    return no_edges;
  }

  /// Get the maximum number of edges (refflexivity not included e.g. xRx)
  unsigned int MaxNoEdges() const
  {
    unsigned int max_nb_edge = no_vertices_ * (no_vertices_ - 1);  // digraph
    if (!base_type::directed_graph)                                           // graphs (undirected graphs)
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

    typename base_type::equivalence_cmp equiv;

    if (equiv(MatrixElemCst(i, j), static_cast<distance_type>(base_type::undefined_weight)))
      return false;
    return true;
  }

  /// Get the weight from node i to node j
  distance_type GetWeight(int i, int j) const
  {
    if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return base_type::undefined_weight;
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

    if (base_type::directed_graph)
    {
      typename base_type::equivalence_cmp equiv;

      if (equiv(MatrixElemCst(i, j), base_type::undefined_weight) || equiv(MatrixElemCst(j, i), base_type::undefined_weight))
        return false;
    }

    return true;
  }

  /// Get the list of links for the current node
  void GetLinks(int vertex, base_type::neighbors_list& vertex_list) const
  {
    if (vertex >= static_cast<int>(no_vertices_) || vertex < 0)
    {
      assert(false && "Invalid vertex specified");
      return;
    }

    typename base_type::equivalence_cmp equiv;
    if (base_type::directed_graph)
    {
      unsigned int st_offset = vertex * no_vertices_;
      unsigned int end_offset = st_offset + no_vertices_;

      for (unsigned int i = st_offset; i < end_offset; ++i)
      {
        if (!equiv(adjacency_matrix_[i], base_type::undefined_weight))
          vertex_list.push_back(i % no_vertices_);
      }
    }
    else
    {
      for (unsigned int i = 0; i < no_vertices_; ++i)
      {
        if (!equiv(MatrixElemCst(vertex, i), base_type::undefined_weight))
          vertex_list.push_back(i);
      }
    }
    
  }

  /// Get a list with all the links defined
  void GetAllLinks(std::deque<base_type::Edge>& links_list) const
  {
    links_list.clear();
    typename base_type::Edge crtEdge;

    if (base_type::directed_graph)
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
    if (base_type::directed_graph && symmetric)
      MatrixElem(j, i) = weight;
  }

  /// Remove an edge from vertex i to vertex j
  distance_type ClearLink(int i, int j)
  {
    if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) || i < 0 || j < 0 || i == j)
    {
      assert(false && "Invalid vertices specified");
      return base_type::undefined_weight;
    }

    distance_type prev_value;
    prev_value = MatrixElem(i, j);
    MatrixElem(i, j) = base_type::undefined_weight;
    return prev_value;
  }

  /// Clear all edges
  void ClearAllLinks()
  {
    unsigned int alloc_size = AllocSize();
    for (unsigned int i = 0; i < alloc_size; ++i)
      *(adjacency_matrix_ + i) = base_type::undefined_weight;
  }
  
protected :

  // - Internals

  /// Get the corresponding matrix element
  base_type::weight_element_type& MatrixElem(int i, int j)
  {
    unsigned int offset = 0;
    if (!base_type::directed_graph)
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
  const base_type::weight_element_type& MatrixElemCst(int i, int j) const
  {
    unsigned int offset = 0;
    if (!base_type::directed_graph)
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
    return base_type::directed_graph ? (no_vertices_ * no_vertices_) : (no_vertices_ * (no_vertices_ + 1) / 2);
  }

  // - Members

  base_type::weight_element_type* adjacency_matrix_;
  unsigned int no_vertices_;
};

GRAPH_END

#endif // !_graph_base_h_