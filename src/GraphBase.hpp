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

GRAPH_END

#endif // !_graph_base_h_