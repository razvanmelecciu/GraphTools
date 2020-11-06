#ifndef _graph_base_h_
#define _graph_base_h_

#include <deque>
#include <list>
#include <utility>
#include <cassert>
#include "GraphCommon.hpp"

GRAPH_START

/**
 * @brief Used for deciding how to store a graph
 */
enum StoragePolicy : unsigned short { ADJACENCY_MATRIX = 0x00, ADJACENCY_LIST };
/**
 * @brief Used for defining either a digraph or a graph
 */
enum LinkType : unsigned short { DIRECTED, UNDIRECTED };

/**
 * @brief The default cost comparison functor
 * @tparam elem_type type of the cost
 */
template
<class elem_type>
struct EquivalentCost
{
  bool operator () (elem_type a, elem_type b)
  {
    return (a == b);
  }
};

/**
 * @brief The default cost comparison functor for determining if a cost is less than another
 * @tparam elem_type type of the cost
 */
template
<class elem_type>
struct SmallerCost
{
  bool operator () (elem_type a, elem_type b)
  {
    return (a < b);
  }
};

/**
 * @brief Used for determining at compile time if a graph is directed
 * @tparam link_type DIRECTED / UNDIRECTED
 */
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

/**
 * @brief A Base structure for the graph specializations
 * @tparam storage_method either ADJACENCY_MATRIX or ADJACENCY_LIST
 * @tparam link_type DIRECTED (digraph) or UNDIRECTED (graph)
 * @tparam cost_type type for the cost between nodes
 * @tparam eq_cost functor for checking if 2 costs are the same
 * @tparam smaller_cost functor for checking if 2 costs are the same
 */
template <StoragePolicy storage_method,
          LinkType link_type = UNDIRECTED,
          typename cost_type = int,
          typename eq_cost = EquivalentCost<cost_type>,
          typename smaller_cost = SmallerCost<cost_type> >
struct GraphBase
{
  using cost_element_type = cost_type;
  using vertex_cost = std::pair<int, cost_type>;
  using neighbors_list = std::deque<int> ;
  using equivalent_cost_cmp = eq_cost;
  using smaller_cost_cmp = smaller_cost;
  using index_elem_type = int;

  enum { crt_storage_type = ADJACENCY_MATRIX };
  enum { directed_graph = IsDirectedGraph<link_type>::value };
  enum { undefined_cost = 0 };

  /**
   * @brief Edge struct
   */
  struct Edge
  {
     /**
     * @brief Default ctor
     */
    Edge() :
       start_(0),
       end_(0),
       edge_cost_(0)
    {
    }

    /**
     * @brief Edge ctor
     * @param[in] i starting node
     * @param[in] j ending node
     * @param[in] cost associated cost from i to j
     */
    Edge(int i, int j, cost_type cost) :
       start_(i),
       end_(j),
       edge_cost_(cost)
    {
    }

    bool operator < (const Edge& RHS) const
    {
       smaller_cost cmp_fn_less;
       return cmp_fn_less(edge_cost_, RHS.edge_cost_);
    }

    bool operator == (const Edge& RHS) const
    {
       eq_cost cmp_fn_eq;
       return cmp_fn_eq(edge_cost_, RHS.edge_cost_);
    }

    bool operator > (const Edge& RHS) const
    {
       smaller_cost cmp_fn_less;
       eq_cost cmp_fn_eq;
       return !(cmp_fn_less(edge_cost_, RHS.edge_cost_) || cmp_fn_eq(edge_cost_, RHS.edge_cost_));
    }

    int start_;                  ///< start vertex
    int end_;                    ///< end vertex
    cost_type edge_cost_;    ///< cost associated from i to j
  };
};

GRAPH_END

#endif // !_graph_base_h_
