// Graph.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>
#include <bitset>
#include <set>
#include "src\GraphCommon.hpp"
#include "src\GraphBase.hpp"
#include "src\GraphAlgorithms.hpp"


int main()
{
  namespace ga = graph::algorithms;
  namespace g = graph;

  g::GraphContainer<g::ADJACENCY_MATRIX> my_graph(8);

  my_graph.InsertLink(0, 1, 1);     // symmetry is implicit for undirected graphs
  my_graph.InsertLink(0, 2, 1);
  my_graph.InsertLink(0, 3, 1);
  my_graph.InsertLink(1, 4, 1);
  my_graph.InsertLink(1, 5, 1);
  my_graph.InsertLink(2, 5, 1);

  ga::Traversal<>::vertices_list crt_list;
  ga::Traversal<g::GraphContainer<g::ADJACENCY_MATRIX> >::BreadthFirst(my_graph, 0, crt_list);    // 0 1 2 3 4 5

  crt_list.clear();
  ga::Traversal<g::GraphContainer<g::ADJACENCY_MATRIX> >::DepthFirst(my_graph, 0, crt_list);      // 0 1 4 5 2 3

  ga::Traversal<g::GraphContainer<g::ADJACENCY_MATRIX>, ga::Traits<std::vector<std::size_t>, std::set<std::size_t> > >::vertices_list crt_list2;
  ga::Traversal<g::GraphContainer<g::ADJACENCY_MATRIX>, ga::Traits<std::vector<std::size_t>, std::set<std::size_t> > >::BreadthFirst(my_graph, 0, crt_list2);    // 0 1 2 3 4 5

  g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> my_graph_2(4);
  my_graph_2.InsertLink(0, 1, 5);
  my_graph_2.InsertLink(1, 2, 3);
  my_graph_2.InsertLink(2, 3, 1);
  my_graph_2.InsertLink(0, 3, 10);

  typedef g::GraphContainer<g::ADJACENCY_MATRIX>::weight_element_type weight_el;
  graph::common::SquareMatrix<weight_el> mat_paths(4, 0);
  ga::Paths<g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> >::ComputePaths(my_graph_2, mat_paths);

  typedef ga::Paths<g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> >::dist_nxt d_nxt;
  graph::common::SquareMatrix<d_nxt> mat_paths_tr(4, d_nxt(0, 0));
  ga::Paths<g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> >::ComputePathsTrails(my_graph_2, mat_paths_tr);

  return 0;
}

