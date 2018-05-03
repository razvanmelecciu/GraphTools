// Graph.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "GraphCommon.hpp"
#include "GraphBase.hpp"
#include "GraphAlgorithms.hpp"


int main()
{
  graph::GraphContainer<graph::ADJACENCY_MATRIX> my_graph(8);

  my_graph.InsertLink(0, 1, 1);
  my_graph.InsertLink(0, 2, 1);
  my_graph.InsertLink(0, 3, 1);
  my_graph.InsertLink(0, 4, 1);
  my_graph.InsertLink(0, 5, 1);

  my_graph.InsertLink(1, 6, 1);
  my_graph.InsertLink(1, 7, 1);

  my_graph.HasLink(0, 1);

  graph::algorithms::Traversal<graph::ADJACENCY_MATRIX>::vertex_list crt_list;
  graph::algorithms::Traversal<graph::ADJACENCY_MATRIX>::BreadthFirst(my_graph, 0, crt_list);

  crt_list.clear();
  graph::algorithms::Traversal<graph::ADJACENCY_MATRIX>::DepthFirst(my_graph, 0, crt_list);

  return 0;
}

