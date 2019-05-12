// Graph.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <vector>
#include <bitset>
#include <set>
#include "src\GraphContainer.hpp"
#include "src\GraphAlgorithms.hpp"


int main(int argc, char* argv[])
{
  namespace ga = graph::algorithms;
  namespace g = graph;

  {
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

    ga::Traversal<g::GraphContainer<g::ADJACENCY_MATRIX>, ga::Traits<std::vector<int>, std::set<int> > >::vertices_list crt_list2;
    ga::Traversal<g::GraphContainer<g::ADJACENCY_MATRIX>, ga::Traits<std::vector<int>, std::set<int> > >::BreadthFirst(my_graph, 0, crt_list2);    // 0 1 2 3 4 5

    auto links_no = my_graph.NoEdges();
  }

  {
    g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> my_graph(4);
    my_graph.InsertLink(0, 1, 5);
    my_graph.InsertLink(1, 2, 3);
    my_graph.InsertLink(2, 3, 1);
    my_graph.InsertLink(0, 3, 10);

    typedef g::GraphContainer<g::ADJACENCY_MATRIX>::weight_element_type weight_el;

    graph::common::SquareMatrix<weight_el> mat_paths(4, 0);
    graph::common::SquareMatrix<int> mat_nxt(4, 0);
    ga::Paths<g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> >::ComputePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = ga::Paths<g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> >::ExtractPath(mat_nxt, 2, 0, seq);

    auto links_no = my_graph.NoEdges();
  }

  {
    g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> my_graph(5);
    my_graph.InsertLink(0, 1, 5);
    my_graph.InsertLink(0, 3, 2);

    my_graph.InsertLink(1, 2, 2);

    my_graph.InsertLink(2, 0, 3);
    my_graph.InsertLink(2, 4, 7);

    my_graph.InsertLink(3, 2, 4);
    my_graph.InsertLink(3, 4, 1);

    my_graph.InsertLink(4, 0, 1);
    my_graph.InsertLink(4, 1, 3);

    typedef g::GraphContainer<g::ADJACENCY_MATRIX>::weight_element_type weight_el;

    graph::common::SquareMatrix<weight_el> mat_paths(5, 0);
    graph::common::SquareMatrix<int> mat_nxt(5, 0);
    ga::Paths<g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> >::ComputePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = ga::Paths<g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> >::ExtractPath(mat_nxt, 0, 4, seq);

    auto links_no = my_graph.NoEdges();
  }

  {
    g::GraphContainer<g::ADJACENCY_LIST, g::UNDIRECTED> my_graph(9);

    my_graph.InsertLink(0, 1, 3, true);
    my_graph.InsertLink(0, 3, 2, true);
    my_graph.InsertLink(0, 8, 4, true);

    my_graph.InsertLink(1, 7, 4, true);

    my_graph.InsertLink(2, 7, 2, true);
    my_graph.InsertLink(2, 3, 6, true);
    my_graph.InsertLink(2, 5, 1, true);

    my_graph.InsertLink(3, 4, 1, true);

    my_graph.InsertLink(4, 8, 8, true);

    my_graph.InsertLink(5, 6, 8, true);

    typedef g::GraphContainer<g::ADJACENCY_LIST>::weight_element_type weight_el;

    graph::common::SquareMatrix<weight_el> mat_paths(9, 0);
    graph::common::SquareMatrix<int> mat_nxt(9, 0);
    ga::Paths<g::GraphContainer<g::ADJACENCY_LIST, g::UNDIRECTED> >::ComputePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = ga::Paths<g::GraphContainer<g::ADJACENCY_LIST, g::UNDIRECTED> >::ExtractPath(mat_nxt, 0, 2, seq);

    auto links_no = my_graph.NoEdges();
  }

  return 0;
}

