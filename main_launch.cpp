#include "main_launch.hpp"
#include "config.h"

#include <vector>
#include <bitset>
#include <set>
#include "src/GraphContainer.hpp"
#include "src/GraphAlgorithms.hpp"

int main(int argc, char* argv[])
{
  namespace ga = graph::algorithms;
  namespace g = graph;

  {
    using gc = g::GraphContainer<g::ADJACENCY_MATRIX>;
    using traits = ga::Traits<std::vector<int>, std::set<int> >;

    gc my_graph(8);

    my_graph.insertLink(0, 1, 1);     // symmetry is implicit for undirected graphs
    my_graph.insertLink(0, 2, 1);
    my_graph.insertLink(0, 3, 1);
    my_graph.insertLink(1, 4, 1);
    my_graph.insertLink(1, 5, 1);
    my_graph.insertLink(2, 5, 1);

    ga::Traversal<>::vertices_list crt_list;
    ga::Traversal<gc>::breadthFirst(my_graph, 0, crt_list);                         // 0 1 2 3 4 5

    crt_list.clear();
    ga::Traversal<gc>::depthFirst(my_graph, 0, crt_list);                           // 0 1 4 5 2 3

    ga::Traversal<gc, traits>::vertices_list crt_list2;
    ga::Traversal<gc, traits>::breadthFirst(my_graph, 0, crt_list2);                // 0 1 2 3 4 5

    auto links_no = my_graph.noEdges();

    ga::Features<gc>::vertex_lists conn_comps;
    auto nbConnComps = ga::Features<gc>::connectedComponents(my_graph, conn_comps); // 0 1 2 3 4 5; 6; 7
  }

  {
    using gc = g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED>;
    gc my_graph(4);
    my_graph.insertLink(0, 1, 5);
    my_graph.insertLink(1, 2, 3);
    my_graph.insertLink(2, 3, 1);
    my_graph.insertLink(0, 3, 10);

    using weight_el = gc::weight_element_type;

    graph::common::SquareMatrix<weight_el> mat_paths(4, 0);
    graph::common::SquareMatrix<int> mat_nxt(4, 0);
    ga::Paths<gc>::computePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = ga::Paths<gc>::extractPath(mat_nxt, 2, 0, seq);

    auto links_no = my_graph.noEdges();

    ga::Features<gc>::vertex_lists conn_comps;
    auto nbConnComps = ga::Features<gc>::connectedComponents(my_graph, conn_comps); // 0 1 3 2
  }

  {
    using gc = g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED>;
    g::GraphContainer<g::ADJACENCY_MATRIX, g::DIRECTED> my_graph(5);
    my_graph.insertLink(0, 1, 5);
    my_graph.insertLink(0, 3, 2);

    my_graph.insertLink(1, 2, 2);

    my_graph.insertLink(2, 0, 3);
    my_graph.insertLink(2, 4, 7);

    my_graph.insertLink(3, 2, 4);
    my_graph.insertLink(3, 4, 1);

    my_graph.insertLink(4, 0, 1);
    my_graph.insertLink(4, 1, 3);

    using weight_el = gc::weight_element_type;

    graph::common::SquareMatrix<weight_el> mat_paths(5, 0);
    graph::common::SquareMatrix<int> mat_nxt(5, 0);
    ga::Paths<gc>::computePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = ga::Paths<gc>::extractPath(mat_nxt, 0, 4, seq);

    auto links_no = my_graph.noEdges();

    ga::Features<gc>::vertex_lists conn_comps;
    auto nbConnComps = ga::Features<gc>::connectedComponents(my_graph, conn_comps); // 0 1 3 2 4
  }

  {
    using gc = g::GraphContainer<g::ADJACENCY_LIST, g::UNDIRECTED>;
    gc my_graph(9);

    my_graph.insertLink(0, 1, 3, true);
    my_graph.insertLink(0, 3, 2, true);
    my_graph.insertLink(0, 8, 4, true);

    my_graph.insertLink(1, 7, 4, true);

    my_graph.insertLink(2, 7, 2, true);
    my_graph.insertLink(2, 3, 6, true);
    my_graph.insertLink(2, 5, 1, true);

    my_graph.insertLink(3, 4, 1, true);

    my_graph.insertLink(4, 8, 8, true);

    my_graph.insertLink(5, 6, 8, true);

    using weight_el = gc::weight_element_type;

    graph::common::SquareMatrix<weight_el> mat_paths(9, 0);
    graph::common::SquareMatrix<int> mat_nxt(9, 0);
    ga::Paths<gc>::computePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = ga::Paths<gc>::extractPath(mat_nxt, 0, 2, seq);

    auto links_no = my_graph.noEdges();

    ga::Features<gc>::vertex_lists conn_comps;
    auto nbConnComps = ga::Features<gc>::connectedComponents(my_graph, conn_comps); // 0 1 3 8 7 2 4 5 6
  }

  return 0;
}