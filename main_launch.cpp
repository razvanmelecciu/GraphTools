#include "main_launch.hpp"
#include "config.h"

#include <vector>
#include <bitset>
#include <set>
#include "src/GraphContainer.hpp"
#include "src/GraphAlgorithms.hpp"

int main(int argc, char* argv[])
{
  namespace algo = graph::algorithms;

  {
    using graph::ADJACENCY_MATRIX;
    using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;
    using traits = algo::Traits<std::vector<int>, 
                                std::set<int>, 
                                std::vector<std::vector<int> > >;

    graph_container my_graph(8);

    my_graph.addLink(0, 1, 1);     // symmetry is implicit for undirected graphs
    my_graph.addLink(0, 2, 1);
    my_graph.addLink(0, 3, 1);
    my_graph.addLink(1, 4, 1);
    my_graph.addLink(1, 5, 1);
    my_graph.addLink(2, 5, 1);

    algo::Traversal<>::vertices_list crt_list;
    algo::Traversal<graph_container>::breadthFirst(my_graph, 0, crt_list);                         // 0 1 2 3 4 5

    crt_list.clear();
    algo::Traversal<graph_container>::depthFirst(my_graph, 0, crt_list);                           // 0 1 4 5 2 3

    algo::Traversal<graph_container, traits>::vertices_list crt_list2;
    algo::Traversal<graph_container, traits>::breadthFirst(my_graph, 0, crt_list2);                // 0 1 2 3 4 5

    auto links_no = my_graph.noEdges();

    algo::Features<graph_container>::vertices_list_categ conn_comps;
    auto nbConnComps = algo::Features<graph_container>::connectedComponents(my_graph, conn_comps); // 0 1 2 3 4 5; 6; 7
  }

  {
    using graph::ADJACENCY_MATRIX;
    using graph::DIRECTED;
    using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
    graph_container my_graph(4);
    my_graph.addLink(0, 1, 5);
    my_graph.addLink(1, 2, 3);
    my_graph.addLink(2, 3, 1);
    my_graph.addLink(0, 3, 10);

    using cost_el = graph_container::cost_element_type;

    graph::common::SquareMatrix<cost_el> mat_paths(4, 0);
    graph::common::SquareMatrix<int> mat_nxt(4, 0);
    algo::Paths<graph_container>::computePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = algo::Paths<graph_container>::extractPath(mat_nxt, 2, 0, seq);

    auto links_no = my_graph.noEdges();

    algo::Features<graph_container>::vertices_list_categ conn_comps;
    auto nbConnComps = algo::Features<graph_container>::connectedComponents(my_graph, conn_comps); // 0 1 3 2
  }

  {
    using graph::ADJACENCY_MATRIX;
    using graph::DIRECTED;
    using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
    graph_container my_graph(5);
    my_graph.addLink(0, 1, 5);
    my_graph.addLink(0, 3, 2);

    my_graph.addLink(1, 2, 2);

    my_graph.addLink(2, 0, 3);
    my_graph.addLink(2, 4, 7);

    my_graph.addLink(3, 2, 4);
    my_graph.addLink(3, 4, 1);

    my_graph.addLink(4, 0, 1);
    my_graph.addLink(4, 1, 3);

    using cost_el = graph_container::cost_element_type;

    graph::common::SquareMatrix<cost_el> mat_paths(5, 0);
    graph::common::SquareMatrix<int> mat_nxt(5, 0);
    algo::Paths<graph_container>::computePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = algo::Paths<graph_container>::extractPath(mat_nxt, 0, 4, seq);

    auto links_no = my_graph.noEdges();

    algo::Features<graph_container>::vertices_list_categ conn_comps;
    auto nbConnComps = algo::Features<graph_container>::connectedComponents(my_graph, conn_comps); // 0 1 3 2 4
  }

  {
    using graph::ADJACENCY_MATRIX;
    using graph::UNDIRECTED;
    using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
    graph_container my_graph(9);

    my_graph.addLink(0, 1, 3, true);
    my_graph.addLink(0, 3, 2, true);
    my_graph.addLink(0, 8, 4, true);

    my_graph.addLink(1, 7, 4, true);

    my_graph.addLink(2, 7, 2, true);
    my_graph.addLink(2, 3, 6, true);
    my_graph.addLink(2, 5, 1, true);

    my_graph.addLink(3, 4, 1, true);

    my_graph.addLink(4, 8, 8, true);

    my_graph.addLink(5, 6, 8, true);

    using cost_el = graph_container::cost_element_type;

    graph::common::SquareMatrix<cost_el> mat_paths(9, 0);
    graph::common::SquareMatrix<int> mat_nxt(9, 0);
    algo::Paths<graph_container>::computePathsTrails(my_graph, mat_paths, mat_nxt);

    std::deque<int> seq;
    bool path_found = algo::Paths<graph_container>::extractPath(mat_nxt, 0, 2, seq);

    auto links_no = my_graph.noEdges();

    algo::Features<graph_container>::vertices_list_categ conn_comps;
    auto nbConnComps = algo::Features<graph_container>::connectedComponents(my_graph, conn_comps); // 0 1 3 8 7 2 4 5 6
  }

  return 0;
}
