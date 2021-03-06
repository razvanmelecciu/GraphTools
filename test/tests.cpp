#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <vector>
#include <bitset>
#include <set>
#include "GraphContainer.hpp"
#include "GraphAlgorithms.hpp"

using namespace ::testing;

TEST(undirected_graphs, test_bf_default_traits_1)
{
  namespace algo = graph::algorithms;
  using graph_container = graph::dense_graph;

  graph_container my_graph(8);

  my_graph.addLink(0, 1, 1);  // symmetry is implicit for undirected graphs
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(0, 3, 1);
  my_graph.addLink(1, 4, 1);
  my_graph.addLink(1, 5, 1);
  my_graph.addLink(2, 5, 1);

  algo::Traversal<>::vertices_list crt_list;
  algo::Traversal<graph_container>::breadthFirst(my_graph, 0, crt_list);  // 0 1 2 3 4 5

  EXPECT_EQ(6, crt_list.size());
  EXPECT_THAT(crt_list, ElementsAre(0, 1, 2, 3, 4, 5));
}

TEST(undirected_graphs, test_bf_default_traits_2)
{
  namespace algo = graph::algorithms;
  using graph_container = graph::sparse_graph;

  graph_container my_graph(8);

  my_graph.addLink(0, 1, 1);  // symmetry is implicit for undirected graphs
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(0, 3, 1);
  my_graph.addLink(1, 4, 1);
  my_graph.addLink(1, 5, 1);
  my_graph.addLink(2, 5, 1);

  algo::Traversal<>::vertices_list crt_list;
  algo::Traversal<graph_container>::breadthFirst(my_graph, 0, crt_list);  // 0 1 2 3 4 5

  EXPECT_EQ(6, crt_list.size());
  EXPECT_THAT(crt_list, ElementsAre(0, 1, 2, 3, 4, 5));
}

TEST(undirected_graphs, test_bf_user_traits)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;
  using traits = algo::Traits<std::vector<int>, std::set<int>, std::vector<std::vector<int>>>;

  graph_container my_graph(8);

  my_graph.addLink(0, 1, 1);  // symmetry is implicit for undirected graphs
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(0, 3, 1);
  my_graph.addLink(1, 4, 1);
  my_graph.addLink(1, 5, 1);
  my_graph.addLink(2, 5, 1);

  algo::Traversal<graph_container, traits>::vertices_list crt_list;
  algo::Traversal<graph_container, traits>::breadthFirst(my_graph, 0, crt_list);  // 0 1 2 3 4 5

  EXPECT_EQ(6, crt_list.size());
  EXPECT_THAT(crt_list, ElementsAre(0, 1, 2, 3, 4, 5));
}

TEST(undirected_graphs, test_df_default_traits)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(8);

  my_graph.addLink(0, 1, 1);  // symmetry is implicit for undirected graphs
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(0, 3, 1);
  my_graph.addLink(1, 4, 1);
  my_graph.addLink(1, 5, 1);
  my_graph.addLink(2, 5, 1);

  algo::Traversal<>::vertices_list crt_list;
  algo::Traversal<graph_container>::depthFirst(my_graph, 0, crt_list);  // 0 1 4 5 2 3

  EXPECT_EQ(6, crt_list.size());
  EXPECT_THAT(crt_list, ElementsAre(0, 1, 4, 5, 2, 3));
}

TEST(undirected_graphs, test_df_user_traits)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;
  using traits = algo::Traits<std::vector<int>, std::set<int>, std::vector<std::vector<int>>>;

  graph_container my_graph(8);

  my_graph.addLink(0, 1, 1);  // symmetry is implicit for undirected graphs
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(0, 3, 1);
  my_graph.addLink(1, 4, 1);
  my_graph.addLink(1, 5, 1);
  my_graph.addLink(2, 5, 1);

  algo::Traversal<graph_container, traits>::vertices_list crt_list;
  algo::Traversal<graph_container, traits>::depthFirst(my_graph, 0, crt_list);  // 0 1 4 5 2 3

  EXPECT_EQ(6, crt_list.size());
  EXPECT_THAT(crt_list, ElementsAre(0, 1, 4, 5, 2, 3));
}

TEST(undirected_graphs, test_conn_comps_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;
  using traits = algo::Traits<std::vector<int>, std::set<int>, std::vector<std::vector<int>>>;

  graph_container my_graph(8);

  my_graph.addLink(0, 1, 1);  // symmetry is implicit for undirected graphs
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(0, 3, 1);
  my_graph.addLink(1, 4, 1);
  my_graph.addLink(1, 5, 1);
  my_graph.addLink(2, 5, 1);

  algo::Features<graph_container>::vertices_list_categ conn_comps;
  auto nbConnComps = algo::Features<graph_container>::connectedComponents(my_graph, conn_comps);  // 0 1 2 3 4 5; 6; 7

  algo::Features<graph_container>::vertices_list_categ conn_comps_expected = {{0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {6, 2}, {7, 3}};

  EXPECT_EQ(3, nbConnComps);
  EXPECT_THAT(conn_comps, ContainerEq(conn_comps_expected));
}

TEST(undirected_graphs, test_conn_comps_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

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

  algo::Features<graph_container>::vertices_list_categ conn_comps;
  auto nbConnComps = algo::Features<graph_container>::connectedComponents(my_graph, conn_comps);  // 0 1 3 8 7 2 4 5 6

  algo::Features<graph_container>::vertices_list_categ conn_comps_expected = {{0, 1}, {1, 1}, {3, 1}, {8, 1},
                                                                       {7, 1}, {2, 1}, {4, 1}, {5, 1}, {6, 1}};

  EXPECT_EQ(1, nbConnComps);
  EXPECT_THAT(conn_comps, ContainerEq(conn_comps_expected));
}

TEST(undirected_graphs, test_connected_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 3, true);
  my_graph.addLink(1, 2, 2, true);
  // vertex 3 is isolated

  EXPECT_FALSE(algo::Features<graph_container>::isConnected(my_graph));
}

TEST(undirected_graphs, test_connected_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 3, true);
  my_graph.addLink(1, 2, 2, true);
  my_graph.addLink(0, 3, 2, true);

  EXPECT_TRUE(algo::Features<graph_container>::isConnected(my_graph));
}

TEST(undirected_graphs, test_vertex_degree)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

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

  EXPECT_EQ(3, algo::Features<graph_container>::computeDegree(my_graph, 0));
  EXPECT_EQ(2, algo::Features<graph_container>::computeDegree(my_graph, 1));
  EXPECT_EQ(3, algo::Features<graph_container>::computeDegree(my_graph, 2));
  EXPECT_EQ(3, algo::Features<graph_container>::computeDegree(my_graph, 3));
  EXPECT_EQ(2, algo::Features<graph_container>::computeDegree(my_graph, 4));
  EXPECT_EQ(2, algo::Features<graph_container>::computeDegree(my_graph, 5));
  EXPECT_EQ(1, algo::Features<graph_container>::computeDegree(my_graph, 6));
  EXPECT_EQ(2, algo::Features<graph_container>::computeDegree(my_graph, 7));
  EXPECT_EQ(2, algo::Features<graph_container>::computeDegree(my_graph, 8));
}

TEST(undirected_graphs, test_compl_graph_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

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

  EXPECT_FALSE(algo::Features<graph_container>::completeGraph(my_graph));
}

TEST(undirected_graphs, test_compl_graph_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 3, true);
  my_graph.addLink(0, 2, 2, true);
  my_graph.addLink(0, 3, 4, true);

  my_graph.addLink(1, 2, 2, true);
  my_graph.addLink(1, 3, 4, true);

  my_graph.addLink(2, 3, 1, true);

  EXPECT_TRUE(algo::Features<graph_container>::completeGraph(my_graph));
}

TEST(undirected_graphs, test_compute_all_min_paths)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

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

  graph::common::SquareMatrix<cost_el> mat_paths(9, 0);
  graph::common::SquareMatrix<int> mat_nxt(9, 0);
  algo::Paths<graph_container>::computePathsTrails(my_graph, mat_paths, mat_nxt);

  algo::Paths<graph_container>::vertices_list seq;
  bool path_found_v0_v4 = algo::Paths<graph_container>::extractPath(mat_nxt, 0, 4, seq);
  EXPECT_TRUE(path_found_v0_v4);
  EXPECT_THAT(seq, ElementsAre(0, 3, 4));

  seq.clear();
  bool path_found_v1_v6 = algo::Paths<graph_container>::extractPath(mat_nxt, 1, 6, seq);
  EXPECT_TRUE(path_found_v1_v6);
  EXPECT_THAT(seq, ElementsAre(1, 7, 2, 5, 6));
}

TEST(undirected_graphs, test_transitive_closure_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

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

  graph::common::SquareMatrix<bool> closure(0, 0);
  algo::Features<graph_container>::transitiveClosure(my_graph, closure);
  EXPECT_EQ(9, closure.getSize());
  for (std::size_t i = 0; i < 9; ++i)
  {
    for (std::size_t j = 0; j < 9; ++j)
    {
      EXPECT_TRUE(closure(i, j));
    }
  }
}

TEST(undirected_graphs, test_transitive_closure_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::UNDIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(9);

  my_graph.addLink(0, 1, 3, true);
  my_graph.addLink(0, 3, 2, true);
  my_graph.addLink(0, 8, 4, true);

  my_graph.addLink(1, 7, 4, true);

  my_graph.addLink(2, 7, 2, true);
  my_graph.addLink(2, 3, 6, true);

  my_graph.addLink(3, 4, 1, true);

  my_graph.addLink(4, 8, 8, true);

  my_graph.addLink(5, 6, 8, true);

  graph::common::SquareMatrix<bool> closure(0, 0);
  algo::Features<graph_container>::transitiveClosure(my_graph, closure);
  EXPECT_EQ(9, closure.getSize());
  for (std::size_t i = 0; i < 9; ++i)
  {
    for (std::size_t j = 0; j < 9; ++j)
    {
      if (i != j && (j == 5 || j == 6 || i == 5 || i == 6))
      {
        if ((i == 5 && j == 6) || (i == 6 && j == 5))
        {
          EXPECT_TRUE(closure(i, j));
        }
        else
        {
          EXPECT_FALSE(closure(i, j));
        }
      }
      else
      {
        EXPECT_TRUE(closure(i, j));
      }
    }
  }
}

TEST(undirected_graphs, test_cycle_present_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 0, 1);

  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 0));
}

TEST(undirected_graphs, test_cycle_present_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);

  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 0));
}

TEST(undirected_graphs, test_cycle_present_3)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 0, 1);
  my_graph.addLink(2, 3, 1);

  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 0));
}

TEST(undirected_graphs, test_cycle_present_4)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(7);

  // connected comp 1 has a cycle
  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 0, 1);
  my_graph.addLink(2, 3, 1);

  // connected comp 2 has no cycles
  my_graph.addLink(4, 5, 1);
  my_graph.addLink(5, 6, 1);

  for (auto initial_vertex = 0; initial_vertex < 4; ++initial_vertex)
  {
    EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, initial_vertex));
  }
  for (auto initial_vertex = 4; initial_vertex < 7; ++initial_vertex)
  {
    EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, initial_vertex));
  }
}

TEST(undirected_graphs, test_cycle_present_5)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(7);

  // connected comp 1 has no cycles
  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);

  // connected comp 2 has a cycles
  my_graph.addLink(4, 5, 1);
  my_graph.addLink(5, 6, 1);
  my_graph.addLink(4, 6, 1);

  for (auto initial_vertex = 0; initial_vertex < 4; ++initial_vertex)
  {
    EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, initial_vertex));
  }
  for (auto initial_vertex = 4; initial_vertex < 7; ++initial_vertex)
  {
    EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, initial_vertex));
  }
}

TEST(undirected_graphs, test_cycle_present_6)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(7);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(4, 5, 1);
  my_graph.addLink(5, 6, 1);
  my_graph.addLink(4, 0, 1);

  for (auto initial_vertex = 0; initial_vertex < 7; ++initial_vertex)
  {
    EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, initial_vertex));
  }
}

TEST(undirected_graphs, test_cycle_present_7)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(7);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(4, 5, 1);
  my_graph.addLink(5, 6, 1);
  my_graph.addLink(4, 0, 1);
  my_graph.addLink(0, 6, 1);

  for (auto initial_vertex = 0; initial_vertex < 7; ++initial_vertex)
  {
    EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, initial_vertex));
  }
}

TEST(undirected_graphs, test_bipartite_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 0, 1);

  algo::Features<graph_container>::vertices_list_categ vertex_list;
  algo::Features<graph_container>::vertices_list_categ vertex_list_expected = {{0, 0}, {1, 1}, {3, 1}, {2, 0}};
  auto isBipartite = algo::Features<graph_container>::isBipartite(my_graph, vertex_list);  // 0 1 3 2

  EXPECT_TRUE(isBipartite);
  EXPECT_THAT(vertex_list, ContainerEq(vertex_list_expected));
}

TEST(undirected_graphs, test_bipartite_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(5);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 4, 1);
  my_graph.addLink(4, 0, 1);

  algo::Features<graph_container>::vertices_list_categ vertex_list;
  algo::Features<graph_container>::vertices_list_categ vertex_list_expected = {{0, 0}, {1, 1}, {4, 1}, {2, 0}, {3, 0}};
  auto isBipartite = algo::Features<graph_container>::isBipartite(my_graph, vertex_list);  // 0 1 4 2 3

  EXPECT_FALSE(isBipartite);
  EXPECT_THAT(vertex_list, ContainerEq(vertex_list_expected));
}

TEST(undirected_graphs, test_bipartite_3)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(5);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 0, 1);
  // node 4 is isolated

  algo::Features<graph_container>::vertices_list_categ vertex_list;
  algo::Features<graph_container>::vertices_list_categ vertex_list_expected = {{0, 0}, {1, 1}, {3, 1}, {2, 0}, {4, 0}};
  auto isBipartite = algo::Features<graph_container>::isBipartite(my_graph, vertex_list);  // 0 1 3 2 4

  EXPECT_TRUE(isBipartite);
  EXPECT_THAT(vertex_list, ContainerEq(vertex_list_expected));
}

TEST(undirected_graphs, test_bipartite_4)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX>;

  graph_container my_graph(8);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 0, 1);

  my_graph.addLink(4, 5, 1);
  my_graph.addLink(5, 6, 1);
  my_graph.addLink(6, 7, 1);
  my_graph.addLink(7, 4, 1);

  algo::Features<graph_container>::vertices_list_categ vertex_list;
  algo::Features<graph_container>::vertices_list_categ vertex_list_expected = {{0, 0}, {1, 1}, {3, 1}, {2, 0},
                                                                               {4, 0}, {5, 1}, {7, 1}, {6, 0}};
  auto isBipartite = algo::Features<graph_container>::isBipartite(my_graph, vertex_list);  // 0 1 3 2 4 5 7 6

  EXPECT_TRUE(isBipartite);
  EXPECT_THAT(vertex_list, ContainerEq(vertex_list_expected));
}

TEST(undirected_graphs, test_bipartite_5)
{
  namespace algo = graph::algorithms;
  using graph_container = graph::sparse_graph;

  graph_container my_graph(7);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 0, 1);

  my_graph.addLink(4, 5, 1);
  my_graph.addLink(5, 6, 1);
  my_graph.addLink(6, 4, 1);

  algo::Features<graph_container>::vertices_list_categ vertex_list;
  algo::Features<graph_container>::vertices_list_categ vertex_list_expected = {{0, 0}, {1, 1}, {3, 1}, {2, 0},
                                                                               {4, 0}, {5, 1}, {6, 1}};
  auto isBipartite = algo::Features<graph_container>::isBipartite(my_graph, vertex_list);  // 0 1 3 2 4 5 6

  EXPECT_FALSE(isBipartite);
  EXPECT_THAT(vertex_list, ContainerEq(vertex_list_expected));
}

TEST(undirected_graphs, test_mspk_1)
{
  namespace algo = graph::algorithms;
  using graph_container = graph::sparse_graph;

  graph_container my_graph(9), out_msp(9);

  my_graph.addLink(0, 1, 4);
  my_graph.addLink(0, 7, 8);
  my_graph.addLink(1, 2, 8);
  my_graph.addLink(1, 7, 11);
  my_graph.addLink(2, 3, 7);
  my_graph.addLink(2, 5, 4);
  my_graph.addLink(2, 8, 2);
  my_graph.addLink(3, 4, 9);
  my_graph.addLink(3, 5, 14);
  my_graph.addLink(4, 5, 10);
  my_graph.addLink(5, 6, 2);
  my_graph.addLink(6, 7, 1);
  my_graph.addLink(6, 8, 6);
  my_graph.addLink(7, 8, 7);

  auto msp_total_cost = algo::MSP<graph_container>::kruskal(my_graph, out_msp);

  std::deque<graph_container::Edge> edges_list;
  out_msp.getAllEdges(edges_list);

  std::deque<graph_container::Edge> edges_list_expected;
  edges_list_expected.emplace_back(graph_container::Edge(0, 1, 4));
  edges_list_expected.emplace_back(graph_container::Edge(0, 7, 8));
  edges_list_expected.emplace_back(graph_container::Edge(2, 8, 2));
  edges_list_expected.emplace_back(graph_container::Edge(2, 5, 4));
  edges_list_expected.emplace_back(graph_container::Edge(2, 3, 7));
  edges_list_expected.emplace_back(graph_container::Edge(3, 4, 9));
  edges_list_expected.emplace_back(graph_container::Edge(5, 6, 2));
  edges_list_expected.emplace_back(graph_container::Edge(6, 7, 1));

  EXPECT_EQ(37, msp_total_cost);
  EXPECT_EQ(8, out_msp.noEdges());
  EXPECT_THAT(edges_list, ContainerEq(edges_list_expected));
}

TEST(undirected_graphs, test_mspk_2)
{
  namespace algo = graph::algorithms;
  using graph_container = graph::dense_graph;

  graph_container my_graph(9), out_msp(9);

  my_graph.addLink(0, 1, 4);
  my_graph.addLink(0, 7, 8);
  my_graph.addLink(1, 2, 8);
  my_graph.addLink(1, 7, 11);
  my_graph.addLink(2, 3, 7);
  my_graph.addLink(2, 5, 4);
  my_graph.addLink(2, 8, 2);
  my_graph.addLink(3, 4, 9);
  my_graph.addLink(3, 5, 14);
  my_graph.addLink(4, 5, 10);
  my_graph.addLink(5, 6, 2);
  my_graph.addLink(6, 7, 1);
  my_graph.addLink(6, 8, 6);
  my_graph.addLink(7, 8, 7);

  auto msp_total_cost = algo::MSP<graph_container>::kruskal(my_graph, out_msp);

  std::deque<graph_container::Edge> edges_list;
  out_msp.getAllEdges(edges_list);

  std::deque<graph_container::Edge> edges_list_expected;
  edges_list_expected.emplace_back(graph_container::Edge(0, 1, 4));
  edges_list_expected.emplace_back(graph_container::Edge(0, 7, 8));
  edges_list_expected.emplace_back(graph_container::Edge(2, 3, 7));
  edges_list_expected.emplace_back(graph_container::Edge(2, 5, 4));
  edges_list_expected.emplace_back(graph_container::Edge(2, 8, 2));
  edges_list_expected.emplace_back(graph_container::Edge(3, 4, 9));
  edges_list_expected.emplace_back(graph_container::Edge(5, 6, 2));
  edges_list_expected.emplace_back(graph_container::Edge(6, 7, 1));

  EXPECT_EQ(37, msp_total_cost);
  EXPECT_EQ(8, out_msp.noEdges());
  EXPECT_THAT(edges_list, ContainerEq(edges_list_expected));
}

TEST(directed_graphs, test_compute_all_min_paths_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 5);
  my_graph.addLink(1, 2, 3);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(0, 3, 10);

  graph::common::SquareMatrix<cost_el> mat_paths(4, 0);
  graph::common::SquareMatrix<int> mat_nxt(4, 0);
  algo::Paths<graph_container>::computePathsTrails(my_graph, mat_paths, mat_nxt);

  algo::Paths<graph_container>::vertices_list seq;
  bool path_found_v2_v0 = algo::Paths<graph_container>::extractPath(mat_nxt, 2, 0, seq);
  EXPECT_FALSE(path_found_v2_v0);

  seq.clear();
  bool path_found_v0_v3 = algo::Paths<graph_container>::extractPath(mat_nxt, 0, 3, seq);
  EXPECT_TRUE(path_found_v0_v3);
  EXPECT_THAT(seq, ElementsAre(0, 1, 2, 3));
}

TEST(directed_graphs, test_compute_all_min_paths_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

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

  graph::common::SquareMatrix<cost_el> mat_paths(5, 0);
  graph::common::SquareMatrix<int> mat_nxt(5, 0);
  algo::Paths<graph_container>::computePathsTrails(my_graph, mat_paths, mat_nxt);

  algo::Paths<graph_container>::vertices_list seq;
  bool path_found_v1_v4 = algo::Paths<graph_container>::extractPath(mat_nxt, 1, 4, seq);
  EXPECT_TRUE(path_found_v1_v4);
  EXPECT_THAT(seq, ElementsAre(1, 2, 0, 3, 4));

  bool path_found_v3_v1 = algo::Paths<graph_container>::extractPath(mat_nxt, 3, 1, seq);
  EXPECT_TRUE(path_found_v3_v1);
  EXPECT_THAT(seq, ElementsAre(3, 4, 1));
}

TEST(directed_graphs, test_conn_comps_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 5);
  my_graph.addLink(1, 2, 3);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(0, 3, 10);

  algo::Features<graph_container>::vertices_list_categ conn_comps;
  auto nbConnComps = algo::Features<graph_container>::connectedComponents(my_graph, conn_comps);  // 0 1 3 2

  algo::Features<graph_container>::vertices_list_categ conn_comps_expected = {{0, 1}, {1, 1}, {3, 1}, {2, 1}};

  EXPECT_EQ(1, nbConnComps);
  EXPECT_THAT(conn_comps, ContainerEq(conn_comps_expected));
}

TEST(directed_graphs, test_conn_comps_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

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

  algo::Features<graph_container>::vertices_list_categ conn_comps;
  auto nbConnComps = algo::Features<graph_container>::connectedComponents(my_graph, conn_comps);  // 0 1 3 2 4

  algo::Features<graph_container>::vertices_list_categ conn_comps_expected = {{0, 1}, {1, 1}, {3, 1}, {2, 1}, {4, 1}};

  EXPECT_EQ(1, nbConnComps);
  EXPECT_THAT(conn_comps, ContainerEq(conn_comps_expected));
}

TEST(directed_graphs, test_cycle_present_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(4);
  my_graph.addLink(0, 1, 1);
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 0, 1);
  my_graph.addLink(2, 3, 1);

  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 0));
  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 1));
  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 2));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 3));
}

TEST(directed_graphs, test_cycle_present_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(4);
  my_graph.addLink(0, 1, 1);
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);

  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 0));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 1));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 2));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 3));
}

TEST(directed_graphs, test_cycle_present_3)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(6);
  my_graph.addLink(0, 1, 1);
  my_graph.addLink(0, 2, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 0, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 4, 1);

  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 0));
  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 1));
  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 2));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 3));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 4));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 5));
}

TEST(directed_graphs, test_cycle_present_4)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(3);
  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 0, 1);

  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 0));
  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 1));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 2));
}

TEST(directed_graphs, test_cycle_present_5)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_LIST;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_LIST, DIRECTED>;
  using cost_el = graph_container::cost_element_type;

  graph_container my_graph(3);
  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 0, 1);

  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 0));
  EXPECT_TRUE(algo::Features<graph_container>::hasCycles(my_graph, 1));
  EXPECT_FALSE(algo::Features<graph_container>::hasCycles(my_graph, 2));
}

TEST(directed_graphs, test_bipartite_1)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;

  graph_container my_graph(3);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 0, 1);

  algo::Features<graph_container>::vertices_list_categ vertex_list;
  algo::Features<graph_container>::vertices_list_categ vertex_list_expected = {{0, 0}, {1, 1}, {2, 0}};
  auto isBipartite = algo::Features<graph_container>::isBipartite(my_graph, vertex_list);  // 0 1 2

  EXPECT_FALSE(isBipartite);
  EXPECT_THAT(vertex_list, ContainerEq(vertex_list_expected));
}

TEST(directed_graphs, test_bipartite_2)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;

  graph_container my_graph(4);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 0, 1);

  algo::Features<graph_container>::vertices_list_categ vertex_list;
  algo::Features<graph_container>::vertices_list_categ vertex_list_expected = {{0, 0}, {1, 1}, {2, 0}, {3, 1}};
  auto isBipartite = algo::Features<graph_container>::isBipartite(my_graph, vertex_list);  // 0 1 2 3

  EXPECT_TRUE(isBipartite);
  EXPECT_THAT(vertex_list, ContainerEq(vertex_list_expected));
}

TEST(directed_graphs, test_bipartite_3)
{
  namespace algo = graph::algorithms;
  using graph::ADJACENCY_MATRIX;
  using graph::DIRECTED;
  using graph_container = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;

  graph_container my_graph(5);

  my_graph.addLink(0, 1, 1);
  my_graph.addLink(1, 2, 1);
  my_graph.addLink(2, 3, 1);
  my_graph.addLink(3, 0, 1);

  algo::Features<graph_container>::vertices_list_categ vertex_list;
  algo::Features<graph_container>::vertices_list_categ vertex_list_expected = {{0, 0}, {1, 1}, {2, 0}, {3, 1}, {4, 0}};
  auto isBipartite = algo::Features<graph_container>::isBipartite(my_graph, vertex_list);  // 0 1 2 3 4

  EXPECT_TRUE(isBipartite);
  EXPECT_THAT(vertex_list, ContainerEq(vertex_list_expected));
}
