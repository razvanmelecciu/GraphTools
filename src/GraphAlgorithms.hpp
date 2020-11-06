#ifndef _graph_algorithms_h_
#define _graph_algorithms_h_

#include "GraphBase.hpp"
#include "GraphMisc.hpp"
#include <deque>
#include <unordered_set>
#include <map>
#include <limits>
#include <algorithm>

GRAPH_START
ALGO_START

/**
 * @brief The DegreeType definition for a particular vertex
 */
enum DegreeType : unsigned char { EXTERNAL = 0x00, INTERNAL };

/**
 * @brief Default traits class that defines the vertex list (deque, vector, list)
 *        and the search structure used (set, unordered_set)
 * @tparam vertex_list the type of list in which the nodes are normally stored
 * @tparam search_structure type of search structure used
 * @tparam vertex_list_category_ list of nodes that also have an id attached (integer label)
 * @tparam mapping_structure_ mapping structure used for assigning 'labels' or categories to nodes
 */
template <typename vertex_list_ = std::deque<int>,
          typename search_structure_ = std::unordered_set<int>,
          typename vertex_list_category_ = std::deque<std::pair<int, int>>,
          typename mapping_structure_ = std::map<int, int>>
struct Traits
{
   using vertices_list_trait     = vertex_list_;
   using search_structure_trait  = search_structure_;
   using vertex_list_categ_trait = vertex_list_category_;
   using vertex_categ_mapping_trait = mapping_structure_;
};

/**
 * @brief Class that deals with graph traversals
 * @tparam graph_type the type of the graph
 * @tparam traits encompasses different types of structures used
 */
template <typename graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          typename traits = Traits<>>
struct Traversal
{
   using equivalent_elements = typename graph_type::equivalent_cost_cmp;
   using vertices_list       = typename traits::vertices_list_trait ;
   using search_structure    = typename traits::search_structure_trait;

   /**
    * @brief Breadth first graph traversal (only explores the current connected component in which crt_vertex resides)
    * @param[in] my_graph input graph
    * @param[in] crt_vertex starting node
    * @param[out] node_labels output vertex list with how the nodes have been visited (resulting tree like)
    */
   static void breadthFirst(const graph_type& my_graph,
                            int crt_vertex, vertices_list& node_labels)
   {
      typename graph_type::neighbors_list vertex_queue;
      search_structure visited_nodes;
      std::pair<typename search_structure::iterator, bool> inserted_it;

      vertex_queue.push_back(crt_vertex);
      my_graph.getLinks(crt_vertex, vertex_queue);

      while (vertex_queue.size() > 0)
      {
         inserted_it = visited_nodes.insert(vertex_queue.front());
         if (!inserted_it.second)
         {
            vertex_queue.pop_front();
         }
         else
         {
            auto front_elem = vertex_queue.front();
            node_labels.push_back(front_elem);
            vertex_queue.pop_front();
            if (vertex_queue.size() > 0)
            {
               my_graph.getLinks(front_elem, vertex_queue);
            }
         }
      }
   }

   /**
    * @brief Depth first graph traversal (only explores the current connected component in which crt_vertex resides)
    * @param[in] my_graph input graph
    * @param[in] crt_vertex starting node
    * @param[out] node_labels output vertex list with how the nodes have been visited (resulting tree like)
    */
   static void depthFirst(const graph_type& my_graph,
                          int crt_vertex, vertices_list& node_labels)
   {
      search_structure visited_nodes;
      dfRecursive(my_graph, crt_vertex, node_labels, visited_nodes);
   }

private :

   /**
    * @brief Internally used for dfs
    * @param[in] my_graph input graph
    * @param[in] crt_vertex starting node
    * @param[out] node_labels output vertex list with how the nodes have been visited (resulting tree like)
    * @param[in/out] visited_nodes list of already visited nodes up to a particular point
    */
   static void dfRecursive(const graph_type& my_graph, int crt_vertex,
                           vertices_list& node_labels, search_structure& visited_nodes)
   {
      typename graph_type::neighbors_list crt_vertex_queue;
      std::pair<typename search_structure::iterator, bool> inserted_it;

      inserted_it = visited_nodes.insert(crt_vertex);
      if (inserted_it.second)
      {
         node_labels.push_back(crt_vertex);
      }

      my_graph.getLinks(crt_vertex, crt_vertex_queue);

      typename graph_type::neighbors_list::const_iterator crt_vertex_it(crt_vertex_queue.begin());
      typename graph_type::neighbors_list::const_iterator end(crt_vertex_queue.end());

      for (; crt_vertex_it != end; ++crt_vertex_it)
      {
         if (visited_nodes.find(*crt_vertex_it) == visited_nodes.end())
         {
            dfRecursive(my_graph, *crt_vertex_it, node_labels, visited_nodes);
         }
      }
   }
};

/**
 * @brief Class that deals with different graph features like cycles, degrees, connected components
 * @tparam graph_type the type of the graph
 * @tparam traits encompasses different types of structures used
 */
template <typename graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          typename traits = Traits<>>
struct Features
{
   using vertices_list        = typename traits::vertices_list_trait;
   using search_structure     = typename traits::search_structure_trait;
   using vertices_list_categ  = typename traits::vertex_list_categ_trait;
   using vertex_mapping_categ = typename traits::vertex_categ_mapping_trait;

   /**
    * @brief Compute the internal/external degree of a node (for undirected graphs the internal and external degree are the same)
    * @param[in] my_graph input graph
    * @param[in] input_vertex the vertex used for the calculation
    * @param[in] degr_option valid for digraphs, either INTERNAL/EXTERNAL
    * @return the degree of the node
    */
   static unsigned int computeDegree(const graph_type& my_input_graph,
                                     int input_vertex, DegreeType degr_option = EXTERNAL)
   {
      unsigned int vertices_nb = my_input_graph.noVertices();

      if (input_vertex >= static_cast<int>(vertices_nb) || input_vertex < 0)
      {
         assert(false && "Invalid vertex specified");
         return false;
      }

      unsigned int comp_degree = 0;

      if (graph_type::directed_graph)
      {
         switch (degr_option)
         {
            case graph::algorithms::EXTERNAL:
            default:
            {
               for (int i = 0; i < static_cast<int>(vertices_nb); ++i)
               {
                  if (my_input_graph.hasLink(input_vertex, i))
                  {
                     ++comp_degree;
                  }
               }
               break;
            }
            case graph::algorithms::INTERNAL:
            {
               for (int i = 0; i < static_cast<int>(vertices_nb); ++i)
               {
                  if (my_input_graph.hasLink(i, input_vertex))
                  {
                     ++comp_degree;
                  }
               }
               break;
            }
         }
      }
      else
      {
         for (int i = 0; i < static_cast<int>(vertices_nb); ++i)
         {
            if (my_input_graph.hasLink(input_vertex, i))
            {
               ++comp_degree;
            }
         }
      }

      return comp_degree;
   }

   /**
    * @brief Extract Connected components (returns the nb of connected components and the lists of vertices)
    * @param[in] my_graph input graph
    * @param[out] node_labels list of connected nodes each node with its connected component id
    * @return the number of connected components
    */
   static unsigned int connectedComponents(const graph_type& my_input_graph,
                                           vertices_list_categ& node_labels)
   {
      return connectedCompsHelper(my_input_graph, &node_labels);
   }

   /**
    * @brief Checks if the graph is connected
    * @param[in] my_graph input graph
    * @return true/false
    */
   static bool isConnected(const graph_type& my_input_graph)
   {
      auto nb_connected_comps = connectedCompsHelper(my_input_graph);
      return (nb_connected_comps == 1);
   }

   /**
    * @brief Check if the graph is bipartite (returns a vertex list with the category)
    * @param[in] my_graph input graph
    * @param[out] vertex_list list of nodes with the color(side) attached
    * @return true/false
    */
   static bool isBipartite(const graph_type& my_graph, vertices_list_categ& vertex_list)
   {
      bool isBipartite = true;

      typename graph_type::neighbors_list crt_neighbors;
      vertex_mapping_categ visited_nodes;
      std::pair<typename vertex_mapping_categ::iterator, bool> inserted_it;

      auto nb_vertices = my_graph.noVertices();
      for (auto crt_vertex = 0 ; crt_vertex < nb_vertices; ++crt_vertex)
      {
         if (visited_nodes.find(crt_vertex) != visited_nodes.end())
         {
            continue;
         }

         typename vertices_list_categ::value_type::second_type current_category = 0;
         typename vertices_list_categ::value_type crt_vertex_category(crt_vertex, current_category);

         vertex_list.push_back(crt_vertex_category);
         (void)visited_nodes.insert(crt_vertex_category);
         typename vertices_list_categ::const_iterator it_crt = vertex_list.end();
         --it_crt;

         while (it_crt != vertex_list.end())
         {
            crt_vertex = it_crt->first;
            current_category = it_crt->second;

            crt_neighbors.clear();
            my_graph.getLinks(crt_vertex, crt_neighbors);

            for (const auto& crt_node : crt_neighbors)
            {
               crt_vertex_category.first = crt_node;
               crt_vertex_category.second = (current_category + 1) % 2;

               inserted_it = visited_nodes.insert(crt_vertex_category);
               if (inserted_it.second) // node inserted
               {
                  vertex_list.push_back(crt_vertex_category);
               }
               else // check previous category for nodes already visited
               {
                  if (inserted_it.first->second != crt_vertex_category.second)
                  {
                     isBipartite = false;
                  }
               }
            }

            ++it_crt;
         }
      }

      return isBipartite;
   }

   /**
    * @brief Checks if the there is at least a cycle in the subgraph that contains the specified vertex
    * @param[in] my_graph input graph
    * @param[in] crt_vertex vertex that will be checked
    * @return true/false
    */
   static bool hasCycles(const graph_type& my_input_graph, int crt_vertex)
   {
      search_structure visited_nodes;
      if (graph_type::directed_graph)
      {
         search_structure rec_chain_visited;
         return dfCycleDetectedD(my_input_graph, crt_vertex, visited_nodes, rec_chain_visited);
      }
      else
      {
         return dfCycleDetectedU(my_input_graph, crt_vertex, visited_nodes);
      }
   }

   /**
    * @brief Check Complete graph (basically verifies if the number of edges has reached the maximum possible)
    * @param[in] my_graph input graph
    * @return true/false
    */
   static bool completeGraph(const graph_type& my_input_graph)
   {
      unsigned int max_edges = my_input_graph.maxNoEdges();
      unsigned int no_edges = my_input_graph.noEdges();

      return (no_edges == max_edges);
   }

   /**
    * @brief Compute Transitive closure (Roy-Warshall -> places 1 as the cost for the vertex connections)
    * @param[in] my_input_graph input graph
    * @param[out] transitive_closure square matrix which contains the components that are reachable
    */
   static void transitiveClosure(const graph_type& my_input_graph,
                                 graph::common::SquareMatrix<bool>& transitive_closure)
   {
      unsigned int vertices_nb = my_input_graph.noVertices();
      transitive_closure.setSize(vertices_nb, false);
      int i = 0, j = 0, k = 0;

      for (i = 0; i < vertices_nb; ++i)
      {
         for (j = 0; j < vertices_nb; ++j)
         {
            transitive_closure(i, j) = my_input_graph.hasLink(i, j);
         }
      }

      for (k = 0; k < vertices_nb; ++k)
      {
         for (i = 0; i < vertices_nb; ++i)
         {
            for (j = 0; j < vertices_nb; ++j)
            {
               transitive_closure(i, j) = transitive_closure(i, j) || (transitive_closure(i, k) && transitive_closure(k, j));
            }
         }
      }
   }

private :

   /**
    * @brief Helper function used to compute the connected components
    * @param[in] my_input_graph input graph
    * @param[out] node_labels list of connected nodes each with its component id
    * @return number of connected components
    */
   static unsigned int connectedCompsHelper(const graph_type& my_input_graph,
                                            vertices_list_categ* node_labels=nullptr)
   {
      search_structure visited_vertices;
      vertices_list crt_node_labels;
      unsigned int nb_conn_components = 0;

      auto nb_vertices = my_input_graph.noVertices();
      typename vertices_list_categ::value_type crt_vertex_conn_component;

      for (auto i = 0 ; i < nb_vertices; ++i)
      {
         if (visited_vertices.find(i) == visited_vertices.end())
         {
            ++nb_conn_components;
            Traversal<graph_type, traits>::breadthFirst(my_input_graph, i, crt_node_labels);
            crt_vertex_conn_component.second = nb_conn_components;

            for (const auto& elem : crt_node_labels)
            {
               crt_vertex_conn_component.first = elem;
               if (node_labels)
               {
                  node_labels->push_back(crt_vertex_conn_component);
               }

               visited_vertices.insert(elem);
            }
            crt_node_labels.clear();
         }
      }

      return nb_conn_components;
   }

   /**
    * @brief Helper function for cycle detection in undirected graphs
    * @param[in] my_graph input graph
    * @param[in] crt_vertex current starting vertex
    * @param[in/out] visited_nodes list of nodes visited so far
    * @param[in] parent_vertex parent of the current vertex
    * @return true/false
    */
   static bool dfCycleDetectedU(const graph_type& my_graph, int crt_vertex,
                                search_structure& visited_nodes, int parent_vertex = -1)
   {
      typename graph_type::neighbors_list crt_vertex_queue;
      std::pair<typename search_structure::iterator, bool> inserted_it;

      inserted_it = visited_nodes.insert(crt_vertex);
      my_graph.getLinks(crt_vertex, crt_vertex_queue);

      typename graph_type::neighbors_list::const_iterator crt_vertex_it(crt_vertex_queue.begin());
      typename graph_type::neighbors_list::const_iterator end(crt_vertex_queue.end());

      if (parent_vertex < 0)
      {
         parent_vertex = crt_vertex;
      }

      for (; crt_vertex_it != end; ++crt_vertex_it)
      {
         // exclude the parent from the list of neighbors
         if (*crt_vertex_it == parent_vertex)
         {
            continue;
         }

         if (visited_nodes.find(*crt_vertex_it) == visited_nodes.end())
         {
            bool result = dfCycleDetectedU(my_graph, *crt_vertex_it, visited_nodes, crt_vertex);
            // end exploration once a cycle has been found
            if (result)
            {
               return result;
            }
         }
         else
         {
            return true;
         }
      }

      // case for an isolated node
      return false;
   }

   /**
    * @brief Helper function for cycle detection in directed graphs
    * @param[in] my_graph input graph
    * @param[in] crt_vertex current starting vertex
    * @param[in/out] visited_nodes list of nodes visited so far
    * @param[in/out] rec_chain_visited current recursion chain of nodes
    * @return true/false
    */
   static bool dfCycleDetectedD(const graph_type& my_graph, int crt_vertex,
                                search_structure& visited_nodes,
                                search_structure& rec_chain_visited)
   {
      typename graph_type::neighbors_list crt_vertex_queue;
      std::pair<typename search_structure::iterator, bool> inserted_it;

      inserted_it = visited_nodes.insert(crt_vertex);
      my_graph.getLinks(crt_vertex, crt_vertex_queue);

      // keep track of the vertexes that are in the crt recursion chain
      rec_chain_visited.insert(crt_vertex);

      typename graph_type::neighbors_list::const_iterator crt_vertex_it(crt_vertex_queue.begin());
      typename graph_type::neighbors_list::const_iterator end(crt_vertex_queue.end());

      for (; crt_vertex_it != end; ++crt_vertex_it)
      {
         if (rec_chain_visited.find(*crt_vertex_it) != rec_chain_visited.end())
         {
            // vertex found in the crt recursive chain - cycle detected
            return true;
         }

         if (visited_nodes.find(*crt_vertex_it) == visited_nodes.end())
         {
            bool cycle_detected = dfCycleDetectedD(my_graph, *crt_vertex_it, visited_nodes, rec_chain_visited);
            // end exploration once a cycle has been found
            if (cycle_detected)
            {
               rec_chain_visited.erase(crt_vertex);
               return cycle_detected;
            }
         }
      }

      rec_chain_visited.erase(crt_vertex);
      return false;
   }
};

/**
 * @brief Class that deals with the generation of MSPs
 * @tparam graph_type the type of the graph
 * @tparam traits encompasses different types of structures used
 */
template <typename graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          typename traits = Traits<>>
struct MSP
{
   static_assert(graph_type::directed_graph == 0, "Invalid graph type");

   /**
    * @brief Determines the minimum spanning tree for the input graph along with
    *        a total connection cost using Kruskal's algorithm. If the input graph is not connected
    *        the output will be a forest of minimum spanning trees and the method will
    *        return the total cost
    * @param[in] my_graph input graph
    * @param[out] output_msp msp output graph
    * @return returns the total cost of the MSP
    */
   static typename graph_type::cost_element_type kruskal(const graph_type& my_graph, graph_type& output_msp)
   {
      if (output_msp.noVertices() != my_graph.noVertices())
      {
         assert(false && "Invalid type of output specified");
         return 0;
      }

      using UnionFinder = graph::common::UnionFind<int>;
      auto union_finder = UnionFinder(my_graph.noVertices());
      typename graph_type::cost_element_type total_cost = 0;

      output_msp.clearAllLinks();

      // Get all from the graph the edges sorted out
      std::deque<typename graph_type::Edge> edges_list;
      my_graph.getAllEdges(edges_list);
      std::sort(edges_list.begin(), edges_list.end());
      bool union_ok = true;
      auto nb_edges_added = 0;

      for (const auto& crt_edge : edges_list)
      {
         if (nb_edges_added == expected_msp_no_edges(my_graph))
         {
            break;
         }

         union_ok = union_finder.makeUnion(crt_edge.start_, crt_edge.end_);
         // Produces a cycle so continue
         if (!union_ok)
         {
            continue;
         }
         // total cost and the associated tree
         total_cost += crt_edge.edge_cost_;
         output_msp.addLink(crt_edge.start_, crt_edge.end_, crt_edge.edge_cost_);
         ++nb_edges_added;
      }

      return total_cost;
   }

   /**
    * @brief Determines the minimum spanning tree for the input graph along with
    *        a total connection cost using Prim's algorithm
    * @param[in] my_graph input graph
    * @param[out] output_msp msp output graph
    * @return returns the total cost of the MSP
    */
   static typename graph_type::cost_element_type prim(const graph_type& my_graph, graph_type& output_msp)
   {
      typename graph_type::cost_element_type total_cost = 0;

      output_msp.clear();

      return total_cost;
   }

private:

   /**
    * @brief Returns the number of edges expected to be found in an msp for an input graph
    * @param my_graph input graph
    * @return the number of edges expected
    */
   static unsigned int expected_msp_no_edges(const graph_type& my_graph)
   {
      return my_graph.noVertices() - 1;
   }
};

/**
 * @brief Class that deals with paths in graphs
 * @tparam graph_type the type of the graph
 * @tparam traits encompasses different types of structures used
 */
template <typename graph_type = graph::GraphContainer<ADJACENCY_MATRIX>,
          typename traits = Traits<>>
struct Paths
{
   using cost_type     = typename graph_type::cost_element_type ;
   using vertices_list = typename traits::vertices_list_trait;
   using cost_nxt      = typename std::pair<cost_type, int>;
   enum { null_vertex = -1 };

   /// Djikstra
   //TODO

   /**
    * @brief Compute all the minimum paths (Floyd-Warshall-returns a square matrix with the associated cost)
    * @param[in] my_input_graph input graph
    * @param[out] cost_output square matrix with the associated costs for reaching a specific node
    */
   static void computePaths(const graph_type& my_input_graph,
                            graph::common::SquareMatrix<cost_type>& cost_output)
   {
      cost_type max_val = std::numeric_limits<cost_type>::max() / 1000;
      auto vertices_nb = my_input_graph.noVertices();
      cost_output.setSize(vertices_nb, 0);
      typename graph_type::cost_element_type crt_cost = 0;

      int i = 0, j = 0, k = 0;

      for (i = 0; i < vertices_nb; ++i)
      {
         for (j = 0; j < vertices_nb; ++j)
         {
            if (i != j)
            {
               if (my_input_graph.hasLink(i, j))
               {
                  cost_output(i, j) = my_input_graph.getCost(i, j);
               }
               else
               {
                  cost_output(i, j) = max_val;
               }
            }
         }
      }

      for (k = 0; k < vertices_nb; ++k)
      {
         for (i = 0; i < vertices_nb; ++i)
         {
            for (j = 0; j < vertices_nb; ++j)
            {
               crt_cost = cost_output(i, k) + cost_output(k, j);
               if (crt_cost < cost_output(i, j))
               {
                  cost_output(i, j) = crt_cost;
               }
            }
         }
      }
   }

   /**
    * @brief Compute all the minimum paths (Floyd-Warshall-returns a square matrix with the
    *        associated cost and a matrix with the next vertex for reconstructing the minimum paths)
    * @param[in] my_input_graph input graph
    * @param[out] cost_output square matrix with the associated costs for reaching a specific node
    * @param[out] nxt_matrix square matrix used for keeping track of the next closest/cheapest vertex
    */
   static void computePathsTrails(const graph_type& my_input_graph, graph::common::SquareMatrix<cost_type>& cost_output,
                                  graph::common::SquareMatrix<int>& nxt_matrix)
   {
      cost_type max_val = std::numeric_limits<cost_type>::max() / 1000;
      auto vertices_nb = my_input_graph.noVertices();

      cost_output.setSize(vertices_nb, 0);
      nxt_matrix.setSize(vertices_nb, null_vertex);

      typename graph_type::cost_element_type crt_cost = 0;

      unsigned int i = 0, j = 0, k = 0;

      for (i = 0; i < vertices_nb; ++i)
      {
         for (j = 0; j < vertices_nb; ++j)
         {
            if (i != j)
            {
               if (my_input_graph.hasLink(i, j))
               {
                  cost_output(i, j) = my_input_graph.getCost(i, j);
                  nxt_matrix(i, j) = j;
               }
               else
               {
                  cost_output(i, j) = max_val;
               }
            }
         }
      }

      for (k = 0; k < vertices_nb; ++k)
      {
         for (i = 0; i < vertices_nb; ++i)
         {
            for (j = 0; j < vertices_nb; ++j)
            {
               crt_cost = cost_output(i, k) + cost_output(k, j);
               if (crt_cost < cost_output(i, j))
               {
                  cost_output(i, j) = crt_cost;
                  nxt_matrix(i, j) = nxt_matrix(i, k);
               }
            }
         }
      }
   }

   /**
    * @brief Extract a path from the specified square matrix (after calling computePathsTrails)
    * @param[in] mat_paths nxt_matrix output from computePathsTrails
    * @param[in] i starting vertex
    * @param[in] j ending vertex
    * @param[out] sequence_trail sequence of nodes traversed from vertex i to vertex j
    * @return true if a path exists, false otherwise
    */
   static bool extractPath(const graph::common::SquareMatrix<int>& mat_paths, int i, int j,
                           vertices_list& sequence_trail)
   {
      sequence_trail.clear();

      auto vertices_nb = mat_paths.getSize();
      if (i >= static_cast<int>(vertices_nb) || j >= static_cast<int>(vertices_nb) || i < 0 || j < 0)
      {
         assert(false && "Invalid vertices specified");
         return false;
      }

      auto nxt_vertex = mat_paths.accessElementCst(i, j);
      if (nxt_vertex == null_vertex)
      {
         return false;
      }

      sequence_trail.push_back(i);
      while (nxt_vertex != null_vertex)
      {
         sequence_trail.push_back(nxt_vertex);
         nxt_vertex = mat_paths.accessElementCst(nxt_vertex, j);
      }

      return true;
   }

   /// Bellman-Ford
   //TODO
};

ALGO_END
GRAPH_END

#endif // !_graph_algorithms_h_
