#ifndef _graph_container_h_
#define _graph_container_h_

#include "GraphBase.hpp"

GRAPH_START

/**
 * @brief A graph container class meant for keeping the nodes of a graph and
 *        the associated edge costs(must be positive)
 * @tparam storage_method either ADJACENCY_MATRIX or ADJACENCY_LIST
 * @tparam link_type DIRECTED (digraph) or UNDIRECTED (graph)
 * @tparam cost_type type for the distance/cost between nodes (must be positive)
 * @tparam eq_cost functor for checking if 2 costs are the same
 * @tparam smaller_cost functor for checking if a cost is smaller than another
 */
template <StoragePolicy storage_method,
          LinkType link_type = UNDIRECTED,
          typename cost_type = int,
          typename eq_cost = EquivalentCost<cost_type>,
          typename smaller_cost = SmallerCost<cost_type> >
class GraphContainer :
   public GraphBase<storage_method, link_type, cost_type, eq_cost, smaller_cost>
{
};

/**
* @brief A graph container class specialized for sparse graphs/digraphs
* @tparam cost_type type for the distance/cost between nodes
* @tparam link_type DIRECTED (digraph) or UNDIRECTED (graph)
* @tparam eq_cost functor for checking if 2 costs are the same
* @tparam smaller_cost functor for checking if a cost is smaller than another
*/
template <typename cost_type,
          LinkType link_type,
          typename eq_cost,
          typename smaller_cost>
class GraphContainer<ADJACENCY_LIST, link_type, cost_type, eq_cost, smaller_cost> :
   public GraphBase<ADJACENCY_LIST, link_type, cost_type, eq_cost, smaller_cost>
{
public:

   using base_type = GraphBase<ADJACENCY_LIST, link_type, cost_type, eq_cost, smaller_cost>;

   /**
    * @brief Ctor
    * @param[in] no_vertices
    */
   GraphContainer(unsigned int no_vertices) :
      no_vertices_(no_vertices)
   {
      collection_node_edge_lists_ = new node_edge_list[no_vertices_];
   }

   /**
    * @brief Copy ctor
    * @param[in] rhs_elem
    */
   GraphContainer(const GraphContainer& rhs_elem) :
      no_vertices_(rhs_elem.no_vertices_)
   {
      collection_node_edge_lists_ = new node_edge_list[no_vertices_];

      for (unsigned int i = 0; i < no_vertices_; ++i)
      {
         *(collection_node_edge_lists_ + i) = *(rhs_elem.collection_node_edge_lists_ + i);
      }
   }

#ifndef _MOVE_SEMANTICS_OFF
   /**
    * @brief Move ctor
    * @param[in] rhs_elem
    */
   GraphContainer(GraphContainer&& rhs_elem) :
      no_vertices_(rhs_elem.no_vertices_)
   {
      if (rhs_elem.collection_node_edge_lists_)
      {
         collection_node_edge_lists_ = rhs_elem.collection_node_edge_lists_;
         rhs_elem.collection_node_edge_lists_ = nullptr;
      }
   }
#endif

   /**
    * @brief Copy assignment
    * @param rhs_elem
    * @return ref to the current element
    */
   GraphContainer& operator = (const GraphContainer& rhs_elem)
   {
      if (this != &rhs_elem)
      {
         if (collection_node_edge_lists_)
         {
            delete[] collection_node_edge_lists_;
         }

         no_vertices_ = rhs_elem.no_vertices_;
         collection_node_edge_lists_ = new node_edge_list[no_vertices_];

         for (unsigned int i = 0; i < no_vertices_; ++i)
         {
            *(collection_node_edge_lists_ + i) = *(rhs_elem.collection_node_edge_lists_ + i);
         }
      }

      return *this;
   }

#ifndef _MOVE_SEMANTICS_OFF
   /**
    * @brief Move assignment
    * @param rhs_elem
    * @return ref to the current element
    */
   GraphContainer& operator = (GraphContainer&& rhs_elem)
   {
      if (this != &rhs_elem)
      {
         if (collection_node_edge_lists_)
         {
            delete[] collection_node_edge_lists_;
         }

         no_vertices_ = rhs_elem.no_vertices_;
         collection_node_edge_lists_ = rhs_elem.collection_node_edge_lists_;
         rhs_elem.collection_node_edge_lists_ = nullptr;
      }

      return *this;
   }
#endif

   // - Accessors

   /**
    * @brief Get the number of vertices
    * @return nb of vertices
    */
   unsigned int noVertices() const
   {
      return no_vertices_;
   }

   /**
    * @brief Count the number of defined edges
    * @return number of edges inserted
    */
   unsigned int noEdges() const
   {
      unsigned int no_links = 0;
      for (unsigned int i = 0; i < no_vertices_; ++i)
      {
         no_links += static_cast<unsigned int>(collection_node_edge_lists_[i].size());
      }

      if (!base_type::directed_graph)
      {
         no_links /= 2;
      }

      return no_links;
   }

   /**
    * @brief Get the maximum number of edges (refflexivity not included e.g. xRx)
    * @return max nb of edges
    */
   unsigned int maxNoEdges() const
   {
      unsigned int max_nb_edge = no_vertices_ * (no_vertices_ - 1);  // digraph
      if (!base_type::directed_graph)                                // graphs (undirected graphs)
      {
         max_nb_edge /= 2;
      }

      return max_nb_edge;
   }

   /**
    * @brief Has link between i and j
    * @param[in] i starting vertex
    * @param[in] j end vertex
    * @return true/false
    */
   bool hasLink(int i, int j) const
   {
      if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) ||
          i < 0 || j < 0)
      {
         assert(false && "Invalid vertices specified");
         return false;
      }

      typename node_edge_list::const_iterator itCrt(collection_node_edge_lists_[i].begin());
      typename node_edge_list::const_iterator itEnd(collection_node_edge_lists_[i].end());

      for (; itCrt != itEnd; ++itCrt)
      {
         if (itCrt->first == j)
         {
            return true;
         }
      }

      return false;
   }

   /**
    * @brief Get the cost from node i to node j
    * @param[in] i starting vertex
    * @param[in] j end vertex
    * @return cost/cost from node i to node j
    */
   cost_type getCost(int i, int j) const
   {
      cost_type crt_cost = base_type::undefined_cost;
      if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) ||
          i < 0 || j < 0 || i == j)
      {
         assert(false && "Invalid vertices specified");
         return crt_cost;
      }

      typename node_edge_list::const_iterator itCrt(collection_node_edge_lists_[i].begin());
      typename node_edge_list::const_iterator itEnd(collection_node_edge_lists_[i].end());

      for (; itCrt != itEnd; ++itCrt)
      {
         if (itCrt->first == j)
         {
            return static_cast<cost_type>(itCrt->second);
         }
      }

      return crt_cost;
   }

   /**
    * @brief Check if the relation is symmetric relation iRj and jRi (not that the cost is the same)
    * @param[in] i starting vertex
    * @param[in] j end vertex
    * @return true/false
    */
   bool symmetricRelation(int i, int j) const
   {
      if (i >= no_vertices_ || j >= no_vertices_ || i < 0 || j < 0 || i == j)
      {
         assert(false && "Invalid vertices specified");
         return false;
      }

      if (base_type::directed_graph)
      {
         if (hasLink(i, j) && hasLink(j, i))
         {
            return true;
         }
      }

      return false;
   }

   /**
    * @brief Get the list of links for the current node
    * @param[in] vertex node used for extracting the neighbors
    * @param[out] vertex_list list of neighbors
    */
   void getLinks(int vertex, typename base_type::neighbors_list& vertex_list) const
   {
      if (vertex >= static_cast<int>(no_vertices_) || vertex < 0)
      {
         assert(false && "Invalid vertex specified");
         return;
      }

      for (const auto& elem : collection_node_edge_lists_[vertex])
      {
         vertex_list.push_back(elem.first);
      }
   }

   /**
    * @brief Get a list with all the edges defined
    * @param[out] links_list output list with all the links
    */
   void getAllEdges(std::deque<typename base_type::Edge>& links_list) const
   {
      links_list.clear();
      typename base_type::Edge crtEdge;

      if (base_type::directed_graph)
      {
         for (unsigned int i = 0; i < no_vertices_; ++i)
         {
            crtEdge.start_ = i;
            for (const auto& elem : collection_node_edge_lists_[i])
            {
               crtEdge.end_ = elem.first;
               crtEdge.edge_cost_ = elem.second;
               links_list.push_back(crtEdge);
            }
         }
      }
      else
      {
         for (unsigned int i = 0; i < no_vertices_; ++i)
         {
            crtEdge.start_ = i;
            for (const auto& elem : collection_node_edge_lists_[i])
            {
               if (elem.first > i)
               {
                  crtEdge.end_ = elem.first;
                  crtEdge.edge_cost_ = elem.second;
                  links_list.push_back(crtEdge);
               }
            }
         }
      }
   }

   // - Mutators

   /**
    * @brief Add an edge from vertex i to vertex j (or both vertices if the relation is symmetric)
    * @param[in] i starting vertex
    * @param[in] j ending vertex
    * @param[in] cost cost associated
    * @param[in] symmetric if the link will be created both ways (only relevant for digraphs)
    */
   void addLink(int i, int j, const cost_type& cost, bool symmetric = false)
   {
      if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) ||
          i < 0 || j < 0 || i == j)
      {
         assert(false && "Invalid vertices specified");
         return;
      }

      if (cost < 0)
      {
         assert(false && "Invalid cost (must be a positive value)");
         return;
      }

      collection_node_edge_lists_[i].push_back(list_node(j, cost));
      if ((base_type::directed_graph && symmetric) || !base_type::directed_graph)
      {
         collection_node_edge_lists_[j].push_back(list_node(i, cost));
      }
   }

   /**
    * @brief Remove an edge from vertex i to vertex j
    * @param[in] i starting vertex
    * @param[in] j ending vertex
    * @return the cost that was set before clearing the link
    */
   cost_type clearLink(int i, int j)
   {
      if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) ||
          i < 0 || j < 0 || i == j)
      {
         assert(false && "Invalid vertices specified");
         return base_type::undefined_cost;
      }

      typename node_edge_list::const_iterator itCrt(collection_node_edge_lists_[i].begin());
      typename node_edge_list::const_iterator itEnd(collection_node_edge_lists_[i].end());

      for (; itCrt != itEnd; ++itCrt)
      {
         if (itCrt->first == j)
         {
            cost_type prev_value = static_cast<cost_type>(itCrt->second);
            collection_node_edge_lists_[i].erase(itCrt);
            return prev_value;
         }
      }

      return base_type::undefined_cost;
   }

   /**
    * @brief Clear all edges
    */
   void clearAllLinks()
   {
      for (unsigned int i = 0; i < no_vertices_; ++i)
      {
         collection_node_edge_lists_[i].clear();
      }
   }

protected:

   // - Members

   using list_node = std::pair<int, typename base_type::cost_element_type>;
   using node_edge_list = std::list<list_node>;

   node_edge_list* collection_node_edge_lists_;                                 ///< pointer to an array of lists
   unsigned int no_vertices_;                                                   ///< no of vertices
};

/**
* @brief A graph container class specialized for dense graphs/digraphs
* @tparam cost_type type for the distance/cost between nodes
* @tparam link_type DIRECTED (digraph) or UNDIRECTED (graph)
* @tparam eq_cost functor for checking if 2 costs are the same
* @tparam smaller_cost functor for checking if a cost is smaller than another
*/
template <typename cost_type,
          LinkType link_type,
          typename eq_cost,
          typename smaller_cost>
class GraphContainer<ADJACENCY_MATRIX, link_type, cost_type, eq_cost, smaller_cost> :
   public GraphBase<ADJACENCY_MATRIX, link_type, cost_type, eq_cost, smaller_cost>
{
public:

   using base_type = GraphBase<ADJACENCY_MATRIX, link_type, cost_type, eq_cost, smaller_cost>;

   /**
    * @brief Ctor
    * @param[in] no_vertices
    */
   GraphContainer(unsigned int no_vertices) :
      no_vertices_(no_vertices)
   {
      unsigned int alloc_size = allocSize();
      adjacency_matrix_ = new typename base_type::cost_element_type[alloc_size];
      for (unsigned int i = 0; i < alloc_size; ++i)
      {
         *(adjacency_matrix_ + i) = base_type::undefined_cost;
      }
   }

   ///
   /**
    * @brief Ctor (practically creates a complete graph with n(n-1) edges for digraphs
    *        or n(n-1)/2 edges for undirected graphs with the specified cost)
    * @param[in] no_vertices
    * @param[in] initial_cost
    */
   GraphContainer(unsigned int no_vertices, const cost_type& initial_cost) :
      no_vertices_(no_vertices)
   {
      unsigned int alloc_size = allocSize();
      adjacency_matrix_ = new typename base_type::cost_element_type[alloc_size];
      for (unsigned int i = 0; i < alloc_size; ++i)                // safe to put since the vertices don't generally have links to themselves
      {
         *(adjacency_matrix_ + i) = initial_cost;
      }
      for (unsigned int i = 0; i < no_vertices_; ++i)
      {
         MatrixElem(i, i) = base_type::undefined_cost;
      }
   }

   /**
    * @brief Copy ctor
    * @param[in] rhs_elem
    */
   GraphContainer(const GraphContainer& rhs_elem) :
      no_vertices_(rhs_elem.no_vertices_)
   {
      unsigned int alloc_size = allocSize();
      adjacency_matrix_ = new typename base_type::cost_element_type[alloc_size];
      for (unsigned int i = 0; i < alloc_size; ++i)
      {
         *(adjacency_matrix_ + i) = *(rhs_elem.adjacency_matrix_ + i);
      }
   }

#ifndef _MOVE_SEMANTICS_OFF
   /**
    * @brief Move ctor
    * @param[in] rhs_elem
    */
   GraphContainer(GraphContainer&& rhs_elem) :
      no_vertices_(rhs_elem.no_vertices_)
   {
      if (rhs_elem.adjacency_matrix_)
      {
         adjacency_matrix_ = rhs_elem.adjacency_matrix_;
         rhs_elem.adjacency_matrix_ = nullptr;
      }
   }
#endif

   /**
    * @brief Copy assignment
    * @param[in] rhs_elem
    */
   GraphContainer& operator = (const GraphContainer& rhs_elem)
   {
      if (this != &rhs_elem)
      {
         if (adjacency_matrix_)
         {
            delete[] adjacency_matrix_;
         }

         no_vertices_ = rhs_elem.no_vertices;
         unsigned int alloc_size = allocSize();

         adjacency_matrix_ = new typename base_type::cost_element_type[alloc_size];
         for (unsigned int i = 0; i < alloc_size; ++i)
         {
            *(adjacency_matrix_ + i) = *(rhs_elem.adjacency_matrix_ + i);
         }
      }

      return *this;
   }

#ifndef _MOVE_SEMANTICS_OFF
   /**
    * @brief Move assignment
    * @param[in] rhs_elem
    */
   GraphContainer& operator = (GraphContainer&& rhs_elem)
   {
      if (this != &rhs_elem)
      {
         if (adjacency_matrix_)
         {
            delete[] adjacency_matrix_;
         }

         no_vertices_ = rhs_elem.no_vertices;
         adjacency_matrix_ = rhs_elem.adjacency_matrix_;
         rhs_elem.adjacency_matrix_ = nullptr;
      }

      return *this;
   }
#endif

   // - Accessors

   /**
    * @brief Get the number of vertices
    * @return nb of vertices
    */
   unsigned int noVertices() const
   {
      return no_vertices_;
   }

   /**
    * @brief Count the number of defined edges
    * @return number of edges inserted
    */
   unsigned int noEdges() const
   {
      unsigned int alloc_size = allocSize();
      unsigned int no_edges = 0;
      typename base_type::equivalent_cost_cmp equiv;

      for (unsigned int i = 0; i < alloc_size; ++i)
      {
         if (!equiv(*(adjacency_matrix_ + i), static_cast<cost_type>(base_type::undefined_cost)))
         {
            ++no_edges;
         }
      }

      return no_edges;
   }

   /**
    * @brief Get the maximum number of edges (refflexivity not included e.g. xRx)
    * @return max nb of edges
    */
   unsigned int maxNoEdges() const
   {
      unsigned int max_nb_edge = no_vertices_ * (no_vertices_ - 1);  // digraph
      if (!base_type::directed_graph)                                // graphs (undirected graphs)
      {
         max_nb_edge /= 2;
      }

      return max_nb_edge;
   }

   /**
    * @brief Has link between i and j
    * @param[in] i starting vertex
    * @param[in] j end vertex
    * @return true/false
    */
   bool hasLink(int i, int j) const
   {
      if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) ||
          i < 0 || j < 0)
      {
         assert(false && "Invalid vertices specified");
         return false;
      }

      typename base_type::equivalent_cost_cmp equiv;

      if (equiv(MatrixElemCst(i, j), static_cast<cost_type>(base_type::undefined_cost)))
      {
         return false;
      }
      return true;
   }

   /**
    * @brief Get the cost from node i to node j
    * @param[in] i starting vertex
    * @param[in] j end vertex
    * @return cost/cost from node i to node j
    */
   cost_type getCost(int i, int j) const
   {
      if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) ||
          i < 0 || j < 0 || i == j)
      {
         assert(false && "Invalid vertices specified");
         return base_type::undefined_cost;
      }

      return static_cast<cost_type>(MatrixElemCst(i, j));
   }

   /**
    * @brief Check if the relation is symmetric relation iRj and jRi (not that the cost is the same)
    * @param[in] i starting vertex
    * @param[in] j end vertex
    * @return true/false
    */
   bool symmetricRelation(int i, int j) const
   {
      if (i >= no_vertices_ || j >= no_vertices_ || i < 0 || j < 0 || i == j)
      {
         assert(false && "Invalid vertices specified");
         return false;
      }

      if (base_type::directed_graph)
      {
         typename base_type::equivalent_cost_cmp equiv;

         if (equiv(MatrixElemCst(i, j), base_type::undefined_cost) ||
             equiv(MatrixElemCst(j, i), base_type::undefined_cost))
         {
            return false;
         }
      }

      return true;
   }

   /**
    * @brief Get the list of links for the current node
    * @param[in] vertex node used for extracting the neighbors
    * @param[out] vertex_list list of neighbors
    */
   void getLinks(int vertex, typename base_type::neighbors_list& vertex_list) const
   {
      if (vertex >= static_cast<int>(no_vertices_) || vertex < 0)
      {
         assert(false && "Invalid vertex specified");
         return;
      }

      typename base_type::equivalent_cost_cmp equiv;
      if (base_type::directed_graph)
      {
         unsigned int st_offset = vertex * no_vertices_;
         unsigned int end_offset = st_offset + no_vertices_;

         for (unsigned int i = st_offset; i < end_offset; ++i)
         {
            if (!equiv(adjacency_matrix_[i], base_type::undefined_cost))
            {
               vertex_list.push_back(i % no_vertices_);
            }
         }
      }
      else
      {
         for (unsigned int i = 0; i < no_vertices_; ++i)
         {
            if (!equiv(MatrixElemCst(vertex, i), base_type::undefined_cost))
            {
               vertex_list.push_back(i);
            }
         }
      }

   }

   /**
    * @brief Get a list with all the links defined
    * @param[out] links_list output list with all the links
    */
   void getAllEdges(std::deque<typename base_type::Edge>& links_list) const
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
               if (hasLink(i, j))
               {
                  crtEdge.end_ = j;
                  crtEdge.edge_cost_ = MatrixElemCst(i, j);
                  links_list.push_back(crtEdge);
               }
            }
         }
      }
      else
      {
         for (unsigned int i = 0; i < no_vertices_; ++i)
         {
            crtEdge.start_ = i;
            for (unsigned int j = i + 1; j < no_vertices_; ++j)
            {
               if (hasLink(i, j))
               {
                  crtEdge.end_ = j;
                  crtEdge.edge_cost_ = MatrixElemCst(i, j);
                  links_list.push_back(crtEdge);
               }
            }
         }
      }
   }

   // - Mutators

   /**
    * @brief Set a node path from vertex i to vertex j (or both vertices if the relation is symmetric)
    * @param[in] i starting vertex
    * @param[in] j ending vertex
    * @param[in] cost cost associated
    * @param[in] symmetric if the link will be created both ways (only relevant for digraphs)
    */
   void addLink(int i, int j, const cost_type& cost, bool symmetric = false)
   {
      if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) ||
          i < 0 || j < 0 || i == j)
      {
         assert(false && "Invalid vertices specified");
         return;
      }

      if (cost < 0)
      {
         assert(false && "Invalid cost (must be a positive value)");
         return;
      }

      MatrixElem(i, j) = cost;
      if (base_type::directed_graph && symmetric)
      {
         MatrixElem(j, i) = cost;
      }
   }

   /**
    * @brief Remove an edge from vertex i to vertex j
    * @param[in] i starting vertex
    * @param[in] j ending vertex
    * @return the cost that was set before clearing the link
    */
   cost_type clearLink(int i, int j)
   {
      if (i >= static_cast<int>(no_vertices_) || j >= static_cast<int>(no_vertices_) ||
          i < 0 || j < 0 || i == j)
      {
         assert(false && "Invalid vertices specified");
         return base_type::undefined_cost;
      }

      cost_type prev_value;
      prev_value = MatrixElem(i, j);
      MatrixElem(i, j) = base_type::undefined_cost;
      return prev_value;
   }

   /**
    * @brief Clear all edges
    */
   void clearAllLinks()
   {
      unsigned int alloc_size = allocSize();
      for (unsigned int i = 0; i < alloc_size; ++i)
      {
         *(adjacency_matrix_ + i) = base_type::undefined_cost;
      }
   }

protected:

   // - Internals

   /**
    * @brief Get the corresponding matrix element
    * @param i row index
    * @param j column index
    * @return a reference to the stored cost
    */
   typename base_type::cost_element_type& MatrixElem(int i, int j)
   {
      unsigned int offset = 0;
      if (!base_type::directed_graph)
      {
         if (i <= j)
         {
            if (i > 0)
            {
               offset = (i * no_vertices_ - (i * (i - 1) / 2));
            }
            offset += (j - i);
         }
         else
         {
            if (j > 0)
            {
               offset = (j * no_vertices_ - (j * (j - 1) / 2));
            }
            offset += (i - j);
         }
      }
      else
      {
         offset = i * no_vertices_ + j;
      }

      return *(adjacency_matrix_ + offset);
   }

   /**
    * @brief Get the corresponding matrix element
    * @param i row index
    * @param j column index
    * @return a const reference to the stored cost
    */
   const typename base_type::cost_element_type& MatrixElemCst(int i, int j) const
   {
      unsigned int offset = 0;
      if (!base_type::directed_graph)
      {
         if (i <= j)
         {
            if (i > 0)
            {
               offset = (i * no_vertices_ - (i * (i - 1) / 2));
            }
            offset += (j - i);
         }
         else
         {
            if (j > 0)
            {
               offset = (j * no_vertices_ - (j * (j - 1) / 2));
            }
            offset += (i - j);
         }
      }
      else
      {
         offset = i * no_vertices_ + j;
      }

      return *(adjacency_matrix_ + offset);
   }

   /**
    * @brief Linear array size
    * @return the number of elements kept
    */
   unsigned int allocSize() const
   {
      if (base_type::directed_graph)
      {
         return (no_vertices_ * no_vertices_);
      }
      return (no_vertices_ * (no_vertices_ + 1) / 2);
   }

   // - Members

   typename base_type::cost_element_type* adjacency_matrix_;       ///< adjancency matrix
   unsigned int no_vertices_;                                        ///< no of vertices
};

/**
 * Graph based on int costs using an adjacency matrix
 */
using dense_graph = graph::GraphContainer<ADJACENCY_MATRIX, UNDIRECTED>;
/**
 * Graph based on int costs using an adjacency list
 */
using sparse_graph = graph::GraphContainer<ADJACENCY_LIST, UNDIRECTED>;
/**
 * Directed Graph based on int costs using an adjacency matrix
 */
using dense_digraph = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;
/**
 * Directed graph based on int costs using an adjacency list
 */
using sparse_digraph = graph::GraphContainer<ADJACENCY_MATRIX, DIRECTED>;

GRAPH_END


#endif // #define _graph_container_h_
