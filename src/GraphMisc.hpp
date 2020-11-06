#ifndef _graph_misc_h
#define _graph_misc_h

#include "GraphBase.hpp"
#include "type_traits"
#include "utility"

GRAPH_START
COM_START

/**
 * @brief Union find data structure
 * @tparam elem_type the type of the elements stored [default int]
 */
template
<typename elem_type = int>
class UnionFind
{
   static_assert(std::is_integral<elem_type>::value, "Template argument must be an integer");

public:

   using element_type = elem_type;
   using elem_set_id_found = std::pair<element_type, bool>;

   /**
    * @brief UnionFind
    * @param nb_elements the number of elements considered
    */
   explicit UnionFind(std::size_t nb_elements):
      no_elements_(nb_elements),
      id_elements_(new element_type[no_elements_]),
      no_disjoint_sets_(nb_elements),
      tree_sizes_(new std::size_t[no_elements_])
   {
      for (auto i = 0; i < no_elements_; ++i)
      {
         id_elements_[i] = i;
         tree_sizes_[i] = 1;
      }
   }

   /**
    * @brief Copy ctor
    * @param RHS the existing element
    */
   UnionFind(const UnionFind& RHS) :
      no_elements_(RHS.no_elements_),
      id_elements_(new element_type[RHS.no_elements_]),
      no_disjoint_sets_(RHS.no_disjoint_sets_),
      tree_sizes_(new std::size_t[RHS.no_elements_])
   {
      for (auto i = 0; i < no_elements_; ++i)
      {
         id_elements_[i] = RHS.id_elements_[i];
         tree_sizes_[i] = RHS.tree_sizes_[i];
      }
   }

#ifndef _MOVE_SEMANTICS_OFF
   /**
    * @brief Move ctor
    * @param[in] RHS element
    */
   UnionFind(UnionFind&& RHS) :
      no_elements_(RHS.no_elements_),
      id_elements_(RHS.id_elements_),
      no_disjoint_sets_(RHS.no_disjoint_sets_),
      tree_sizes_(RHS.tree_sizes_)
   {
      RHS.id_elements_ = nullptr;
      RHS.tree_sizes_ = nullptr;
   }
#endif

  /**
   * @brief Dtor
   */
   ~UnionFind()
   {
      if (id_elements_)
      {
         delete[] id_elements_;
      }

      if (tree_sizes_)
      {
         delete[] tree_sizes_;
      }
   }

   // - Operators

   /**
    * @brief Copy assignment
    * @param[in] RHS element
    * @return current element
    */
   UnionFind& operator = (const UnionFind& RHS)
   {
      if (this != &RHS)
      {
         if (id_elements_ != nullptr)
         {
            delete[] id_elements_;
         }

         if (tree_sizes_ != nullptr)
         {
            delete[] tree_sizes_;
         }

         no_elements_ = RHS.no_elements_;
         id_elements_ = new element_type[no_elements_];
         tree_sizes_  = new std::size_t[no_elements_];
         no_disjoint_sets_ = RHS.no_disjoint_sets_;

         for (auto i = 0; i < no_elements_; ++i)
         {
            id_elements_[i] = RHS.id_elements_[i];
            tree_sizes_[i] = RHS.tree_sizes_[i];
         }
      }
      return *this;
   }


#ifndef _MOVE_SEMANTICS_OFF
   /**
    * @brief Move assignment
    * @param[in] RHS element
    * @return current element
    */
   UnionFind& operator = (UnionFind&& RHS)
   {
      if (this != &RHS)
      {
         no_elements_ = RHS.no_elements_;

         if (id_elements_)
         {
            delete[] id_elements_;
         }

         if (tree_sizes_)
         {
            delete[] tree_sizes_;
         }

         id_elements_ = RHS.id_elements_;
         RHS.id_elements_ = nullptr;
         tree_sizes_ = RHS.tree_sizes_;
         RHS.tree_sizes_ = nullptr;

         no_disjoint_sets_ = RHS.no_disjoint_sets_;
      }
      return *this;
   }
#endif

   // - Accessors

   /**
    * @brief Returns the set id for the specified element
    * @param elem_x
    * @return the set id where elem_x is included as the first element and a flag if the set id has been found or not
    */
   elem_set_id_found findSetId(element_type elem_x)
   {
      elem_set_id_found ret_value(elem_x, true);

      if (elem_x >= no_elements_)
      {
         assert(false && "Invalid element specified");
         ret_value.second = false;
         return ret_value;
      }

      auto crt_root = elem_x;
      while (crt_root != id_elements_[crt_root])
      {
         crt_root = id_elements_[crt_root];
      }

      ret_value.first = crt_root;
      return ret_value;
   }

   /**
    * @brief Esentially checks if the 2 elements belong to the same set (are connected)
    * @param elem_a first element
    * @param elem_b second element
    * @return true if the sets have been previously united or false otherwise
    */
   bool connected(element_type elem_a, element_type elem_b)
   {
      auto set_a = findSetId(elem_a);
      auto set_b = findSetId(elem_b);
      if (!(set_a.second && set_b.second))
      {
         return false;
      }

      return (set_a.first == set_b.first);
   }

   /**
    * @brief Returns the number of disjoint sets
    * @return number of current sets
    */
   std::size_t noDisjointSets()
   {
      return no_disjoint_sets_;
   }

   // - Mutators

   /**
    * @brief Attempts to unite the sets which contain the 2 elements specified
    * @param elem_a first element
    * @param elem_b second element
    * @return true if the sets have been united or false otherwise
    */
   bool makeUnion(element_type elem_a, element_type elem_b)
   {
      auto set_id_a_pair = findSetId(elem_a);
      auto set_id_b_pair = findSetId(elem_b);

      auto set_id_a = set_id_a_pair.first;
      auto set_id_b = set_id_b_pair.first;

      // part of the same set, no union required
      if (set_id_a == set_id_b)
      {
         return false;
      }

      if (tree_sizes_[set_id_a] < tree_sizes_[set_id_b])
      {
         id_elements_[set_id_a] = id_elements_[set_id_b];
         tree_sizes_[set_id_b] += tree_sizes_[set_id_a];
      }
      else
      {
         id_elements_[set_id_b] = id_elements_[set_id_a];
         tree_sizes_[set_id_a] += tree_sizes_[set_id_b];
      }

      // decrease the number of sets (individual trees from the forest)
      --no_disjoint_sets_;

      return true;
   }

protected:

   std::size_t no_elements_;         ///< number of elements stored
   std::size_t no_disjoint_sets_;    ///< number of disjoint sets (decreases as sets are unified)
   elem_type* id_elements_;          ///< array of elements
   std::size_t* tree_sizes_;         ///< array that keeps the current size of a tree from the forest
};

/**
 * @brief Square matrix class
 * @tparam elem_type the type of the elements stored
 */
template
<typename elem_type>
class SquareMatrix
{
public:

   using element_type = elem_type;

   /**
    * @brief Ctor
    * @param[in] mat_size matrix size
    * @param[in] initial_value used for filling the matrix
    */
   explicit SquareMatrix(std::size_t mat_size, element_type initial_value = 0) :
      no_elements_(mat_size),
      storage_(new element_type[mat_size * mat_size])
   {
      for (std::size_t i = 0; i < allocSize(); ++i)
      {
         *(storage_ + i) = initial_value;
      }
   }

   /**
    * @brief Copy ctor
    * @param[in] RHS element
    */
   SquareMatrix(const SquareMatrix& RHS) :
      no_elements_(RHS.no_elements_),
      storage_(new element_type[RHS.no_elements_ * RHS.no_elements_])
   {
      for (auto i = 0; i < allocSize(); ++i)
      {
         *(storage_ + i) = *(RHS.storage_ + i);
      }
   }

#ifndef _MOVE_SEMANTICS_OFF
   /**
    * @brief Move ctor
    * @param[in] RHS element
    */
   SquareMatrix(SquareMatrix&& RHS) :
      no_elements_(RHS.no_elements_),
      storage_(RHS.storage_)
   {
      RHS.storage_ = nullptr;
   }
#endif

   /**
    * @brief Dtor
    */
   ~SquareMatrix()
   {
      if (storage_)
      {
         delete[] storage_;
      }
   }

   // - Operators

   /**
    * @brief Copy assignment
    * @param[in] RHS element
    * @return current element
    */
   SquareMatrix& operator = (const SquareMatrix& RHS)
   {
      if (this != &RHS)
      {
         if (no_elements_ != RHS.no_elements_)
         {
            no_elements_ = RHS.no_elements_;
            delete[] storage_;
            storage_ = new element_type[allocSize()];
         }

         for (auto i = 0; i < allocSize(); ++i)
         {
            *(storage_ + i) = *(RHS.storage_ + i);
         }
      }

      return *this;
   }

#ifndef _MOVE_SEMANTICS_OFF
   /**
    * @brief Move assignment
    * @param[in] RHS element
    * @return current element
    */
   SquareMatrix& operator = (SquareMatrix&& RHS)
   {
      if (this != &RHS)
      {
         no_elements_ = RHS.no_elements_;

         if (storage_)
         {
            delete[] storage_;
         }
         storage_ = RHS.storage_;
         RHS.storage_ = nullptr;
      }
      return *this;
   }
#endif

   // - Accessors

   /**
    * @brief The total number of elements allocated
    * @return nb elements
    */
   std::size_t allocSize() const
   {
      return no_elements_ * no_elements_;
   }

   /**
    * @brief Get the number of rows/columns
    * @return the size
    */
   std::size_t getSize() const
   {
      return no_elements_;
   }

   /**
    * @brief Get a const reference to a stored element
    * @param[in] i row index
    * @param[in] j column index
    * @return reference to the matrix element
    */
   const element_type& accessElementCst(std::size_t i, std::size_t j) const
   {
      if (i >= no_elements_ || j >= no_elements_)
      {
         assert(false && "Invalid element index");
         return *storage_;
      }

      return *(storage_ + i * no_elements_ + j);
   }

   // - Mutators

   /**
    * @brief Reset the size (all the existing content will be lost)
    * @param new_size new capacity
    * @param reset_val value used for filling
    */
   void setSize(std::size_t new_size, element_type reset_val = 0)
   {
      if (new_size != no_elements_)
      {
         no_elements_ = new_size;
         delete[] storage_;
         storage_ = new element_type[allocSize()];
      }

      for (std::size_t i = 0; i < allocSize(); ++i)
      {
         *(storage_ + i) = reset_val;
      }
   }

   /**
    * @brief Get a reference to a stored element
    * @param[in] i row index
    * @param[in] j column index
    * @return reference to the matrix element
    */
   element_type& accessElement(std::size_t i, std::size_t j)
   {
      if (i >= no_elements_ || j >= no_elements_)
      {
         assert(false && "Invalid element index");
         return *storage_;
      }

      return *(storage_ + i * no_elements_ + j);
   }

   /**
    * @brief Modify/Retrieve an element
    * @param[in] i row index
    * @param[in] j column index
    * @return reference to the element
    */
   element_type& operator () (std::size_t i, std::size_t j)
   {
      return accessElement(i, j);
   }

protected:

   element_type* storage_;          ///< stored elements
   std::size_t no_elements_;        ///< nb of elements
};

COM_END
GRAPH_END

#endif
