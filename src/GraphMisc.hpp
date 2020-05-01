#ifndef _graph_misc_h
#define _graph_misc_h

#include "GraphBase.hpp"

GRAPH_START
COM_START

template
<class elem_type>
class SquareMatrix
{
public:

  typedef elem_type element_type;

  /// Ctor
  explicit SquareMatrix(std::size_t mat_size, element_type initial_value = 0) : 
    no_elements_(mat_size), 
    storage_(new element_type[mat_size * mat_size])
  {
    for (std::size_t i = 0; i < allocSize(); ++i)
      *(storage_ + i) = initial_value;
  }

  /// Copy ctor
  SquareMatrix(const SquareMatrix& RHS) : 
    no_elements_(RHS.no_elements_), 
    storage_(new element_type[RHS.no_elements_ * RHS.no_elements_])
  {
  }

#ifndef _MOVE_SEMANTICS_OFF
  /// Move ctor
  SquareMatrix(SquareMatrix&& RHS) : 
    no_elements_(RHS.no_elements_), 
    storage_(RHS.storage_)
  {
    RHS.storage_ = nullptr;
  }
#endif

  /// Dtor
  ~SquareMatrix()
  {
    if (storage_)
      delete[] storage_;
  }

  // - Operators

  /// Copy assignment
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
        *(storage_ + i) = *(RHS.storage_ + i);
    }

    return *this;
  }

#ifndef _MOVE_SEMANTICS_OFF
  /// Move assignment
  SquareMatrix& operator = (SquareMatrix&& RHS)
  {
    if (this != &RHS)
    {
      no_elements_ = RHS.no_elements_;

      if (storage_)
        delete[] storage_;
      storage_ = RHS.storage_;
      RHS.storage_ = nullptr;
    }
  }
#endif

  // - Accessors

  /// AllocSize
  std::size_t allocSize() const
  {
    return no_elements_ * no_elements_;
  }

  /// Nb rows/cols
  std::size_t getSize() const
  {
    return no_elements_;
  }

  /// Access an element
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

  /// Reset the size (all the existing content will be lost)
  void setSize(std::size_t new_size, element_type reset_val = 0)
  {
    if (new_size != no_elements_)
    {
      no_elements_ = new_size;
      delete[] storage_;
      storage_ = new element_type[allocSize()];
    }

    for (std::size_t i = 0; i < allocSize(); ++i)
      *(storage_ + i) = reset_val;
  }

  /// Access an element
  element_type& accessElement(std::size_t i, std::size_t j)
  {
    if (i >= no_elements_ || j >= no_elements_)
    {
      assert(false && "Invalid element index");
      return *storage_;
    }

    return *(storage_ + i * no_elements_ + j);
  }

  /// Modify/Retrieve an element
  element_type& operator () (std::size_t i, std::size_t j)
  {
    return accessElement(i, j);
  }

protected:
  element_type* storage_;
  std::size_t no_elements_;
};




COM_END
GRAPH_END

#endif
