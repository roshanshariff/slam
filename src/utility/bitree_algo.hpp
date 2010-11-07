#ifndef _UTILITY_BITREE_ALGO_HPP
#define _UTILITY_BITREE_ALGO_HPP

#include <iterator>

#include <boost/utility.hpp>

namespace bitree_algorithms {

  using boost::prior;
  using boost::next;


  template <class Iter>
  inline Iter parent_element (Iter begin, Iter element) {
    typedef std::iterator_traits<Iter>::difference_type difference_type;
    difference_type i = std::distance (begin, element);
    assert (i > 0);
    std::advance (begin, i & (i - 1));
    return begin;
  }


  template <class Iter>
  inline Iter next_sibling (Iter begin, Iter element) {
    typedef std::iterator_traits<Iter>::difference_type difference_type;
    difference_type i = std::distance (begin, element);
    assert (i > 0);
    std::advance (begin, i + (i & -i));
    return begin;
  }


  template <class Iter>
  inline Iter child_element (Iter element, Iter descendant) {
    typedef std::iterator_traits<Iter>::difference_type difference_type;
    difference_type i = std::distance (element, descendant);
    assert (i > 0);
    i |= i >> 1;
    i |= i >> 2;
    i |= i >> 4;
    i |= i >> 8;
    i |= i >> 16;
    i |= i >> 32;
    i ^= i >> 1;
    std::advance (element, i);
    return element;
  }


  template <class Iter>
  inline Iter common_ancestor (Iter begin, Iter first, Iter second) {
    typedef std::iterator_traits<Iter>::difference_type difference_type;
    difference_type a = std::distance (begin, first);
    difference_type b = std::distance (begin, second);
    assert (a >= 0 && b >= 0);
    b ^= a;
    b |= b >> 1;
    b |= b >> 2;
    b |= b >> 4;
    b |= b >> 8;
    b |= b >> 16;
    b |= b >> 32;
    a &= ~b;
    std::advance (begin, a);
    return begin;
  }


  template <class Iter, class MonoidOperation>
  typename MonoidOperation::result_type
  accumulate_relative (Iter begin, Iter ancestor, Iter element, MonoidOperation op) {
    typename MonoidOperation::result_type result = identity_element (op);
    while (element != ancestor) {
      assert (element != begin);
      result = op (*element, result);
      element = parent_element (begin, element)) {
    }
    return result;
  }


  template <class Iter, class MonoidOperation, class InverseOperation>
  typename MonoidOperation::result_type
  accumulate (Iter begin, Iter start, Iter end, MonoidOperation op, InverseOperation inv) {
    if (start == end) {
      return identity_element (op);
    }
    else if (start == begin) {
      return op (*begin, accumulate_relative (begin, start, prior (end), op));
    }
    else if (end == begin) {
      return inv (op (*begin, accumulate_relative (begin, end, prior (start), op)));
    }
    else {
      Iter ancestor = common_ancestor (begin, prior (start), prior (end));
      return op (inv (accumulate_relative (begin, ancestor, prior (start), op)),
		 accumulate_relative (begin, ancestor, prior (end), op));
    }
  }


  template <class Iter, class MonoidOperation, class InverseOperation>
  inline typename MonoidOperation::result_type
  accumulate (Iter begin, Iter end, MonoidOperation op, InverseOperation inv) {
    return accumulate (begin, begin, end, op, inv);
  }
    

  template <class Iter, class MonoidOperation>
  inline typename MonoidOperation::result_type
  transform_value (Iter begin, Iter position, MonoidOperation op,
		   const typename MonoidOperation::result_type& value) {
    if (position == begin) {
      return value;
    }
    else {
      Iter parent = parent_element (begin, position);
      return op (accumulate_relative (begin, parent, prior (position), op), value);
    }
  }


  template <class Iter, class MonoidOperation>
  inline void push_value (Iter begin, Iter end, MonoidOperation op) {
    *prior(end) = transform_value (begin, end, op, *prior(end));
  }


  template <class Iter, class MonoidOperation, class InverseOperation>
  inline typename MonoidOperation::result_type
  compute_value (Iter begin, Iter element, MonoidOperation op, InverseOperation inv) {
    if (element == begin) {
      return *element;
    }
    else {
      Iter parent = parent_element (begin, element);
      return op (inv (accumulate_relative (begin, parent, prior (element), op)), *element);
    }
  }



  template <class Iter, class MonoidOperation, class InverseOperation>
  void update_element (Iter begin, Iter element, Iter end,
		       MonoidOperation op, InverseOperation inv, 
		       const typename MonoidOperation::result_type& value) {
    typedef typename MonoidOperation::result_type& result_type;
    result_type inv_delta = *element;
    *element = transform_value (begin, element, op, value);
    inv_delta = op (inv_delta, inv (*element));
    if (element != begin) {
      Iter next = next_sibling (begin, element);
      while (next < end) {
	Iter next_parent = parent (begin, next);
	while ((element = parent (begin, element)) != next_parent) {
	  inv_delta = op (op (*element, inv_delta), inv (*element));
	}
	result_type old_value = *next;
	*next = op (inv (inv_delta), *next);
	inv_delta = op (old_value, inv (*next));
	element = next;
	next = next_sibling (begin, element)) {
      }
    }
  }


  template <class Iter, class MonoidOperation, class Ordering>
  Iter binary_search (Iter begin, Iter end, MonoidOperation op, Ordering comp,
		      const typename MonoidOperation::result_type& value) {
    if (begin == end || comp (value, *begin)) {
      return begin;
    }
    else {
      typename MonoidOperation::result_type delta = *begin;
      while (begin < prior (end)) {
	Iter middle = child_containing (begin, prior (end));
	if (comp (value, op (delta, *middle))) {
	  end = middle;
	}
	else {
	  begin = middle;
	  delta = op (delta, *begin);
	}
      }
      return end;
    }
  }


} // namespace bitree_algorithms

#endif //_UTILITY_BITREE_ALGO_HPP
