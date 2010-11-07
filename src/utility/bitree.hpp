#ifndef _UTILITY_BITREE_HPP
#define _UTILITY_BITREE_HPP

#include <cassert>
#include <vector>


template <class Grp>
class bitree {
public:

  typedef Grp value_type;
  typedef typename std::vector<value_type>::size_type size_type;
  typedef typename std::vector<value_type>::difference_type difference_type;

private:

  std::vector<value_type> elements;

  value_type accumulate_relative (size_type i, size_type ancestor) const;
  value_type compute_element (size_type i, const value_type& value) const;
  void update_element (size_type i, const value_type& value);

public:

  class reference;
  friend class reference;
  typedef const reference const_reference;

  class reference {
    friend class bitree;

    bitree* container;
    size_type index;

    reference (bitree* c, size_type i) : container(c), index(i) { }

  public:

    operator value_type () const {
      return container->get(index);
    }

    reference& operator= (const value_type& value) {
      container->update_element (index, value);
      return *this;
    }

    reference& operator= (const reference& element) {
      return *this = value_type(element);
    }

  };

  bitree () { }
  bitree (size_type n) : elements(n) { }


  size_type size () const { return elements.size(); }
  size_type capacity () const { return elements.capacity(); }
  size_type max_size () const { return elements.max_size(); }
  bool empty () const { return elements.empty(); }

  reference operator[] (size_type n) { return reference (this, n); }
  const_reference operator[] (size_type n) const { return const_cast<bitree&>(*this)[n]; }

  reference front () { return (*this)[0]; }
  reference back () { return (*this)[size()-1]; }
  const_reference front () const { return *this[0]; }
  const_reference back () const { return *this[size()-1]; }

  void pop_back () { elements.pop_back(); }
  void push_back (const value_type& value) { elements.push_back (compute_element(size(), value)); }

  void resize (size_type n, value_type v = value_type());
  void reserve (size_type n) { elements.reserve(n); }
  void clear () { elements.clear(); }

  void swap (bitree& o) { elements.swap (o.elements); } 

  value_type get (size_type i) const;
  value_type accumulate (size_type begin, size_type end) const;
  value_type accumulate (size_type end) const { return accumulate(0, end); }
  value_type accumulate () const { return accumulate(size()); }
  size_type binary_search (const value_type& value) const;

};


namespace bitree_impl {

  /** Clears the lowest set bit. */
  inline size_t parent (size_t i) {
    assert (i > 0);
    return i & (i - 1);
  }

  /** Isolates the lowest set bit and adds resulting value to input. */
  inline size_t next_sibling (size_t i) {
    assert (i > 0);
    return i + (i & -i);
  }

  /** Returns the immediate child of 'parent' that contains descendant 'i'. */
  inline size_t child_containing (size_t parent, size_t i) {
    assert (i > parent);
    i -= parent;
    i |= i >> 1;
    i |= i >> 2;
    i |= i >> 4;
    i |= i >> 8;
    i |= i >> 16;
    i |= i >> 32;
    i ^= i >> 1;
    i += parent;
    return i;
  }

  /** Returns the closest common ancestor of both a and b. **/
  inline size_t common_ancestor (size_t a, size_t b) {
    b ^= a;
    b |= b >> 1;
    b |= b >> 2;
    b |= b >> 4;
    b |= b >> 8;
    b |= b >> 16;
    b |= b >> 32;
    a &= ~b;
    return a;
  }

}


template <class Grp>
Grp bitree<Grp>::accumulate_relative (size_type i, size_type ancestor) const {
  using namespace bitree_impl;
  assert (i < elements.size());
  value_type result = value_type();
  for (; i != ancestor; i = parent(i)) {
    assert (i > 0);
    result = elements[i] + result;
  }
  return result;
}


template <class Grp>
Grp bitree<Grp>::compute_element (size_type i, const value_type& value) const {
  using namespace bitree_impl;
  assert (i <= elements.size());
  if (i == 0) return value;
  else return accumulate_relative(i-1, parent(i)) + value;
}


template <class Grp>
void bitree<Grp>::update_element (size_type i, const value_type& value) {
  using namespace bitree_impl;
  assert (i < elements.size());
  value_type origin = elements[i];
  elements[i] = compute_element(i, value);
  origin += -elements[i];
  if (i > 0) {
    size_t next = next_sibling(i);
    for (; next < elements.size(); i = next, next = next_sibling(i)) {
      size_t next_parent = parent(next);
      while ((i = parent(i)) != next_parent) {
	origin = elements[i] + origin + (-elements[i]);
      }
      value_type old_value = elements[next];
      elements[next] = -origin + elements[next];
      origin = old_value + (-elements[next]);
    }
  }
}


template <class Grp>
Grp bitree<Grp>::get (size_type i) const {
  using namespace bitree_impl;
  assert (i < elements.size());
  if (i == 0) return elements[0];
  else return -accumulate_relative(i-1, parent(i)) + elements[i];
}


template <class Grp>
void bitree<Grp>::resize (size_type n, value_type v) {
  if (n < elements.size()) {
    elements.resize(n);
  }
  else {
    elements.reserve(n);
    for (size_type i = elements.size(); i < n; ++i) {
      push_back (v);
    }
  }
}


template <class Grp>
Grp bitree<Grp>::accumulate (size_type begin, size_type end) const {
  using namespace bitree_impl;
  assert (begin <= elements.size() && end <= elements.size());
  if (begin == end) {
    return value_type();
  }
  else if (begin == 0) {
    return elements[0] + accumulate_relative(end-1, 0);
  }
  else if (end == 0) {
    return -(elements[0] + accumulate_relative(begin-1, 0));
  }
  else {
    size_type ancestor = common_ancestor(begin-1, end-1);
    return -accumulate_relative(begin-1, ancestor) + accumulate_relative(end-1, ancestor);
  }
}


template <class Grp>
typename bitree<Grp>::size_type bitree<Grp>::binary_search (const value_type& value) const {
  using namespace bitree_impl;
  if (elements.size() == 0 || value < elements[0]) {
    return 0;
  }
  else {
    size_type min = 0;
    size_type max = elements.size();
    value_type origin = elements[min];
    while (min + 1 < max) {
      size_type mid = child_containing(min, max-1);
      if (value < origin + elements[mid]) {
	max = mid;
      }
      else {
	min = mid;
	origin += elements[min];
      }
    }
    return max;
  }
}

#endif //_UTILITY_BITREE_HPP
