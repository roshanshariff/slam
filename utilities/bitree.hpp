#ifndef _UTILITIES_BITREE_HPP
#define _UTILITIES_BITREE_HPP

#include <cassert>
#include <vector>


template <class Grp>
class bitree {
public:

  std::vector<Grp> elements;

  Grp accumulate_relative (size_t i, size_t ancestor) const;
  Grp compute_element (size_t i, const Grp& value) const;

public:

  bitree () { }
  bitree (size_t n) : elements(n) { }

  Grp get (size_t i) const;
  size_t binary_search (const Grp& value) const;

  Grp accumulate (size_t n) const;
  Grp accumulate (size_t begin, size_t end) const;
  Grp accumulate () const { return accumulate(size()); }

  void set (size_t i, const Grp& value);
  void push_back (const Grp& value);

  bool empty () const { return elements.empty(); }
  size_t size () const { return elements.size(); }
  size_t capacity () const { return elements.capacity(); }
  size_t max_size () const { return elements.max_size(); }

  void clear () { elements.clear(); }
  void pop_back () { elements.pop_back(); }
  void reserve (size_t n) { elements.reserve(n); }
  void resize (size_t n);

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

}


template <class Grp>
Grp bitree<Grp>::accumulate_relative (size_t i, size_t ancestor) const {
  using namespace bitree_impl;
  assert (i < elements.size());
  Grp result = Grp();
  for (; i != ancestor; i = parent(i)) {
    assert (i > 0);
    result = elements[i] + result;
  }
  return result;
}


template <class Grp>
Grp bitree<Grp>::compute_element (size_t i, const Grp& value) const {
  using namespace bitree_impl;
  assert (i <= elements.size());
  if (i == 0) return value;
  else return accumulate_relative(i-1, parent(i)) + value;
}


template <class Grp>
Grp bitree<Grp>::get (size_t i) const {
  using namespace bitree_impl;
  assert (i < elements.size());
  if (i == 0) return elements[0];
  else return -accumulate_relative(i-1, parent(i)) + elements[i];
}


template <class Grp>
void bitree<Grp>::set (size_t i, const Grp& value) {
  using namespace bitree_impl;
  assert (i < elements.size());
  Grp origin = elements[i];
  elements[i] = compute_element(i, value);
  origin += -elements[i];
  if (i > 0) {
    size_t next = next_sibling(i);
    for (; next < elements.size(); i = next, next = next_sibling(i)) {
      size_t next_parent = parent(next);
      while ((i = parent(i)) != next_parent) {
	origin = elements[i] + origin + (-elements[i]);
      }
      Grp old_value = elements[next];
      elements[next] = -origin + elements[next];
      origin = old_value + (-elements[next]);
    }
  }
}


template <class Grp>
void bitree<Grp>::push_back (const Grp& value) {
  elements.push_back (compute_element(elements.size(), value));
}


template <class Grp>
void bitree<Grp>::resize (size_t n) {
  if (n < elements.size()) {
    elements.resize(n);
  }
  else {
    elements.reserve(n);
    for (size_t i = elements.size(); i < n; ++i) {
      push_back (Grp());
    }
  }
}


template <class Grp>
Grp bitree<Grp>::accumulate (size_t n) const {
  using namespace bitree_impl;
  assert (n <= elements.size());
  if (n == 0) return Grp();
  else return elements[0] + accumulate_relative(n-1, 0);
}


template <class Grp>
Grp bitree<Grp>::accumulate (size_t begin, size_t end) const {
  // TODO: make this more efficient
  return -accumulate(begin) + accumulate(end);
}


template <class Grp>
size_t bitree<Grp>::binary_search (const Grp& value) const {
  using namespace bitree_impl;
  if (elements.size() == 0 || value < elements[0]) {
    return 0;
  }
  else {
    size_t min = 0;
    size_t max = elements.size();
    Grp origin = elements[min];
    while (min + 1 < max) {
      size_t mid = child_containing(min, max-1);
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

#endif //_UTILITIES_BITREE_HPP
