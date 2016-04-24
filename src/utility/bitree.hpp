#ifndef _UTILITY_BITREE_HPP
#define _UTILITY_BITREE_HPP

#include <cassert>
#include <cstddef>
#include <vector>

#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include "utility/container_fwd.hpp"

namespace bitree_impl {

/** Finds the parent of a node by clearing the lowest set bit.
 ** parent(0) == 0 */
inline std::size_t parent(std::size_t i) {
  return i & (i - 1);
}

/** Finds the next node in a depth-first traversal of the tree that is
 *not a descendant or * ancestor. Isolates the lowest set bit and adds
 *it to the index.  next_sibling(0) == 0 */
inline std::size_t next_sibling(std::size_t i) {
  return i + (i & -i);
}

/** Copies highest set bit in 64-bit integer to all lower bits */
inline std::size_t suffix_mask(std::size_t i) {
  i |= i >> 1;
  i |= i >> 2;
  i |= i >> 4;
  i |= i >> 8;
  i |= i >> 16;
  i |= i >> 32;
  return i;
}

/** Returns the closest common ancestor of both a and b. **/
inline std::size_t common_ancestor(std::size_t a, std::size_t b) {
  return a & ~(suffix_mask(a ^ b));
}

/** Returns the immediate child of 'parent' that contains descendant 'i'. */
inline std::size_t child_containing(std::size_t parent, std::size_t i) {
  assert(parent < i && (parent == 0 || i < next_sibling(parent)));
  return i & ~(suffix_mask(parent ^ i) >> 1);
}
}

namespace utility {

template <typename Grp, typename Alloc>
class bitree {

public:
  using value_type = Grp;
  using allocator_type = Alloc;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  struct const_iterator;
  struct iterator;

  struct reference;

  struct const_reference {
    friend class bitree;
    friend struct reference;
    friend struct const_iterator;
    operator value_type() const { return container->get(index); }
    const_reference& operator=(const const_reference&) = delete;

  private:
    const bitree* container;
    size_type index;
    const_reference(const bitree& c, size_type i) : container(&c), index(i) {}
  };

  struct reference : public const_reference {
    friend class bitree;
    friend struct const_iterator;
    const reference& operator=(const value_type& v) const {
      const_cast<bitree*>(const_reference::container)
          ->set(const_reference::index, v);
      return *this;
    }

  private:
    reference(bitree& c, size_type i) : const_reference(c, i) {}
  };

private:
  using const_iterator_base =
      boost::iterator_facade<const_iterator, value_type,
                             boost::random_access_traversal_tag,
                             const_reference>;

  using iterator_base =
      boost::iterator_adaptor<iterator, const_iterator, boost::use_default,
                              boost::use_default, reference>;

public:
  struct const_iterator : public const_iterator_base {

    friend class boost::iterator_core_access;
    friend class bitree;
    friend struct iterator;

    const_iterator& operator=(const const_iterator& ci) {
      ref.container = ci.ref.container;
      ref.index = ci.ref.index;
    }

  private:
    reference ref;

    explicit const_iterator(const const_reference& cref)
        : ref(const_cast<bitree&>(*cref.container), cref.index) {}

    const const_reference& dereference() const { return ref; }

    void increment() { ++ref.index; }
    void decrement() { --ref.index; }
    void advance(difference_type n) { ref.index += n; }

    difference_type distance_to(const const_iterator& i) const {
      return i.ref.index - ref.index;
    }

    bool equal(const const_iterator& i) const {
      return ref.container == i.ref.container && ref.index == i.ref.index;
    }
  };

  struct iterator : public iterator_base {

    friend class boost::iterator_core_access;
    friend class bitree;

    operator const const_iterator&() const {
      return iterator_base::base_reference();
    }
    operator const_iterator&() { return iterator_base::base_reference(); }

  private:
    explicit iterator(const reference& ref)
        : iterator_base(const_iterator(ref)) {}

    const reference& dereference() const {
      return iterator_base::base_reference().ref;
    }
  };

  bitree() = default;
  explicit bitree(size_type n) : elements(n) {}

  size_type size() const { return elements.size(); }
  size_type capacity() const { return elements.capacity(); }
  size_type max_size() const { return elements.max_size(); }
  bool empty() const { return elements.empty(); }

  reference operator[](size_type i) { return reference(*this, i); }
  const_reference operator[](size_type i) const {
    return const_reference(*this, i);
  }

  reference front() { return (*this)[0]; }
  const_reference front() const { return *this[0]; }

  reference back() { return (*this)[size() - 1]; }
  const_reference back() const { return *this[size() - 1]; }

  iterator begin() { return iterator((*this)[0]); }
  const_iterator begin() const { return const_iterator((*this)[0]); }

  iterator end() { return iterator((*this)[size()]); }
  const_iterator end() const { return const_iterator((*this)[size()]); }

  void pop_back() { elements.pop_back(); }

  void push_back(const value_type& value) {
    elements.push_back(parent_relative_value(size(), value));
  }

  void push_back_accumulated(const value_type& value);

  void resize(size_type n, const value_type& v = value_type());
  void reserve(size_type n) { elements.reserve(n); }
  void clear() { elements.clear(); }

  void swap(bitree& o) { elements.swap(o.elements); }

  value_type accumulate(size_type begin, size_type end) const;
  value_type accumulate(size_type end) const { return accumulate(0, end); }
  value_type accumulate() const { return accumulate(size()); }
  size_type binary_search(const value_type& value) const;

private:
  std::vector<value_type, allocator_type> elements;

  value_type accumulate_relative(size_type i, size_type ancestor) const;
  void update(size_type i, value_type new_value);
  value_type get(size_type i) const;

  value_type parent_relative_value(size_type i, const value_type& value) const {
    using namespace bitree_impl;
    if (i == 0)
      return value;
    else
      return accumulate_relative(i - 1, parent(i)) + value;
  }

  void set(size_type i, const value_type& value) {
    update(i, parent_relative_value(i, value));
  }
};

template <typename Grp, typename Alloc>
auto bitree<Grp, Alloc>::accumulate_relative(size_type i,
                                             size_type ancestor) const
    -> value_type {
  using namespace bitree_impl;
  assert(i < elements.size());
  value_type result = value_type();
  for (; i != ancestor; i = parent(i)) {
    assert(i > 0);
    result = elements[i] + result;
  }
  return result;
}

template <typename Grp, typename Alloc>
void bitree<Grp, Alloc>::update(size_type i, value_type new_value) {

  using namespace bitree_impl;
  assert(i < elements.size());

  if (i > 0)
    for (std::size_t next; (next = next_sibling(i)) < size(); i = next) {

      value_type delta = new_value + (-elements[i]);
      elements[i] = new_value;

      const std::size_t next_parent = parent(next);
      while ((i = parent(i)) != next_parent) {
        delta = elements[i] + delta + (-elements[i]);
      }

      new_value = delta + elements[next];
    }

  elements[i] = new_value;
}

template <typename Grp, typename Alloc>
auto bitree<Grp, Alloc>::get(size_type i) const -> value_type {
  using namespace bitree_impl;
  assert(i < elements.size());
  if (i == 0)
    return elements[0];
  else
    return -accumulate_relative(i - 1, parent(i)) + elements[i];
}

template <typename Grp, typename Alloc>
void bitree<Grp, Alloc>::resize(size_type n, const value_type& v) {
  if (n < size())
    elements.resize(n);
  else
    while (elements.size() < n) elements.push_back(v);
}

template <typename Grp, typename Alloc>
auto bitree<Grp, Alloc>::accumulate(size_type begin, size_type end) const
    -> value_type {
  using namespace bitree_impl;
  assert(begin <= elements.size() && end <= elements.size());
  if (begin == end) { return value_type(); }
  else if (begin == 0) {
    return elements[0] + accumulate_relative(end - 1, 0);
  }
  else if (end == 0) {
    return -(elements[0] + accumulate_relative(begin - 1, 0));
  }
  else {
    size_type ancestor = common_ancestor(begin - 1, end - 1);
    return -accumulate_relative(begin - 1, ancestor)
           + accumulate_relative(end - 1, ancestor);
  }
}

template <typename Grp, typename Alloc>
void bitree<Grp, Alloc>::push_back_accumulated(const value_type& value) {
  using namespace bitree_impl;
  if (empty())
    elements.push_back(value);
  else
    elements.push_back(-accumulate(parent(size()) + 1) + value);
}

template <typename Grp, typename Alloc>
auto bitree<Grp, Alloc>::binary_search(const value_type& value) const
    -> size_type {
  using namespace bitree_impl;
  if (elements.size() == 0 || value < elements[0]) { return 0; }
  else {
    size_type min = 0;
    size_type max = elements.size();
    value_type origin = elements[min];
    while (min + 1 < max) {
      size_type mid = child_containing(min, max - 1);
      if (value < origin + elements[mid]) { max = mid; }
      else {
        min = mid;
        origin += elements[min];
      }
    }
    return max;
  }
}

} // namespace utility

#endif //_UTILITY_BITREE_HPP
