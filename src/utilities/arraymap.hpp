#ifndef _UTILITIES_ARRAYMAP_HPP
#define _UTILITIES_ARRAYMAP_HPP

#include <vector>
#include <utility>
#include <algorithm>
#include <functional>

template <class Key, class Mapped, class Cmp = std::less<Key>,
	  class Alloc = std::allocator<std::pair<const Key, Mapped> > >
class arraymap {

public:

  typedef Key    key_type;
  typedef Mapped mapped_type;
  typedef Cmp    key_compare;
  typedef Alloc  allocator_type;

  typedef std::pair<key_type, mapped_type> value_type;

private:
  
  typedef std::vector<value_type, allocator_type> vector_type;
  vector_type entries;
  key_compare cmp;

public:

  class value_compare : public std::binary_function<value_type, value_type, bool> {
    friend class arraymap<key_type, mapped_type, key_compare, allocator_type>;
  protected:
    key_compare cmp;
    value_compare (const key_compare& _cmp) : cmp(_cmp) { }
  public:
    bool operator() (const value_type& x, const value_type& y) const {
      return cmp (x.first, y.first);
    }
  };

  class value_eq : public std::binary_function<value_type, value_type, bool> {
    friend class arraymap<key_type, mapped_type, key_compare, allocator_type>;
  protected:
    value_compare cmp;
    value_eq (const value_compare& _cmp) : cmp(_cmp) { }
  public:
    bool operator() (const value_type& x, const value_type& y) const {
      return !(cmp(x,y) || cmp(y,x));
    }
  };

  typedef typename vector_type::pointer                pointer;
  typedef typename vector_type::const_pointer          const_pointer;
  typedef typename vector_type::reference              reference;
  typedef typename vector_type::const_reference        const_reference;
  typedef typename vector_type::iterator               iterator;
  typedef typename vector_type::const_iterator         const_iterator;
  typedef typename vector_type::size_type              size_type;
  typedef typename vector_type::difference_type        difference_type;
  typedef typename vector_type::reverse_iterator       reverse_iterator;
  typedef typename vector_type::const_reverse_iterator const_reverse_iterator;

public:

  key_compare key_comp () const { return cmp; }
  value_compare value_comp () const { return value_compare(key_comp()); }
  value_eq value_eq () const { return value_eq(value_comp()); }
  allocator_type get_allocator () const { return entries.get_allocator(); }

  iterator begin () { return entries.begin(); }
  iterator end () { return entries.end(); }
  const_iterator begin () const { return entries.begin(); }
  const_iterator end () const { return entries.end(); }
  reverse_iterator rbegin () { return entries.rbegin(); }
  reverse_iterator rend () { return entries.rend(); }
  const_reverse_iterator rbegin () const { return entries.rbegin(); }
  const_reverse_iterator rend () const { return entries.rend(); }

  reference front () { return entries.front(); }
  reference back () { return entries.back(); }
  const_reference front () const { return entries.front(); }
  const_reference back () const { return entries.back(); }

  bool empty () const { return entries.empty(); }
  size_type size () const { return entries.size(); }
  size_type max_size () const { return entries.max_size(); }
  size_type capacity () const { return entries.capacity(); }

  void clear () { entries.clear(); }
  void reserve (size_type n) { entries.reserve(n); }

  void erase (iterator pos) { entries.erase(pos); }
  void erase (iterator first, iterator last) { entries.erase (first, last); }


  // Constructors

  arraymap () { }

  explicit
  arraymap (const key_compare& _cmp, const allocator_type& _alloc = allocator_type())
    : cmp(_cmp), entries(_alloc) { }

  template <class InputIterator>
  arraymap (InputIterator first, InputIterator last) : entries(first, last) {
    std::sort (begin(), end(), value_comp());
    entries.erase (std::unique (begin(), end(), value_eq()), end());
  }

  template <class InputIterator>
  arraymap (InputIterator first, InputIterator last, const key_compare& _cmp,
	    const allocator_type& _alloc = allocator_type())
    : cmp(_cmp), entries(first, last, _alloc) {
    std::sort (begin(), end(), value_comp());
    entries.erase (std::unique (begin(), end(), value_eq()), end());
  }

  // Search functions

  iterator lower_bound (const key_type& key) {
    return std::lower_bound (begin(), end(), value_type(key, mapped_type()), value_comp());
  }

  const_iterator lower_bound (const key_type& key) const {
    return std::lower_bound (begin(), end(), value_type(key, mapped_type()), value_comp());
  }

  iterator upper_bound (const key_type& key) {
    return std::upper_bound (begin(), end(), value_type(key, mapped_type()), value_comp());
  }

  const_iterator upper_bound (const key_type& key) const {
    return std::upper_bound (begin(), end(), value_type(key, mapped_type()), value_comp());
  }

  std::pair<iterator, iterator> equal_range (const key_type& key) {
    return std::equal_range (begin(), end(), value_type(key, mapped_type()), value_comp());
  }

  std::pair<const_iterator, const_iterator> equal_range (const key_type& key) const {
    return std::equal_range (begin(), end(), value_type(key, mapped_type()), value_comp());
  }

  iterator find (const key_type& key) {
    iterator i = lower_bound(key);
    if (i == end() || key_comp()(key, i->first)) return end();
    else return i;
  }

  const_iterator find (const key_type& key) const {
    const_iterator i = lower_bound(key);
    if (i == end() || key_comp()(key, i->first)) return end();
    else return i;
  }

  size_type count (const key_type& key) const {
    std::pair<const_iterator, const_iterator> range = equal_range(key);
    return range.second - range.first;
  }


  // Mutation functions

  iterator insert (const value_type& entry) {
    iterator i = std::lower_bound(entries.begin(), entries.end(), entry, value_comp());
    if (i == end() || value_comp()(entry, *i)) {
      return entries.insert(i, entry);
    }
    else {
      i->second = entry.second;
      return i;
    }
  }

  iterator insert (iterator i, const value_type& entry) {
    if ((i == end() || value_comp()(entry, *i)) &&
	(i == begin() || value_comp()(*(i-1), entry))) {
      return entries.insert(i, entry);
    }
    else if (i != end() && value_eq()(entry, *i)) {
      i->second = entry.second;
      return i;
    }
    else {
      return insert(entry);
    }
  }

  template <class InputIterator>
  void insert (InputIterator first, InputIterator last) {
    entries.insert(end(), first, last);
    std::sort(begin(), end(), value_comp());
    entries.erase (std::unique (begin(), end(), value_eq()), end());
  }

  size_type erase (const key_type& key) {
    std::pair<iterator, iterator> range = equal_range(key);
    size_type range_size = range.second - range.first;
    erase (range.first, range.second);
    return range_size;
  }


  // Miscellaneous utility functions

  void swap (arraymap& other) {
    std::swap (entries, other.entries);
    std::swap (cmp, other.cmp);
  }

  mapped_type& operator[] (const key_type& key) {
    iterator i = lower_bound(key);
    if (i == end() || key_comp()(key, i->first)) {
      i = entries.insert(i, value_type(key, mapped_type()));
    }
    return i->second;
  }

};

#endif //_UTILITIES_ARRAYMAP_HPP
