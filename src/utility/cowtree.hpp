//
//  cowtree.hpp
//  slam
//
//  Created by Roshan Shariff on 11-09-06.
//  Copyright 2011 University of Alberta. All rights reserved.
//

#ifndef slam_cowtree_hpp
#define slam_cowtree_hpp

#include <cassert>
#include <memory>
#include <utility>

class cowtree {

public:
  bool empty() const { return node_ptr.get() == nullptr; }

  const cowtree& left() const;
  const cowtree& right() const;
  template <class T>
  const T& value() const;

  class root;
  class editor;

private:
  cowtree() = default;
  cowtree(const cowtree& o) = default;
  cowtree(cowtree&& o) = default;
  cowtree& operator=(const cowtree& o) = default;
  cowtree& operator=(cowtree&& o) = default;

  void swap(cowtree& o) noexcept { node_ptr.swap(o.node_ptr); }

  void clear() { node_ptr.reset(); }

  class node;

  const node& get_node() const {
    assert(!empty());
    return *node_ptr;
  }
  node& get_node() {
    assert(!empty());
    return *node_ptr;
  }

  cowtree& left();
  cowtree& right();
  template <class T>
  T& value();

  bool is_black() const;
  void set_black(bool black);
  void make_unique();

  template <class T>
  static std::shared_ptr<node> make_node(const T& value);

  std::shared_ptr<node> node_ptr;
};

class cowtree::root : public cowtree {
public:
  using cowtree::clear;
  using cowtree::swap;
  friend void swap(root& a, root& b) noexcept { a.swap(b); }
};

class cowtree::node {

public:
  bool black = false;
  cowtree left, right;

  node& operator=(const node& o) = delete;
  virtual ~node() = default;
  virtual std::shared_ptr<node> clone() const = 0;

  template <class T>
  class typed;

private:
  node() = default;
  node(const node& o) = default;
};

template <class T>
class cowtree::node::typed : public cowtree::node {

public:
  T value;

  explicit typed(const T& value) : value(value) {}
  typed(const typed& o) = default;
  typed& operator=(const typed& o) = delete;

  virtual std::shared_ptr<node> clone() const {
    return std::make_shared<typed>(*this);
  }
};

inline const cowtree& cowtree::left() const {
  return get_node().left;
}
inline cowtree& cowtree::left() {
  return get_node().left;
}

inline const cowtree& cowtree::right() const {
  return get_node().right;
}
inline cowtree& cowtree::right() {
  return get_node().right;
}

inline bool cowtree::is_black() const {
  return empty() || get_node().black;
}
inline void cowtree::set_black(bool black) {
  get_node().black = black;
}

inline void cowtree::make_unique() {
  if (!empty() && !node_ptr.unique()) node_ptr = get_node().clone();
}

template <class T>
inline const T& cowtree::value() const {
  return static_cast<const node::typed<T>&>(get_node()).value;
}

template <class T>
inline T& cowtree::value() {
  return static_cast<node::typed<T>&>(get_node()).value;
}

template <class T>
inline std::shared_ptr<cowtree::node> cowtree::make_node(const T& value) {
  return std::make_shared<node::typed<T>>(value);
}

class cowtree::editor {

  editor* const parent_ptr;
  cowtree* subtree_ptr;

  void rotate();

  editor(editor& parent, cowtree& subtree);

public:
  editor(root& tree);
  editor(const editor&) = delete;
  editor& operator=(const editor&) = delete;

  bool is_root() const { return parent_ptr == nullptr; }
  bool is_left_child() const {
    return !is_root() && subtree_ptr == &parent_ptr->subtree_ptr->left();
  }

  const cowtree& subtree() const { return *subtree_ptr; }

  template <class T>
  T& value() {
    return subtree_ptr->template value<T>();
  }
  template <class T>
  void insert(const T& value);

  ~editor();

  class left;
  class right;
};

class cowtree::editor::left : public cowtree::editor {
public:
  left(const left&) = delete;
  left& operator=(const left&) = delete;
  left(editor& parent) : editor(parent, parent.subtree_ptr->left()) {}
};

class cowtree::editor::right : public cowtree::editor {
public:
  right(const right&) = delete;
  right& operator=(const right&) = delete;
  right(editor& parent) : editor(parent, parent.subtree_ptr->right()) {}
};

template <class T>
void cowtree::editor::insert(const T& value) {
  assert(subtree_ptr->empty());
  subtree_ptr->node_ptr = cowtree::make_node(value);
}

#endif