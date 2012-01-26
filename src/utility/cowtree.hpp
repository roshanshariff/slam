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
#include <utility>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/utility.hpp>


class cowtree {
    
public:
    
    bool empty () const { return node_ptr.get() == 0; }
    
    const cowtree& left () const;
    const cowtree& right () const;
    template <class T> const T& value () const;
    
    class root;
    class editor;
    
protected:
    
    cowtree () { }
    cowtree (const cowtree& o) : node_ptr(o.node_ptr) { }
    cowtree& operator= (const cowtree& o) { node_ptr = o.node_ptr; return *this; }
    void swap (cowtree& o) { node_ptr.swap(o.node_ptr); }
    
    void clear () { node_ptr.reset(); }
    
private:
    
    class node;

    const node& get_node () const { assert(!empty()); return *node_ptr; }
    node& get_node () { assert(!empty()); return *node_ptr; }
    
    cowtree& left ();
    cowtree& right ();
    template <class T> T& value ();
    
    bool is_black () const;
    void set_black (bool black);
    void make_unique ();
    
    template <class T> static boost::shared_ptr<node> make_node (const T& value);
    
    boost::shared_ptr<node> node_ptr;
    
};


class cowtree::root : public cowtree {
public:
    void swap (root& o) { cowtree::swap(o); }
    void clear () { cowtree::clear(); }
};


inline void swap (cowtree::root& a, cowtree::root& b) { a.swap(b); }

namespace std {
    template<> void swap (cowtree::root& a, cowtree::root& b);
}

class cowtree::node {
    
public:
    
    bool black;
    cowtree left, right;
    
    virtual ~node() { }    
    virtual boost::shared_ptr<node> clone () const = 0;
    
    template <class T> class typed;
    
protected:
    
    node () { }
    node (const node& o) : black(o.black), left(o.left), right(o.right) { }
    
private:
    
    node& operator= (const node& o);
    
};


template <class T>
class cowtree::node::typed : public cowtree::node {
    
public:
    
    T value;
    
    explicit typed (const T& value) : value(value) { }
    typed (const typed<T>& o) : node(o), value(o.value) { }
    virtual boost::shared_ptr<node> clone () const { return boost::make_shared<typed<T> >(*this); }
    
private:
    
    typed& operator= (const typed& o);
    
};


inline const cowtree& cowtree::left () const { return get_node().left; }
inline cowtree& cowtree::left () { return get_node().left; }

inline const cowtree& cowtree::right () const { return get_node().right; }
inline cowtree& cowtree::right () { return get_node().right; }

inline bool cowtree::is_black () const { return empty() || get_node().black; }
inline void cowtree::set_black (bool black) { get_node().black = black; }

inline void cowtree::make_unique () {
    if (!empty() && !node_ptr.unique()) node_ptr = get_node().clone();
}

template <class T> inline const T& cowtree::value () const {
    return static_cast<const node::typed<T>&>(get_node()).value;
}

template <class T> inline T& cowtree::value () {
    return static_cast<node::typed<T>&>(get_node()).value;
}

template <class T> inline boost::shared_ptr<cowtree::node> cowtree::make_node (const T& value) {
    return boost::make_shared<node::typed<T> >(value);
}


class cowtree::editor : boost::noncopyable {
    
    editor* const parent_ptr;
    cowtree* subtree_ptr;
    
    void rotate ();
    
    editor (editor& parent, cowtree& subtree);
    
public:
    
    editor (root& tree);
    
    bool is_root () const { return parent_ptr == 0; }    
    bool is_left_child () const { return !is_root() && subtree_ptr == &parent_ptr->subtree_ptr->left(); }

    const cowtree& subtree () const { return *subtree_ptr; }
    
    template <class T> T& value () { return subtree_ptr->template value<T>(); }    
    template <class T> void insert (const T& value);
    
    ~editor ();
    
    class left;
    class right;
    
};


class cowtree::editor::left : public cowtree::editor {
    left (const left&);
    left& operator= (const left&);
public:
    left (editor& parent) : editor(parent, parent.subtree_ptr->left()) { }
};


class cowtree::editor::right : public cowtree::editor {
    right (const right&);
    right& operator= (const right&);
public:
    right (editor& parent) : editor(parent, parent.subtree_ptr->right()) { }
};


template <class T> void cowtree::editor::insert (const T& value) {
    assert(subtree_ptr->empty());
    subtree_ptr->node_ptr = cowtree::make_node(value);
}


#endif