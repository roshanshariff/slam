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

#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

template <class T>
class cowtree {
    
    T m_data;
    bool m_black;
    
    explicit cowtree (const T& data_) : m_data(data_) { }

public:
    
    class editor;

    class handle {
        
        friend class editor;
        
        boost::shared_ptr<cowtree> m_node;
        
    public:
        
        handle () { }        
        
        bool is_empty () const { return m_node.get() == 0; }
        
        const handle& left () const;
        const handle& right () const;
        const T& data () const;

        void swap (handle& other) { m_node.swap (other.m_node); }
        
        bool operator== (const handle& other) const { return this == &other; }
        
    private:
        
        const cowtree& node () const { assert(!is_empty()); return *m_node; }
        cowtree& node () { assert(!is_empty()); return *m_node; }

        void make_unique () {
            if (!is_empty() && !m_node.unique()) {
                m_node = boost::make_shared<cowtree>(node());
            }
        }

    };
    
private:
    
    handle m_left;
    handle m_right;
    
public:
    
    class editor : boost::noncopyable {
        
        editor* const m_parent;
        handle* m_hnd;
        
        const handle& handle () const { assert(m_hnd); return *m_hnd; }
        handle& handle () { assert(m_hnd); return *m_hnd; }
        
        bool is_black () const { return handle().is_empty() || handle().node().m_black; }
        void set_black (bool black) { handle().node().m_black = black; }
        
        void rotate ();
        
    protected:
        
        editor (editor& p, handle& hnd) : m_parent(&p), m_hnd(&hnd) { }
        
        handle& left () { handle().make_unique(); return handle().node().m_left; }        
        handle& right () { handle().make_unique(); return handle().node().m_right; }
    
    public:
        
        T& data () { handle().make_unique(); return handle().node().m_data; }
        
        editor (handle& hnd) : m_parent(0), m_hnd(&hnd) { assert (handle().is_black()); }
        
        ~editor ();

        bool is_root () const { return m_parent == 0; }
        editor& parent () { assert(!is_root()); return *m_parent; }
        
        bool is_left_child () const { return handle() == parent().handle().left(); }
        
    };
    
    struct left_editor : public editor { left_editor (editor& p) : editor(p, p.left()) { } };
    
    struct right_editor : public editor { right_editor (editor& p) : editor(p, p.right()) { } };
        
};


template <class T>
inline const handle& cowtree<T>::handle::left () const { return node().m_left; }

template <class T>
inline const handle& cowtree<T>::handle::right () const { return node().m_right; }

template <class t>
inline const handle& cowtree<T>::handle::data () const { return node().m_data; }


template <class T>
void cowtree<T>::editor::rotate () {
    if (is_left_child()) {
        right().swap(parent().handle());
        parent().handle().swap(handle());
        m_hnd = &parent().right();
    }
    else {
        left().swap(parent().handle());
        parent().handle().swap(handle());
        m_hnd = &parent().left();
    }
}


template <class T>
cowtree<T>::editor::~inserter () {
    if (!is_black()) {
        if (is_root()) {
            set_black(true);
        }
        else if (!parent().is_black()) {
            if (is_left_child() != parent().is_left_child()) {
                rotate();
            }
            set_black(true);
            parent().rotate();
        }
    }
}



#endif
