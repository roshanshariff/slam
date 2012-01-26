//
//  cowtree.cpp
//  slam
//
//  Created by Roshan Shariff on 11-10-09.
//  Copyright 2011 University of Alberta. All rights reserved.
//

#include <cassert>

#include "utility/cowtree.hpp"

template<> void std::swap (cowtree::root& a, cowtree::root& b) {
    a.swap(b);
}

cowtree::editor::editor (editor& parent, cowtree& subtree)
: parent_ptr(&parent), subtree_ptr(&subtree)
{
    assert (subtree_ptr != 0);
    subtree_ptr->make_unique();
}


cowtree::editor::editor (root& tree)
: parent_ptr(0), subtree_ptr(&tree)
{
    assert (subtree_ptr != 0);
    subtree_ptr->make_unique();
    assert (subtree_ptr->is_black());
}


void cowtree::editor::rotate () {
    if (is_left_child()) {
        parent_ptr->subtree_ptr->swap(subtree_ptr->right());
        parent_ptr->subtree_ptr->swap(*subtree_ptr);
        subtree_ptr = &parent_ptr->subtree_ptr->right();
    }
    else {
        parent_ptr->subtree_ptr->swap(subtree_ptr->left());
        parent_ptr->subtree_ptr->swap(*subtree_ptr);
        subtree_ptr = &parent_ptr->subtree_ptr->left();
    }
    assert (subtree_ptr != 0);
}


cowtree::editor::~editor () {
    if (!subtree_ptr->is_black()) {
        if (is_root()) {
            subtree_ptr->set_black(true);
        }
        else if (!parent_ptr->subtree_ptr->is_black()) {
            if (is_left_child() != parent_ptr->is_left_child()) {
                rotate();
            }
            subtree_ptr->set_black(true);
            parent_ptr->rotate();
        }
    }
}
