//
//  cowtree.cpp
//  slam
//
//  Created by Roshan Shariff on 11-10-09.
//  Copyright 2011 University of Alberta. All rights reserved.
//

#include <cassert>

#include "utility/cowtree.hpp"


cowtree::editor::editor (editor& parent, cowtree& subtree)
: parent_ptr(&parent), subtree_ptr(&subtree)
{
    subtree.make_unique();
}


cowtree::editor::editor (root& tree)
: parent_ptr(0), subtree_ptr(&tree)
{
    subtree().make_unique();
    assert (subtree().is_black());
}


void cowtree::editor::rotate () {
    if (is_left_child()) {
        parent().subtree().swap(subtree().right());
        parent().subtree().swap(subtree());
        subtree_ptr = &parent().subtree().right();
    }
    else {
        parent().subtree().swap(subtree().left());
        parent().subtree().swap(subtree());
        subtree_ptr = &parent().subtree().left();
    }
}


cowtree::editor::~editor () {
    if (!subtree().is_black()) {
        if (is_root()) {
            subtree().set_black(true);
        }
        else if (!parent().subtree().is_black()) {
            if (is_left_child() != parent().is_left_child()) {
                rotate();
            }
            subtree().set_black(true);
            parent().rotate();
        }
    }
}
