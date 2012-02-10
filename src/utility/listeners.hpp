//
//  listeners.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-10.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_listeners_hpp
#define slam_listeners_hpp

#include <vector>
#include <algorithm>

#include <boost/shared_ptr.hpp>


namespace utility {
    
    
    template <class Listener>
    class listeners {
        
        mutable std::vector<boost::weak_ptr<Listener>> weak_ptrs;
        
        template <class Functor>
        class invoke_or_remove {
            
            Functor& f;
            
        public:
            
            invoke_or_remove (Functor& f) : f(f) { }
            
            bool operator() (const boost::weak_ptr<Listener>& weak_ptr) {
                if (auto listener = weak_ptr.lock()) {
                    f (listener.get());
                    return false;
                }
                else {
                    return true;
                }
            }
        };
        
    public:
        
        void add (const boost::shared_ptr<Listener>& l) { weak_ptrs.push_back (l); }
        
        template <class Functor> void for_each (Functor) const;

    };
    
    
    template <class Listener> template <class Functor>
    void listeners<Listener>::for_each (Functor f) const {

        weak_ptrs.erase (std::remove_if (weak_ptrs.begin(), weak_ptrs.end(), invoke_or_remove<Functor>(f)),
                         weak_ptrs.end());
    }

    
}

#endif
