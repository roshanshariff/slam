//
//  interfaces.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-09.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_interfaces_hpp
#define slam_interfaces_hpp

#include <cstddef>

#include <boost/container/container_fwd.hpp>
#include <boost/shared_ptr.hpp>

#include "utility/bitree_fwd.hpp"

namespace slam {
    
    
    class timestep_type {
        std::size_t timestep;
    public:
        explicit timestep_type (std::size_t timestep) : timestep(timestep) { }
        operator std::size_t () const { return timestep; }
        bool operator== (timestep_type other) const { return timestep == other.timestep; }
        bool operator< (timestep_type other) const { return timestep < other.timestep; }

        timestep_type& operator++ () { ++timestep; return *this; }
        timestep_type& operator-- () { --timestep; return *this; }
        timestep_type& operator+= (int x) { timestep += x; return *this; }
        timestep_type& operator-= (int x) { timestep -= x; return *this; }
    };
    
    inline timestep_type operator++ (timestep_type& t, int) { auto copy = t; ++t; return copy; }
    inline timestep_type operator-- (timestep_type& t, int) { auto copy = t; --t; return copy; }
    
    inline timestep_type operator+ (timestep_type t, int x) { return t += x; }
    inline timestep_type operator- (timestep_type t, int x) { return t -= x; }
    
    
    class featureid_type {
        std::size_t featureid;
    public:
        explicit featureid_type (std::size_t featureid) : featureid(featureid) { }
        operator std::size_t () const { return featureid; }
        bool operator== (featureid_type other) const { return featureid == other.featureid; }
        bool operator< (featureid_type other) const { return featureid < other.featureid; }
    };
    
    
    struct timestep_listener {
        virtual void timestep (timestep_type) = 0;
    };
    
    
    struct data_source : public timestep_listener {
        virtual timestep_type current_timestep () const = 0;
    };
    
    
    template <class State, class Feature>
    struct slam_result : public data_source {
        
        typedef utility::bitree<State> trajectory_type;
        typedef boost::container::flat_map<featureid_type, Feature> feature_map_type;
        
        virtual State get_state () const = 0;
        virtual boost::shared_ptr<const trajectory_type> get_trajectory () const = 0;
        virtual boost::shared_ptr<const feature_map_type> get_map () const = 0;
    };
    
    
}

#endif
