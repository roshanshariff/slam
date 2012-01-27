//
//  slam_estimator.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-26.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_slam_estimator_hpp
#define slam_slam_estimator_hpp

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/container/flat_map.hpp>

#include "slam/slam_data.hpp"
#include "utility/bitree.hpp"

template <class ControlModel, class ObservationModel>
class slam_estimator {
    
    typedef typename ControlModel::result_type state_type;
    typedef typename ObservationModel::result_type feature_type;

    typedef bitree<state_type> trajectory_type;

    typedef typename slam_data<ControlModel, ObservationModel>::featureid_t featureid_t;
    typedef boost::container::flat_map<featureid_t, feature_type> map_estimate_type;

public:
    
    virtual state_type state_estimate () const = 0;
    
    virtual boost::shared_ptr<const trajectory_type> trajectory_estimate () const = 0;
    
    virtual boost::shared_ptr<const map_estimate_type> map_estimate () const = 0;
    
};

#endif
