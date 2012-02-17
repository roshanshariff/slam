//
//  vector_transforms.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-11.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_vector_transforms_hpp
#define slam_vector_transforms_hpp

template <class ControlModel, class ObservationModel>
struct vector_transform_functors {
    
    using state_type = typename ControlModel::result_type;
    using feature_type = typename ObservationModel::result_type;

    /** Vector representation of states. */
    static const int state_dim = state_type::vector_dim;
    using state_vector_type = typename state_type::vector_type;
    
    /** Vector representation of features. */
    static const int feature_dim = feature_type::vector_dim;
    using feature_vector_type = typename feature_type::vector_type;
    
    /** Vector representations of controls. */
    static const int control_dim = ControlModel::vector_dim;
    using control_vector_type = typename ControlModel::vector_type;
    
    /** Vector representation of observations. */
    static const int observation_dim = ObservationModel::vector_dim;
    using observation_vector_type = typename ObservationModel::vector_type;
    
    
    struct state_predictor {
        const state_type& state;
        state_predictor (const state_type& state) : state(state) { }
        state_vector_type operator() (const control_vector_type& control) const {
            return (state + ControlModel::from_vector (control)).to_vector();
        }
    };
    
    
    struct state_feature_observer {
        observation_vector_type operator() (const Eigen::Matrix<double, state_dim+feature_dim, 1>& vec) const {
            state_type state = state_type::from_vector (vec.template head<state_dim>());
            feature_type feature = feature_type::from_vector (vec.template tail<feature_dim>());
            return ObservationModel::to_vector (-state + feature);
        }
    };
    
    
    struct feature_observer {
        const state_type& state;
        feature_observer (const state_type& state) : state(state) { }
        observation_vector_type operator() (const feature_vector_type& feature) const {
            return ObservationModel::to_vector (-state + feature_type::from_vector (feature));
        }
    };
    
    
    struct feature_initializer {
        const state_type& state;
        feature_initializer (const state_type& state) : state(state) { }
        feature_vector_type operator() (const observation_vector_type& obs) const {
            return (state + ObservationModel::from_vector(obs)).to_vector();
        }
    };
    
};

#endif
