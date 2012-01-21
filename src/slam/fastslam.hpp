//
//  fastslam.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-16.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_fastslam_hpp
#define slam_fastslam_hpp

#include <vector>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Eigen>

#include "utility/random.hpp"
#include "utility/cowmap.hpp"
#include "utility/options.hpp"

template <class ControlModel, class ObservationModel>
class fastslam : public slam_data<ControlModel, ObservationModel>::listener {
    
    typedef slam_data<ControlModel, ObservationModel> slam_data_type;
    
	/** The types of action and feature identifiers, respectively. Usually same as size_t. */
	typedef typename slam_data_type::timestep_t timestep_t;
	typedef typename slam_data_type::featureid_t featureid_t;
        
    /** The types of controls and observations, respectively. Also used as states and features. */
	typedef typename ControlModel::result_type state_type;
	typedef typename ObservationModel::result_type feature_type;
    
    /** Vector representation of states. */
    static const int state_dim = state_type::vector_dim;
    typedef typename state_type::vector_type state_vector_type;

    /** Vector representation of features. */
    static const int feature_dim = feature_type::vector_dim;
    typedef typename feature_type::vector_type feature_vector_type;
    
    /** Vector representations of controls. */
    static const int control_dim = control_model::vector_dim;
    typedef typename control_model::vector_type control_vector_type;
    
    /** Vector representation of observations. */
    static const int observation_dim = observation_model::vector_dim;
    typedef typename observation_model::vector_type observation_vector_type;
    
    
    struct state_dist : public multivariate_normal_dense_base<state_dim, state_dist> {
        
        typedef multivariate_normal_dense_base<state_dim, state_dist> base_type;
        typedef typename base_type::vector_type vector_type;
        typedef typename base_type::matrix_type matrix_type;
        
        state_dist () { }
        
        state_dist (const vector_type& mean, const matrix_type& cov) : base_type(mean, cov) { }
        
        static vector_type subtract (const vector_type& a, const vector_type& b) {
            return state_type::subtract (a, b);
        }
        
    };
    

    struct feature_dist : public multivariate_normal_dense_base<feature_dim, feature_dist> {

        typedef multivariate_normal_dense_base<feature_dim, feature_dist> base_type;
        typedef typename base_type::vector_type vector_type;
        typedef typename base_type::matrix_type matrix_type;
        
        feature_dist () { }
        
        feature_dist (const vector_type& mean, const matrix_type& cov) : base_type(mean, cov) { }
        
        static vector_type subtract (const vector_type& a, const vector_type& b) {
            return feature_type::subtract (a, b);
        }
        
    };
    

    struct state_predictor {        
        state_vector_type (const Eigen::Matrix<double, state_dim+control_dim, 1>& vec) const {
            return (state_type::from_vector (vec.head<state_dim>())
                    + control_model::from_vector (vec.tail<control_dim>())
                    ).to_vector();
        }
    };
    

    struct state_updater {
        observation_vector_type (const Eigen::Matrix<double, state_dim+feature_dim>& vec) const {
            return observation_model::to_vector (
                    -state_type::from_vector (vec.head<state_dim>())
                    + feature_type::from_vector (vec.tail<feature_dim>())
            );
        }
    }
    

    class feature_updater {
        const state_type base;
    public:
        feature_observer (const state_type& state) : base(-state) { }
        observation_vector_type operator() (const feature_vector_type& feature) const {
            return observation_model::to_vector (base + feature_type::from_vector (feature));
        }
    };
    

    struct trajectory_t {
        state_type state;
        boost::shared_ptr<const trajectory_t> previous;
    };
    

    struct particle_t {
        double weight;
        trajectory_type trajectory;
        cowmap<featureid_t, feature_dist> features;
    };
    

    std::vector<particle_t> particles;
    
    unscented_params<state_dim+control_dim> state_predict_params;
    unscented_params<state_dim+feature_dim> state_update_params;
    bool keep_trajectory;
    
    void update_particle (particle_t& particle, const control_model& control) {
        
        state_dist state;

        { // Create state distibution augmented with control
            multivariate_normal_dist<state_dim+control_dim> state_aug;
            state_aug.mean().head<state_dim>() = particle.trajectory.current_state.to_vector();
            state_aug.mean().tail<control_dim>() = control.vector_model().mean();
            state_aug.chol_cov().setZero();
            state_aug.chol_cov().bottomRightCorner<control_dim, control_dim>() = control.vector_model().chol_cov();
        
            unscented_transform (state_predictor(), state_aug, state, state_predict_params);
        }
        
        { /* for each observed feature */
            
            const feature_dist& feature;
            const observation_model& observation;
            
            // Create state distribution augmented with feature
            multivariate_normal_dist<state_dim+feature_dim> state_aug;
            state_aug.mean().head<state_dim>() = state.mean();
            state_aug.mean().tail<feature_dim>() = feature.mean();
            state_aug.chol_cov().setZero();
            state_aug.chol_cov().topLeftCorner<state_dim, state_dim>() = state.chol_cov();
            state_aug.chol_cov().bottomRightCorner<feature_dim, feature_dim>() = feature.chol_cov();
            
            unscented_update (state_updater(), state_aug, observation.vector_model(), state_update_params);
        }
        
        if (keep_trajectory) particle.trajectory.previous = boost::make_shared<particle_t> (particle);
        particle.trajectory.state = state_type::from_vector
    }
};

#endif
