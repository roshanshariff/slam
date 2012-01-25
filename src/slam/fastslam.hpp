//
//  fastslam.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-16.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_fastslam_hpp
#define slam_fastslam_hpp

#include <boost/container/vector.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/bind.hpp>

#include <Eigen/Eigen>

#include "utility/random.hpp"
#include "utility/cowmap.hpp"
#include "utility/bitree.hpp"
#include "utility/options.hpp"
#include "slam/particle_filter.hpp"

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
        const state_type& state;
        state_predictor (const state_type& state) : state(state) { }
        state_vector_type operator() (const control_vector_type& control) const {
            return (state + ControlModel::from_vector (control)).to_vector();
        }
    };
    
    struct state_feature_observer {
        observation_vector_type operator() (const Eigen::Matrix<double, state_dim+feature_dim>& vec) const {
            state_type state = state_type::from_vector (vec.head<state_dim>());
            feature_type feature = feature_type::from_vector (vec.tail<state_dim>());
            return -state + feature;
        }
    };
    
    struct feature_observer {
        const state_type& state;
        feature_observer (const state_type& state) : state(state) { }
        observation_vector_type operator() (const feature_vector_type& feature) const {
            return observation_model::to_vector (-state + feature_type::from_vector (feature));
        }
    };
    
    struct feature_initializer {
        const state_type& state;
        feature_initializer (const state_type& state) : state(state) { }
        feature_vector_type operator() (const observation_vector_type& obs) const {
            return (state + ObservationModel::from_vector(obs)).to_Vector();
        }
    };
    
    struct particle_type {
        struct trajectory_type {
            state_type state;
            boost::shared_ptr<const trajectory_type> previous;
        } trajectory;
        cowmap<featureid_t, feature_dist> features;
    };

    mutable random_source random;
    timestep_t current_timestep;
    ControlModel current_control;
    boost::dynamic_bitset feature_observed;
    boost::container::vector<std::pair<featureid_t, ObservationModel> > seen_features, new_features;
    
    particle_filter<particle_type> particles;
    size_t num_particles;
    double resample_threshold;

    bool keep_particle_trajectory;
    bitree<state_type> trajectory;

    unscented_params<control_dim> ukf_control_params;
    unscented_params<observation_dim> ukf_obs_params;
    unscented_params<state_dim+feature_dim> ukf_state_feature_params;
    unscented_params<feature_dim> ukf_feature_params;
    
public:
    
    virtual void add_control (timestep_t, const ControlModel&);
    virtual void add_observation (timestep_t, featureid_t, const ObservationModel&);
    virtual void end_observation (timestep_t);
    
};

template <class ControlModel, ObservationModel>
void fastslam<ControlModel, ObservationModel>
::add_control (timestep_t timestep, const ControlModel& control) {
    assert (timestep == current_timestep);
    current_control = control;
    ++current_timestep;
}

template <class ControlModel, class ObservationModel>
void fastslam<ControlModel, ObservationModel>
::add_observation (timestep_t timestep, featureid_t feature_id, const ObservationModel& obs) {
    assert (timestep == current_timestep);
    if (feature_observed.size() <= feature_id) feature_observed.resize (feature_id+1);
    if (feature_observed[feature_id]) seen_features.emplace_back (feature_id, obs);
    else new_features.emplace_back (feature_id, obs);
}

template <class ControlModel, class ObservationModel>
void fastslam<ControlModel, ObservationModel>
::end_observation (timestep_t timestep) {
    
    assert (timestep == current_timestep);

    if (timestep > 0) particles.update (boost::bind (&fastslam::update_particle, this, _1));
    particles.update (boost::bind (&fastslam::initialize_features, this, _1));

    for (size_t i = 0; i < new_features.size(); ++i) {
        feature_observed[new_features[i].first] = true;
    }
    
    if (!keep_particle_trajectory) {
        trajectory.push_back (-trajectory.accumulate() + particles.max_weight_particle().trajectory.state).
    }
    
    if (particles.effective_size() < num_particles*resample_threshold) {
        particles.resample (random, num_particles);
    }

    seen_features.clear();
    new_features.clear();
}

template <class ControlModel, class ObservationModel>
double fastslam<ControlModel, ObservationModel>::update_particle (particle_type& particle) const {
    
    state_dist state;

    unscented_transform (ukf_control_params, current_control.vector_model(), state,
                         state_dist::matrix_type::Zero(), state_predictor (particle.trajectory.state));
    
    { // Calculate importance weights and proposal distribution
        
        multivariate_normal_dist<state_dim+feature_dim> state_feature_joint, state_proposal;

        state_feature_joint.chol_cov().setZero();
        state_proposal.chol_cov().setZero();
        
        state_feature_joint.mean().head<state_dim>() = state.mean();
        state_proposal.mean().head<state_dim>() = state.mean();
        
        state_feature_joint.chol_cov().topLeftCorner<state_dim, state_dim> = state.chol_cov();
        state_proposal.chol_cov().topLeftCorner<state_dim, state_dim> = state.chol_cov();
        
        double log_weight = 0;
        
        for (size_t i = 0; i < seen_features.size(); ++i) {
            
            const feature_dist& feature = particle.features.get (seen_features[i].first);
            const ObservationModel& obs = seen_features[i].second;
            
            state_feature_joint.mean().tail<feature_dim>() = feature.mean();
            state_proposal.mean().tail<feature_dim>() = feature.mean();

            state_feature_joint.chol_cov().bottomRightCorner<feature_dim, feature_dim>() = feature.chol_cov();
            state_proposal.chol_cov().bottomRightCorner<feature_dim, feature_dim>() = feature.chol_cov();

            // Importance weight
            
            multivariate_normal_dense_adapter<typename ObservationModel::vector_model::type> predicted_obs;
            unscented_transform (ukf_state_feature_params, state_feature_joint, predicted_obs,
                                 obs.vector_model().derived().chol_cov(), state_feature_observer());
            
            log_weight += predicted_obs.log_likelihood (obs.vector_model().mean());
            
            // State proposal
            
            unscented_update (ukf_state_feature_params, state_proposal, obs.vector_model(),
                              state_feature_observer());
            
            state_proposal.chol_cov().topRightCorner<state_dim, feature_dim>().setZero();
            state_proposal.chol_cov().bottomLeftCorner<feature_dim, state_dim>().setZero();
        }
        
        state.mean() = state_proposal.mean().head<state_dim>();
        state.chol_cov() = state_proposal.chol_cov().topLeftCorner<state_dim, state_dim>();
    }
    
    if (keep_particle_trajectory) {
        particle.trajectory.previous = boost::make_shared<particle_t> (particle.trajectory);
    }
    particle.trajectory.state = state_type::from_vector (state (random));
    
    // Update feature estimates
    
    for (size_t i = 0; i < seen_features.size(); ++i) {
        feature_dist feature = particle.features.get (seen_features[i].first);
        unscented_update (ukf_feature_params, feature, seen_features[i].second.vector_model(),
                          feature_observer (particle.trajectory.state));
        particle.features.insert (seen_features[i].first, feature);
    }
    
}

template <class ControlModel, class ObservationModel>
double fastslam<ControlModel, ObservationModel>::initialize_features (particle_type& particle) const {
    for (size_t i = 0; i < new_features.size(); ++i) {
        feature_dist feature;
        unscented_transform (ukf_obs_params, new_features[i].second.vector_model(), feature,
                             feature_dist::matrix_type::Zero(), feature_initializer (particle.trajectory.state));
        particle.features.insert (new_features[i].first, feature);
    }
    return 1.0;
}


#endif
