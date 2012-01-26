//
//  fastslam.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-16.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_fastslam_hpp
#define slam_fastslam_hpp

#include <cmath>

#include <boost/container/vector.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

#include <Eigen/Eigen>

#include "slam/vector_model.hpp"
#include "utility/random.hpp"
#include "utility/unscented.hpp"
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
    static const int control_dim = ControlModel::vector_dim;
    typedef typename ControlModel::vector_type control_vector_type;
    
    /** Vector representation of observations. */
    static const int observation_dim = ObservationModel::vector_dim;
    typedef typename ObservationModel::vector_type observation_vector_type;

    /** Distributions over states and features respectively. */
    typedef vector_model_adapter <multivariate_normal_adapter<state_type> > state_dist;
    typedef vector_model_adapter <multivariate_normal_adapter<feature_type> > feature_dist;

    
    /** Functors */
    
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
    
    struct particle_type {
        struct trajectory_type {
            state_type state;
            boost::shared_ptr<const trajectory_type> previous;
        } trajectory;
        cowmap<featureid_t, feature_dist> features;
    };
    
    /* Private implementation members */
    
    double particle_state_update (particle_type&) const;
    double particle_log_likelihood (const particle_type&) const;
    
    /** Data members */

    /** Our very own pseudo-random number generator. */
    mutable random_source random;
    
    particle_filter<particle_type> particles;
    size_t num_particles;
    double resample_threshold;

    bool keep_particle_trajectory;
    bitree<state_type> trajectory;

    const struct unscented_params_holder {
        unscented_params<control_dim> control;
        unscented_params<observation_dim> obs;
        unscented_params<feature_dim> feature;
        unscented_params<state_dim+feature_dim> state_feature;
        unscented_params_holder (double alpha, double beta, double kappa)
        : control(alpha, beta, kappa),
        obs(alpha, beta, kappa),
        feature(alpha, beta, kappa),
        state_feature(alpha, beta, kappa) { }
    } ukf_params;

    timestep_t current_timestep;
    ControlModel current_control;
    boost::container::vector<std::pair<featureid_t, ObservationModel> > seen_features, new_features;
    
public:
    
    fastslam (boost::program_options::variables_map& options, unsigned int seed);
    
    static boost::program_options::options_description program_options ();
    
    virtual void add_control (timestep_t, const ControlModel&);
    virtual void add_observation (timestep_t, featureid_t, const ObservationModel&, bool new_feature);
    virtual void end_observation (timestep_t);
    
};


template <class ControlModel, class ObservationModel>
void fastslam<ControlModel, ObservationModel>
::add_control (timestep_t timestep, const ControlModel& control) {
    assert (timestep == current_timestep);
    current_control = control;
    ++current_timestep;
}


template <class ControlModel, class ObservationModel>
void fastslam<ControlModel, ObservationModel>
::add_observation (timestep_t timestep, featureid_t feature_id, const ObservationModel& obs, bool new_feature) {
    assert (timestep == current_timestep);
    (new_feature ? new_features : seen_features).emplace_back (feature_id, obs);
}


template <class ControlModel, class ObservationModel>
void fastslam<ControlModel, ObservationModel>
::end_observation (timestep_t timestep) {
    
    assert (timestep == current_timestep);

    // Update particle state
    
    if (timestep > 0) {
        if (particles.effective_size() < num_particles*resample_threshold) {
            particles.resample (random, num_particles);
        }
        particles.update (boost::bind (&fastslam::particle_state_update, this, _1));
    }

    if (!keep_particle_trajectory) {
        trajectory.push_back (-trajectory.accumulate() + particles.max_weight_particle().trajectory.state);
    }
    
    // Update particle features

    for (size_t i = 0; i < seen_features.size(); ++i) {

        const featureid_t feature_id = seen_features[i].first;
        const ObservationModel& obs = seen_features[i].second;

        for (size_t j = 0; j < particles.size(); ++j) {
            
            particle_type& particle = particles[j];
            
            feature_dist feature = particle.features.get (feature_id);
            unscented_update (ukf_params.feature, feature.vector_model(), obs.vector_model(),
                              feature_observer (particle.trajectory.state));
            particle.features.insert (feature_id, feature);
        }
    }
    seen_features.clear();
    
    // Initialize new features
    
    for (size_t i = 0; i < new_features.size(); ++i) {
        
        const featureid_t feature_id = new_features[i].first;
        const ObservationModel& obs = new_features[i].second;
        
        for (size_t j = 0; j < particles.size(); ++j) {
            
            particle_type& particle = particles[j];
            
            feature_dist feature;
            unscented_transform (ukf_params.obs, obs.vector_model(), feature.vector_model(),
                                 feature_dist::matrix_type::Zero(),
                                 feature_initializer (particle.trajectory.state));
            particle.features.insert (feature_id, feature);
        }
    }
    new_features.clear();
}


template <class ControlModel, class ObservationModel>
double fastslam<ControlModel, ObservationModel>
::particle_state_update (particle_type& particle) const {
    
    state_dist state, state_proposal;

    unscented_transform (ukf_params.control, current_control.vector_model(), state.vector_model(),
                         state_dist::matrix_type::Zero(), state_predictor (particle.trajectory.state));
    
    { // Calculate proposal distribution
        
        multivariate_normal_dist<state_dim+feature_dim> state_feature_joint;
        state_feature_joint.mean().template head<state_dim>() = state.vector_model().mean();
        state_feature_joint.chol_cov().template topLeftCorner<state_dim, state_dim>() = state.vector_model().chol_cov();
        
        for (size_t i = 0; i < seen_features.size(); ++i) {
            
            const feature_dist& feature = particle.features.get (seen_features[i].first);
            const ObservationModel& obs = seen_features[i].second;
            
            state_feature_joint.mean().template tail<feature_dim>() = feature.vector_model().mean();
            state_feature_joint.chol_cov().template bottomRightCorner<feature_dim, feature_dim>() = feature.vector_model().chol_cov();

            state_feature_joint.chol_cov().template topRightCorner<state_dim, feature_dim>().setZero();
            state_feature_joint.chol_cov().template bottomLeftCorner<feature_dim, state_dim>().setZero();

            unscented_update (ukf_params.state_feature, state_feature_joint, obs.vector_model(),
                              state_feature_observer());
        }
        
        state_proposal.vector_model().mean() = state_feature_joint.mean().template head<state_dim>();
        state_proposal.vector_model().chol_cov() = state_feature_joint.chol_cov().template topLeftCorner<state_dim, state_dim>();
    }
    
    if (keep_particle_trajectory) {
        particle.trajectory.previous = boost::make_shared<typename particle_type::trajectory_type> (particle.trajectory);
    }
    
    particle.trajectory.state = state_proposal (random);
    
    return std::exp (particle_log_likelihood (particle)
                     + state.log_likelihood (particle.trajectory.state)
                     - state_proposal.log_likelihood (particle.trajectory.state));
}


template <class ControlModel, class ObservationModel>
double fastslam<ControlModel, ObservationModel>
::particle_log_likelihood (const particle_type& particle) const {

    double log_likelihood = 0;

    for (size_t i = 0; i < seen_features.size(); ++i) {

        const feature_dist& feature = particle.features.get (seen_features[i].first);
        const ObservationModel& obs = seen_features[i].second;
        
        multivariate_normal_adapter<typename ObservationModel::vector_model_type> predicted_obs;
        unscented_transform (ukf_params.feature, feature.vector_model(), predicted_obs,
                             obs.vector_model().derived().chol_cov(),
                             feature_observer (particle.trajectory.state));
        
        log_likelihood += predicted_obs.log_likelihood (obs.vector_model().mean());
    }
    
    return log_likelihood;
}


template <class ControlModel, class ObservationModel>
boost::program_options::options_description fastslam<ControlModel, ObservationModel>
::program_options () {
	namespace po = boost::program_options;
	po::options_description options ("FastSLAM 2.0 Parameters");
	options.add_options()
    ("num-particles", po::value<size_t>()->default_value(10), "Number of particles in the particle filter")
    ("resample-threshold", po::value<double>()->default_value(0.5), "Minimum ratio of effective particles")
    ("particle-trajectory", "Keep per-particle trajectory information")
    ("ukf-alpha", po::value<double>()->default_value(0.002), "The alpha parameter for the scaled UKF")
    ("ukf-beta", po::value<double>()->default_value(2), "The beta parameter for the scaled UKF")
    ("ukf-kappa", po::value<double>()->default_value(0), "The kappa parameter for the scaled UKF")
    ("fastslam-seed", po::value<unsigned int>(), "FastSLAM 2.0 random seed");
	return options;
}


template <class ControlModel, class ObservationModel>
fastslam<ControlModel, ObservationModel>
::fastslam (boost::program_options::variables_map& options, unsigned int seed)
: random                (remember_option (options, "fastslam-seed", seed)),
num_particles           (options["num-particles"].as<size_t>()),
resample_threshold      (options["resample-threshold"].as<double>()),
keep_particle_trajectory(options.count("particle-trajectory")),
ukf_params              (options["ukf-alpha"].as<double>(),
                         options["ukf-beta"].as<double>(),
                         options["ukf-kappa"].as<double>()),
current_timestep        (0) { }

#endif
