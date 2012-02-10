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
#include <boost/container/flat_map.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/none.hpp>
#include <boost/bind.hpp>

#include <Eigen/Eigen>

#include "slam/vector_model.hpp"
#include "slam/particle_filter.hpp"
#include "slam/interfaces.hpp"
#include "utility/random.hpp"
#include "utility/unscented.hpp"
#include "utility/cowmap.hpp"
#include "utility/bitree.hpp"
#include "utility/utility.hpp"


namespace slam {

template <class ControlModel, class ObservationModel>
class fastslam :
public slam_data<ControlModel, ObservationModel>::listener,
public slam_result<typename ControlModel::result_type, typename ObservationModel::result_type> {
    
    typedef slam_data<ControlModel, ObservationModel> slam_data_type;
    
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
    typedef vector_model_adapter<multivariate_normal_adapter<state_type> > state_dist;
    typedef vector_model_adapter<multivariate_normal_adapter<feature_type> > feature_dist;
    
    typedef utility::bitree<state_type> trajectory_type;
    typedef boost::container::flat_map<featureid_type, feature_type> map_estimate_type;
    
    /** Nested types */
    
    struct state_predictor;
    struct state_feature_observer;
    struct feature_observer;
    struct feature_initializer;
    struct map_estimate_inserter;
    
    struct particle_type {
        struct state_list {
            state_type state;
            boost::shared_ptr<const state_list> previous;
        } trajectory;
        cowmap<featureid_type, feature_dist> features;
    };
    
    /* Private implementation members */
    
    double particle_state_update (particle_type&) const;
    double particle_log_likelihood (const particle_type&) const;
    
    /** Data members */

    /** Our very own pseudo-random number generator. */
    mutable random_source random;
    
    /** The particle filter, its target size, and the resample threshold. */
    particle_filter<particle_type> particles;
    size_t num_particles;
    double resample_threshold;

    /** Whether to keep a per-particle trajectory as opposed to one combined trajectory. */
    const bool keep_particle_trajectory;
    mutable boost::shared_ptr<trajectory_type> trajectory_estimate;
    mutable boost::shared_ptr<map_estimate_type> map_estimate;

    /** All the UKF parameters used by FastSLAM */
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

    /** Current timestep, current control, and observations made in the current timestep. */
    timestep_type timestep_counter;
    boost::optional<ControlModel> current_control;
    boost::container::vector<std::pair<featureid_type, ObservationModel> > seen_features, new_features;
    
public:
    
    fastslam (boost::program_options::variables_map& options, unsigned int seed);
    
    static boost::program_options::options_description program_options ();
    
    // Overridden virtual member functions of slam::timestep_listener
    
    virtual void timestep (timestep_type t) override;
    
    // Overridden virtual member functions of slam::data_source
    
    virtual timestep_type current_timestep () const override { return timestep_counter; }
        
    // Overridden virtual member functions of slam::slam_data::listener
    
    virtual void control (timestep_type, const ControlModel&);
    virtual void observation (timestep_type, const typename slam_data_type::observation_data_type&);
    
    // Overridden virtual member functions of slam_result
    
    virtual state_type get_state () const {
        if (trajectory_estimate) return trajectory_estimate->accumulate();
        else return particles.max_weight_particle().trajectory.state;
    }
    
    virtual boost::shared_ptr<const trajectory_type> get_trajectory () const;
    
    virtual boost::shared_ptr<const map_estimate_type> get_map () const {
        if (!map_estimate) {
            map_estimate = boost::make_shared<map_estimate_type>();
            particles.max_weight_particle().features.for_each (map_estimate_inserter (*map_estimate));
        }
        return map_estimate;
    }

};


template <class ControlModel, class ObservationModel>
struct fastslam<ControlModel, ObservationModel>
::state_predictor {
    const state_type& state;
    state_predictor (const state_type& state) : state(state) { }
    state_vector_type operator() (const control_vector_type& control) const {
        return (state + ControlModel::from_vector (control)).to_vector();
    }
};


template <class ControlModel, class ObservationModel>
struct fastslam<ControlModel, ObservationModel>
::state_feature_observer {
    observation_vector_type operator() (const Eigen::Matrix<double, state_dim+feature_dim, 1>& vec) const {
        state_type state = state_type::from_vector (vec.template head<state_dim>());
        feature_type feature = feature_type::from_vector (vec.template tail<feature_dim>());
        return ObservationModel::to_vector (-state + feature);
    }
};


template <class ControlModel, class ObservationModel>
struct fastslam<ControlModel, ObservationModel>
::feature_observer {
    const state_type& state;
    feature_observer (const state_type& state) : state(state) { }
    observation_vector_type operator() (const feature_vector_type& feature) const {
        return ObservationModel::to_vector (-state + feature_type::from_vector (feature));
    }
};


template <class ControlModel, class ObservationModel>
struct fastslam<ControlModel, ObservationModel>
::feature_initializer {
    const state_type& state;
    feature_initializer (const state_type& state) : state(state) { }
    feature_vector_type operator() (const observation_vector_type& obs) const {
        return (state + ObservationModel::from_vector(obs)).to_vector();
    }
};


template <class ControlModel, class ObservationModel>
struct fastslam<ControlModel, ObservationModel>::map_estimate_inserter {
    map_estimate_type& map;
    map_estimate_inserter (map_estimate_type& map) : map(map) { }
    void operator() (featureid_type feature_id, const feature_dist& estimate) {
        map.emplace_hint (map.end(), feature_id, estimate.mean());
    }
};


template <class ControlModel, class ObservationModel>
void fastslam<ControlModel, ObservationModel>
::control (timestep_type timestep, const ControlModel& control) {
    assert (timestep == current_timestep());
    assert (!current_control);
    current_control = control;
    ++timestep_counter;
}


template <class ControlModel, class ObservationModel>
void fastslam<ControlModel, ObservationModel>
    ::observation (timestep_type t, const typename slam_data<ControlModel, ObservationModel>::observation_data_type& obs) {
    assert (t == current_timestep());
    (obs.sequence() == 0 ? new_features : seen_features).emplace_back (obs.feature_id(), obs.observation());
}


template <class ControlModel, class ObservationModel>
void fastslam<ControlModel, ObservationModel>
::timestep (timestep_type timestep) {
    
    assert (timestep == current_timestep());

    // Update particle state
    
    if (timestep > 0) {

        if (particles.effective_size() < num_particles*resample_threshold) {
            particles.resample (random, num_particles);
        }

        assert (current_control);
        particles.update (boost::bind (&fastslam::particle_state_update, this, _1));
        current_control = boost::none;
        
        const state_type& state_estimate = particles.max_weight_particle().trajectory.state;
        
        if (keep_particle_trajectory) trajectory_estimate.reset();
        else trajectory_estimate->push_back (-trajectory_estimate->accumulate() + state_estimate);
    }
    
    map_estimate.reset();
    
    // Update particle features

    for (size_t i = 0; i < seen_features.size(); ++i) {

        const featureid_type feature_id = seen_features[i].first;
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
        
        const featureid_type feature_id = new_features[i].first;
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

    unscented_transform (ukf_params.control, current_control->vector_model(), state.vector_model(),
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
        particle.trajectory.previous = boost::make_shared<typename particle_type::state_list> (particle.trajectory);
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
boost::shared_ptr<const typename fastslam<ControlModel, ObservationModel>::trajectory_type>
fastslam<ControlModel, ObservationModel>::get_trajectory () const {
    
    if (!trajectory_estimate) {
        
        size_t i = current_control ? current_timestep()-1 : current_timestep();
        const typename particle_type::state_list* p = &particles.max_weight_particle().trajectory;

        trajectory_estimate = boost::make_shared<trajectory_type>(i);
        
        for (; i > 0 && p->previous; p = p->previous.get()) {
            (*trajectory_estimate)[--i] = -p->previous->state + p->state;
        }
    }
            
    return trajectory_estimate;
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
timestep_counter        (0)
{
    if (!keep_particle_trajectory) {
        trajectory_estimate = boost::make_shared<trajectory_type>();
    }
}
    
}

#endif
