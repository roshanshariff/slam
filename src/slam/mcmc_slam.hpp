#ifndef _SLAM_MCMC_SLAM_HPP
#define _SLAM_MCMC_SLAM_HPP

#include <cassert>
#include <cmath>
#include <utility>
#include <algorithm>

#include <boost/container/vector.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/program_options.hpp>

#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"
#include "utility/random.hpp"
#include "utility/bitree.hpp"
#include "utility/utility.hpp"

namespace slam {
    
    /** This class implements the MCMC SLAM algorithm. To use it, construct an instance passing in
     a reference to a slam_data object and a random generator. This class will register to listen
     to slam data events. Call mcmc_slam::update to perform the specified number of MCMC-SLAM
     iterations on the data available so far. */
    
    template <class ControlModel, class ObservationModel>
    class mcmc_slam :
    public slam_result<typename ControlModel::result_type, typename ObservationModel::result_type>,
    public boost::enable_shared_from_this<mcmc_slam<ControlModel, ObservationModel>>
    {
        
        typedef slam_data<ControlModel, ObservationModel> slam_data_type;
        
        /** The types of labels for action and observation edges, respectively. Action edge labels must
         form a group, with the binary + operator and unary - to form inverses. Additionally, there
         must be a defined addition action_type + observation type which gives the observation as
         seen from the given pose. */
        typedef typename ControlModel::result_type control_type;
        typedef typename ObservationModel::result_type observation_type;
        
        /** All the information about each feature for MCMC SLAM. Stores a reference to the observations
         of this feature from slam_data, an estimate for this feature, and the time step relative
         to which the estimate is computed. */
        class feature_estimate {
            
            const typename slam_data_type::feature_data_type* data;
            
        public:
            
            featureid_type id;
            timestep_type parent_timestep;
            observation_type estimate;
            
            feature_estimate (featureid_type id, timestep_type t, const observation_type& est,
                              const typename slam_data_type::feature_data_type& data)
            : id(id), parent_timestep(t), estimate(est), data(&data) { }
            
            const typename slam_data_type::feature_data_type& feature_data () const { return *data; }
            
        };
        
        typedef utility::bitree<control_type> trajectory_type;
        typedef boost::container::flat_map<featureid_type, observation_type> map_estimate_type;
        
        const boost::shared_ptr<const slam_data_type> data;
        
        /** Our very own pseudo-random number generator. */
        mutable random_source random;
        
        /** MCMC Constants. mcmc_steps is the number of iterations per update. The action and observation
         dimensions are a measure of the number of independent parameters represented by each type of
         edge. */
        unsigned int mcmc_steps;
        double control_edge_importance;
        double observation_edge_importance;
        
        /** The current action edge labels and the corresponding edge weights. */
        trajectory_type state_estimates;
        utility::bitree<double> state_weights;
        
        /** The current feature edge labels and the corresponding edge weights. */
        boost::container::vector<feature_estimate> feature_estimates;
        utility::bitree<double> feature_weights;
        
    public:
        
        mcmc_slam (boost::shared_ptr<const slam_data_type>, boost::program_options::variables_map& options, unsigned int seed);
        
        static boost::program_options::options_description program_options ();
        
        // Overridden virtual member functions of slam::timestep_listener
        
        virtual void timestep (timestep_type t) override;
        
        // Overridden virtual member functions of slam::data_source
        
        virtual timestep_type current_timestep () const override { return timestep_type(state_estimates.size()); }
        
        // Overridden virtual member functions of slam::slam_result
        
        virtual control_type get_state () const override { return state_estimates.accumulate(); }
        
        virtual boost::shared_ptr<const trajectory_type> get_trajectory () const override {
            return boost::shared_ptr<const trajectory_type> (this->shared_from_this(), &state_estimates);
        }
        
        virtual boost::shared_ptr<const map_estimate_type> get_map () const override;
        
    private:
        
        void update ();
        void update_trajectory (timestep_type);
        double state_change (timestep_type) const;
        void update_feature (size_t);
        
        /** Calculates the edge weight given the probability distribution and label. Uses the formula
         p_e(x) = k * J_e(x)^(-1/k) where p_e is the edge weight, x is the edge label, and J_e is
         the probability density of the label by its own distribution. k is a parameter expressing
         the number of independent dimensions in the probability distribution. */
        
        double edge_weight (const ControlModel& model, const control_type& label) {
            return control_edge_importance * std::exp(-model.log_likelihood(label)/control_edge_importance);
        }
        
        double edge_weight (const ObservationModel& model, const observation_type& label) {
            return observation_edge_importance * std::exp(-model.log_likelihood(label)/observation_edge_importance);
        }
        
    };
    
    template <class ControlModel, class ObservationModel>
    void mcmc_slam<ControlModel, ObservationModel>
    ::timestep (timestep_type t) {
        
        auto obs_range = data->observations_at (t);
        
        for (auto iter = obs_range.first; iter != obs_range.second; ++iter) {
            const auto& obs = iter->second;
            if (obs.sequence() == 0) {
                std::cout << "mcmc_slam: adding feature " << obs.feature_id() << '\n';
                observation_type initial_estimate = obs.observation().mean();
                feature_estimates.emplace_back (obs.feature_id(), t, initial_estimate, obs.feature_data());
                feature_weights.push_back (edge_weight (obs.observation(), initial_estimate));
            }
        }
        
        if (t > 0) {

            const ControlModel& control = data->control (t-1);
            
            // Use the mean of the distribution as the initial estimate.
            control_type initial_estimate = control.mean();
            state_estimates.push_back (initial_estimate);
            
            // Compute the edge weight based on the distribution and the initial estimate.
            state_weights.push_back (edge_weight (control, initial_estimate));

            assert (state_estimates.size() == t);
            assert (state_weights.size() == t);
            
            for (int i = 0; i < mcmc_steps; ++i) update();
        }
    }
    
    
    // Performs the MCMC SLAM update step
    template <class ControlModel, class ObservationModel>
    void mcmc_slam<ControlModel, ObservationModel>
    ::update () {
        
        // The total action and feature edge weights.
        const double state_weight = state_weights.accumulate();
        const double feature_weight = feature_weights.accumulate();
        
        if (state_weight == 0 && feature_weight == 0) return;
        
        // Randomly select an action or a feature by its weight.
        double state_range, feature_range;
        do {
            state_range = random.uniform() * (state_weight + feature_weight);
            feature_range = state_range - state_weight;
        }
        while (!(state_range < state_weight || feature_range < feature_weight));
        
        if (state_range < state_weight) {
            // Identify the time step in whose interval the randomly selected number lies.
            update_trajectory (timestep_type (state_weights.binary_search (state_range)));
        }
        else if (feature_weight > 0) {
            // Identify the feature in whose interval the randomly selected number lies.
            update_feature (feature_weights.binary_search (feature_range));
        }
    }
    
    
    /** Resamples the action edge given by id, computes the corresponding acceptance probability,
     and either accepts or rejects the change. */
    template <class ControlModel, class ObservationModel>
    void mcmc_slam<ControlModel, ObservationModel>
    ::update_trajectory (const timestep_type timestep) {
        
        assert (timestep < state_estimates.size());
        
        // Generate a new estimate for this action as a random sample from the associated distribution,
        // and compute the edge weight for this new estimate.
        const control_type new_estimate = data->control(timestep)(random);
        const double new_weight = edge_weight(data->control(timestep), new_estimate);
        
        // The old action estimate and corresponding edge weight.
        const control_type old_estimate = state_estimates[timestep];
        const double old_weight = state_weights[timestep];
        
        double acceptance_probability = 0.0; // log of the acceptance probability.
        
        // Subtract the log probability of all affected edges, before the action edge is updated.
        acceptance_probability -= state_change(timestep);
        
        // Update the action edge to the new estimate.
        state_estimates[timestep] = new_estimate;
        state_weights[timestep] = new_weight;
        
        // Add the log probability of all edges affected, after the action edge is updated.
        acceptance_probability += state_change(timestep);
        
        // Compute the acceptance probability as the ratio of probabilities after and before the edge
        // was modified, multiplied by the ratio of the new and old edge weight.
        acceptance_probability = std::exp(acceptance_probability) * new_weight / old_weight;
        
        // If the change was not accepted, roll back the changes by reverting to the old estimate and weight.
        if (random.uniform() >= acceptance_probability) {
            state_estimates[timestep] = old_estimate;
            state_weights[timestep] = old_weight;
        }
    }
    
    
    /** Computes the log probability of all the edges whose labels change when the action edge given by
     action_id is updated. Changing an action splits the spanning tree of the inference graph into
     two subtrees, T1 and T2. T1 is the tree that contains action 0. A feature vertex lies in T1 if
     its parent action is before the one being changed, and it lies in T2 otherwise. If a feature vertex
     lies in T1, then the observation edges affected by the change are those after the change. Otherwise,
     if a feature vertex lies in T2 then the observations made before the change are affected. */
    template <class ControlModel, class ObservationModel>
    double mcmc_slam<ControlModel, ObservationModel>
    ::state_change (const timestep_type timestep) const {
        
        double result = 0.0; // Initialise log probability to zero.
        
        for (size_t i = 0; i < feature_estimates.size(); ++i) { // iterate over all observed features.
            
            const feature_estimate& feature = feature_estimates[i];
            
            // the range of observations to consider
            typename slam_data_type::feature_data_type::const_iterator j, end;
            
            if (feature.parent_timestep <= timestep) {
                // The feature edge lies in T1 since its parent action is before the modified action edge.
                // The observations to consider are all those in T2, i.e. after the modified action.
                j = feature.feature_data().upper_bound(timestep);
                end = feature.feature_data().end();
            }
            else {
                // The feature edge lies in T2 since its parent action is after the modified action edge.
                // The features to consider are all those in T1, i.e. before the modified action.
                j = feature.feature_data().begin();
                end = feature.feature_data().upper_bound(timestep);
            }
            
            // The position of the feature, stored as an observation relative to the observation_base.
            // The initial position is stored as the estimate relative to the parent action.
            observation_type observation = feature.estimate;
            timestep_type observation_base = feature.parent_timestep;
            
            for (; j != end; ++j) { // iterate over observations of this feature.
                
                // Each observation is relative to a different action. Computes the state change between the old
                // base of observation (which is initially the parent action) and the current base of the
                // observation.
                const control_type state_change = state_estimates.accumulate (observation_base, j->first);
                observation_base = j->first;
                
                // Retrieve the distribution associated with this observation edge.
                const ObservationModel& distribution = j->second;
                
                // Rebase the observation relative to the current action, and accumulate its log likelihood
                // according to the associated distribution.
                observation = -state_change + observation;
                result += distribution.log_likelihood(observation);
            }
        }
        
        return result;
    }
    
    
    /** Resamples the observation edge given by id, computes the corresponding acceptance probability,
     and either accepts or rejects the change. Changing an observation edge splits the spanning tree
     of the inference graph into two trees, T1 and T2. T1 contains all the action vertices and all other
     features, whereas T2 contains only the feature being updated. The affected edges are all observations
     of the feature whose estimate is being modified. */
    template <class ControlModel, class ObservationModel>
    void mcmc_slam<ControlModel, ObservationModel>
    ::update_feature (size_t feature_index) {
        
        feature_estimate& feature = feature_estimates[feature_index];
        
        // Find the associated distribution corresponding to the observation from the parent action of
        // this feature.
        const ObservationModel& distribution = feature.feature_data().at(feature.parent_timestep);
        
        // Compute a new estimate of this feature's position as a random sample from the associated distribution,
        // and compute a new edge weight accordingly.
        const observation_type new_estimate = distribution(random);
        const double new_feature_weight = edge_weight(distribution, new_estimate);
        
        // These variables will store the new and old estimates of the feature's position, computed relative to
        // some action. Initially, they are relative to the parent action of the feature.
        observation_type new_observation = new_estimate;
        observation_type old_observation = feature.estimate;
        timestep_type observation_base = feature.parent_timestep;
        
        double acceptance_probability = 0.0; // log of the acceptance probability.
        
        // Iterate over all observations of this feature.
        typename slam_data_type::feature_data_type::const_iterator i = feature.feature_data().begin();
        for (; i != feature.feature_data().end(); ++i) {
            
            // Ignore the observation edge that is in the spanning tree of the inference graph.
            if (i->first == feature.parent_timestep) continue;
            
            // Each observation is relative to a different action. Computes the state change between the old
            // base of observation (which is initially the parent action) and the current base of the
            // observation.
            const control_type state_change = state_estimates.accumulate (observation_base, i->first);
            observation_base = i->first;
            
            // Retrieve the distribution associated with this observation edge.
            const ObservationModel& distribution = i->second;
            
            // Update the new observation by rebasing it relative to the current action and multiply the
            // corresponding probability to the running total (i.e. add the log probability).
            new_observation = -state_change + new_observation;
            acceptance_probability += distribution.log_likelihood(new_observation);
            
            // Update the old observation by rebasing it relative to the current action and divide the
            // corresponding probability from the running total (i.e. subtract the log probability).
            old_observation = -state_change + old_observation;
            acceptance_probability -= distribution.log_likelihood(old_observation);
        }
        
        // Compute the acceptance probability as the ratio of probabilities of the new and the old estimates,
        // multiplied by the ratio between the new and the old edge weights.
        acceptance_probability = std::exp(acceptance_probability) * new_feature_weight / feature_weights[feature_index];
        
        // If the change was accepted, replace the old estimate and edge weight by the new ones.
        if (random.uniform() < acceptance_probability) {
            feature.estimate = new_estimate;
            feature_weights[feature_index] = new_feature_weight;
        }
    }
    
    
    /** Returns current estimates of the positions of all observed features. */
    template <class ControlModel, class ObservationModel>
    boost::shared_ptr<const typename mcmc_slam<ControlModel, ObservationModel>::map_estimate_type>
    mcmc_slam<ControlModel, ObservationModel>::get_map () const {
        
        boost::container::vector<typename map_estimate_type::value_type> estimates;
        estimates.reserve (feature_estimates.size());
        
        for (const auto& f : feature_estimates) { // iterate through each feature
            estimates.emplace_back (f.id, state_estimates.accumulate(f.parent_timestep)+f.estimate);
        }
        
        std::sort (estimates.begin(), estimates.end(), pair_compare_first<featureid_type>());
        
        return boost::make_shared<const map_estimate_type> (boost::container::ordered_unique_range,
                                                            estimates.begin(), estimates.end());
    }
    
    
    template <class ControlModel, class ObservationModel>
    boost::program_options::options_description mcmc_slam<ControlModel, ObservationModel>
    ::program_options () {
        namespace po = boost::program_options;
        po::options_description options ("MCMC-SLAM Parameters");
        options.add_options()
        ("mcmc-steps", po::value<unsigned int>()->default_value(1), "MCMC iterations per simulation step")
        ("control-edge-importance", po::value<double>()->default_value(ControlModel::vector_dim), "degrees of freedom for control edges")
        ("observation-edge-importance", po::value<double>()->default_value(ObservationModel::vector_dim), "degrees of freedom for observation edges")
        ("mcmc-slam-seed", po::value<unsigned int>(), "MCMC-SLAM random seed");
        return options;
    }
    
    
    template <class ControlModel, class ObservationModel>
    mcmc_slam<ControlModel, ObservationModel>
    ::mcmc_slam (boost::shared_ptr<const slam_data<ControlModel, ObservationModel>> data,
                 boost::program_options::variables_map& options, unsigned int seed)
    : data                     (data),
    random                     (remember_option (options, "mcmc-slam-seed", seed)),
    mcmc_steps                  (options["mcmc-steps"].as<unsigned int>()),
    control_edge_importance     (options["control-edge-importance"].as<double>()),
    observation_edge_importance (options["observation-edge-importance"].as<double>()) { }
    
} // namespace slam


#endif //_SLAM_MCMC_SLAM_HPP
