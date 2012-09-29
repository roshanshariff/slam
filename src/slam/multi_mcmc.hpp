//
//  multi_mcmc.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-14.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_multi_mcmc_hpp
#define slam_multi_mcmc_hpp

#include <vector>
#include <memory>
#include <functional>
#include <utility>
#include <limits>
#include <fstream>

#include <boost/program_options.hpp>

#include "slam/interfaces.hpp"
#include "slam/mcmc_slam.hpp"
#include "slam/average_slam_result.hpp"
#include "utility/random.hpp"
#include "utility/bitree.hpp"
#include "utility/flat_map.hpp"
#include "utility/utility.hpp"

#include "main.hpp"


namespace slam {
    
    template <class ControlModel, class ObservationModel>
    class multi_mcmc : public slam_result_of<ControlModel, ObservationModel> {
        
        using slam_data_type = slam_data<ControlModel, ObservationModel>;

        using mcmc_slam_type = mcmc_slam<ControlModel, ObservationModel>;
        
        using state_type = typename ControlModel::associated_type;
        using feature_type = typename ObservationModel::associated_type;

        using slam_result_type = slam_result<state_type, feature_type>;

        using trajectory_type = utility::bitree<state_type>;
        using feature_map_type = utility::flat_map<featureid_type, feature_type>;
        
        std::vector<std::unique_ptr<mcmc_slam_type>> mcmc_chains;
        mcmc_slam_type* max_likelihood;
        
        unsigned int num_updates = 0;
        unsigned int num_accepted = 0;
        
        unsigned int mcmc_end_steps;
        
    public:
        
        multi_mcmc (std::shared_ptr<const slam_data_type>, boost::program_options::variables_map& options, unsigned int seed);
        
        static auto program_options () -> boost::program_options::options_description;
        
        auto num_chains () const -> unsigned int {
            return mcmc_chains.size();
        }
        
        void update (unsigned int count);
        
        auto get_log_likelihood () const -> double {
            return max_likelihood->get_log_likelihood();
        }
        
        auto get_average () -> std::unique_ptr<slam_result_type> {
            return average_slam_result<state_type, feature_type> (mcmc_chains.begin(), mcmc_chains.end());
        }
        
        // Overridden virtual member functions of slam::slam_result
        
        virtual void timestep (timestep_type t) override {
            for (const auto& mcmc : mcmc_chains) {
                mcmc->timestep (t);
            }
        }
        
        virtual auto current_timestep () const override -> timestep_type {
            return max_likelihood->current_timestep();
        }
        
        virtual auto get_state (timestep_type t) const override -> state_type {
            return max_likelihood->get_state (t);
        }
        
        virtual auto get_feature (featureid_type id) const override -> feature_type {
            return max_likelihood->get_feature (id);
        }
        
        virtual auto get_trajectory () const override -> const trajectory_type& {
            return max_likelihood->get_trajectory();
        }
        
        virtual auto get_feature_map () const override -> const feature_map_type& {
            return max_likelihood->get_feature_map();
        }
        
        virtual void completed () override;
    
    };
    
}


template <class ControlModel, class ObservationModel>
void slam::multi_mcmc<ControlModel, ObservationModel>
::update (unsigned int count) {
    for (const auto& mcmc : mcmc_chains) {
        for (unsigned int i = 0; i < count; ++i) {
            bool accepted = mcmc->update();
            if (accepted) ++num_accepted;
            ++num_updates;
        }
        if (mcmc->get_log_likelihood() > max_likelihood->get_log_likelihood()) {
            max_likelihood = mcmc.get();
        }
    }
}


template <class ControlModel, class ObservationModel>
void slam::multi_mcmc<ControlModel, ObservationModel>
::completed () {
    std::ofstream report ("multi-mcmc-report.txt");
    unsigned int remaining_steps = mcmc_end_steps;
    const unsigned int report_every = 100;
    while (remaining_steps > 0) {
        if (remaining_steps > report_every) {
            update (report_every);
            remaining_steps -= report_every;
        }
        else {
            update (remaining_steps);
            remaining_steps = 0;
        }
        report
        << ((double)num_updates / num_chains()) << '\t'
        << ((double)num_accepted / num_chains()) << '\t'
        << get_log_likelihood() << '\n';
    }
}


template <class ControlModel, class ObservationModel>
auto slam::multi_mcmc<ControlModel, ObservationModel>
::program_options () -> boost::program_options::options_description {
    namespace po = boost::program_options;
    po::options_description options ("Multi-MCMC Parameters");
    options.add_options()
    ("multi-mcmc-chains", po::value<unsigned int>()->default_value(100), "Number of MCMC chains")
    ("multi-mcmc-end-steps", po::value<unsigned int>()->default_value(0), "MCMC iterations after simulation")
    ("multi-mcmc-seed", po::value<unsigned int>(), "MCMC-SLAM random seed");
    return options;
}


template <class ControlModel, class ObservationModel>
slam::multi_mcmc<ControlModel, ObservationModel>
::multi_mcmc (std::shared_ptr<const slam_data<ControlModel, ObservationModel>> data,
              boost::program_options::variables_map& options, unsigned int seed)
: mcmc_end_steps (options["multi-mcmc-end-steps"].as<unsigned int>())
{

    const unsigned int num_mcmc_chains = options["multi-mcmc-chains"].as<unsigned int>();
    random_source random (remember_option (options, "multi-mcmc-seed", seed));
    
    for (unsigned int i = 0; i < num_mcmc_chains; ++i) {
        mcmc_chains.push_back (make_unique<mcmc_slam_type> (data, options, random()));
    }
    
    max_likelihood = mcmc_chains.front().get();
}


extern template class slam::multi_mcmc<control_model_type, observation_model_type>;

#endif
