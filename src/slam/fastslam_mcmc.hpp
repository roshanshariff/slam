//
//  fastslam_mcmc.hpp
//  slam
//
//  Created by Roshan Shariff on 12-03-06.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_fastslam_mcmc_hpp
#define slam_fastslam_mcmc_hpp

#include <iostream>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <Eigen/Eigen>

#include "slam/interfaces.hpp"
#include "slam/fastslam.hpp"
#include "slam/mcmc_slam.hpp"


namespace slam {
    
    template <class ControlModel, class ObservationModel>
    class fastslam_mcmc : public timestep_listener {
        
        using fastslam_type = fastslam<ControlModel, ObservationModel>;
        using mcmc_slam_type = mcmc_slam<ControlModel, ObservationModel>;
        
        boost::shared_ptr<fastslam_type> fastslam;
        boost::shared_ptr<mcmc_slam_type> mcmc_slam;
        
        typename fastslam_type::particle_type sample_particle () const;
        
        unsigned int mcmc_updates;
        unsigned int mcmc_feature_updates;
        unsigned int num_feature_samples;
        
        bool resample_required = false;
        
        std::size_t mcmc_slam_vertices () const {
            return mcmc_slam->state_estimates.size() + mcmc_slam->feature_estimates.size();
        }
        
    public:
        
        fastslam_mcmc (const fastslam_mcmc&) = delete;
        fastslam_mcmc& operator= (const fastslam_mcmc&) = delete;
        
        fastslam_mcmc (boost::program_options::variables_map& options);
        static boost::program_options::options_description program_options ();
        
        virtual void timestep (timestep_type) override;
        
        void set_fastslam (const decltype(fastslam)& p) { fastslam = p; }
        void set_mcmc_slam (const decltype(mcmc_slam)& p) { mcmc_slam = p; }
    };
        
}

template <class ControlModel, class ObservationModel>
void slam::fastslam_mcmc<ControlModel, ObservationModel>
::timestep (const timestep_type t) {
    
    if (mcmc_slam) {
        if (!fastslam) {
            auto vertices_before = mcmc_slam_vertices();
            mcmc_slam->timestep(t);
            auto vertices_after = mcmc_slam_vertices();
            unsigned int num_updates = mcmc_updates * (vertices_after - vertices_before);
            for (unsigned int i = 0; i < num_updates; ++i) mcmc_slam->update();
            return;
        }
    }
    else return;
    
    fastslam->timestep(t);
    if (t == 0) return;

    bool resample_required_previous = resample_required;
    resample_required = fastslam->resample_required();
    
    if (resample_required && resample_required_previous) {
    fastslam->get_trajectory();
        mcmc_slam->timestep(t);
        std::cout << "Reinitialising FastSLAM... ";
        fastslam->particles.reinitialize (fastslam->num_particles,
                                          boost::bind(&fastslam_mcmc::sample_particle, this));
        std::cout <<"done\n";
        resample_required = false;
    }
}


template <class ControlModel, class ObservationModel>
auto slam::fastslam_mcmc<ControlModel, ObservationModel>
::sample_particle () const -> typename fastslam_type::particle_type {
    
    auto num_updates = mcmc_updates;
    num_updates *= mcmc_slam->state_estimates.size() + mcmc_slam->feature_estimates.size();
    
    for (std::size_t i = 0; i < num_updates; ++i) {
        mcmc_slam->update();
    }
        
    typename fastslam_type::particle_type particle;
    
    for (std::size_t i = 0; i < mcmc_slam->get_trajectory().size(); ++i) {
        particle.trajectory.previous = boost::make_shared<decltype(particle.trajectory)>(particle.trajectory);
        particle.trajectory.state += mcmc_slam->get_trajectory()[i];
    }
    
    for (std::size_t feature_index = 0; feature_index < mcmc_slam->feature_estimates.size(); ++feature_index) {
        
        const auto& f = mcmc_slam->feature_estimates[feature_index];
        
        using feature_dist = typename fastslam_type::feature_dist;
        using feature_vector = typename feature_dist::vector_type;
        static const int feature_dim = fastslam_type::vec::feature_dim;
        
        const feature_vector base = (mcmc_slam->get_state(f.parent_timestep()) + f.estimate()).to_vector();
        Eigen::Matrix<double, fastslam_type::vec::feature_dim, Eigen::Dynamic> samples (feature_dim, num_feature_samples);
        
        for (unsigned int i = 0; i < num_feature_samples; ++i) {
            
            for (unsigned int j = 0; j < mcmc_feature_updates; ++j) {
                mcmc_slam->update (typename mcmc_slam_type::feature_edge (*mcmc_slam, feature_index), false);
            }
            
            feature_vector feature = (mcmc_slam->get_state(f.parent_timestep()) + f.estimate()).to_vector();
            samples.col(i) = feature_dist::subtract (base, feature);
        }
        
        const feature_vector base_innov = samples.rowwise().sum() / (1+num_feature_samples);
        
        for (unsigned int i = 0; i < num_feature_samples; ++i) {
            samples.col(i) = feature_dist::subtract (base_innov, samples.col(i));
        }
        
        samples /= std::sqrt(num_feature_samples);
        
        feature_dist feature;
        feature.vector_model().mean() = feature_dist::subtract (base, base_innov);
        feature.vector_model().chol_cov()
        = samples.transpose().householderQr().matrixQR()
        .template topLeftCorner<fastslam_type::vec::feature_dim, fastslam_type::vec::feature_dim>()
        .template triangularView<Eigen::Upper>()
        .transpose();
        
        particle.features.insert (f.id(), feature);
    }
    
    return particle;
}


template <class ControlModel, class ObservationModel>
auto slam::fastslam_mcmc<ControlModel, ObservationModel>
::program_options () -> boost::program_options::options_description {
    namespace po = boost::program_options;
    po::options_description options ("FastSLAM-MCMC Parameters");
    options.add_options()
    ("mcmc-updates", po::value<unsigned int>()->default_value(1), "MCMC iterations per graph vertex")
    ("mcmc-feature-updates", po::value<unsigned int>()->default_value(10), "MCMC iterations per feature")
    ("feature-samples", po::value<unsigned int>()->default_value(100), "number of independent samples of each feature");
    return options;
}


template <class ControlModel, class ObservationModel>
slam::fastslam_mcmc<ControlModel, ObservationModel>
::fastslam_mcmc (boost::program_options::variables_map& options)
: mcmc_updates (options["mcmc-updates"].as<unsigned int>()),
mcmc_feature_updates (options["mcmc-feature-updates"].as<unsigned int>()),
num_feature_samples (options["feature-samples"].as<unsigned int>())
{ }


#endif
