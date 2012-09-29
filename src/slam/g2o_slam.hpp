//
//  g2o_slam.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-08-07.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_g2o_slam_hpp
#define slam_g2o_slam_hpp

#include <memory>
#include <iostream>

#include <boost/program_options.hpp>

#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"
#include "utility/bitree.hpp"
#include "utility/random.hpp"
#include "utility/utility.hpp"

#include "main.hpp"


namespace slam {    
    
    template <class ControlModel, class ObservationModel>
    class g2o_slam :
    public slam_result_of<ControlModel, ObservationModel>,
    public slam_data<ControlModel, ObservationModel>::listener
    {
        
        using state_type = typename ControlModel::associated_type;
        using feature_type = typename ObservationModel::associated_type;
        using control_type = typename ControlModel::vector_type;
        using observation_type = typename ObservationModel::vector_type;
        
        using slam_result_type = slam_result<state_type, feature_type>;
        using slam_data_type = slam_data<ControlModel, ObservationModel>;
        
        /** Nested types */
        
        struct vertex_state : public g2o::BaseVertex<state_type::vector_dim, state_type> {
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            
            vertex_state () { }
            
            vertex_state (int id, const state_type& estimate) {
                this->setId (id);
                this->setEstimate (estimate);
            }
            
            virtual bool read (std::istream&) { return true; }
            virtual bool write (std::ostream&) const { return true; }
            
        protected:
            
            virtual void setToOriginImpl () override {
                this->setEstimate (state_type());
            }
            
            virtual void oplusImpl (const double* update) override {
                using map_type = Eigen::Map<const typename state_type::vector_type>;
                this->setEstimate(this->estimate() + state_type::from_vector (map_type (update)));
            }
        };
        
        
        struct vertex_landmark : public g2o::BaseVertex<feature_type::vector_dim, feature_type> {
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            
            vertex_landmark () { }
            
            vertex_landmark (int id, const feature_type& estimate) {
                this->setId (id);
                this->setEstimate (estimate);
            }
            
            virtual bool read (std::istream&) { return true; }
            virtual bool write (std::ostream&) const { return true; }
            
        protected:
            
            virtual void setToOriginImpl () override {
                this->setEstimate (feature_type());
            }
            
            virtual void oplusImpl (const double* update) override {
                using map_type = Eigen::Map<const typename feature_type::vector_type>;
                this->setEstimate (feature_type::from_vector (this->estimate().to_vector() + map_type(update)));
            }
        };
        
        
        struct edge_control
        : public g2o::BaseBinaryEdge<ControlModel::vector_dim, control_type, vertex_state, vertex_state> {
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            edge_control (vertex_state* v0, vertex_state* v1, const control_type& m) {
                this->setVertex (0, v0);
                this->setVertex (1, v1);
                this->setMeasurement (m);
            }
            
            virtual bool read (std::istream&) { return true; }
            virtual bool write (std::ostream&) const { return true; }
            
        protected:
            
            void computeError () override {
                const vertex_state* v1 = static_cast<const vertex_state*> (this->vertex(0));
                const vertex_state* v2 = static_cast<const vertex_state*> (this->vertex(1));
                this->error() = ControlModel::subtract (ControlModel::observe (-v1->estimate() + v2->estimate()),
                                                        this->measurement());
            }
        };
        
        
        struct edge_obs
        : public g2o::BaseBinaryEdge<ObservationModel::vector_dim, observation_type, vertex_state, vertex_landmark> {
          
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            
            edge_obs (vertex_state* v0, vertex_landmark* v1, const observation_type& m) {
                this->setVertex (0, v0);
                this->setVertex (1, v1);
                this->setMeasurement (m);
            }
            
            virtual bool read (std::istream&) { return true; }
            virtual bool write (std::ostream&) const { return true; }
            
        protected:
            
            void computeError () override {
                const vertex_state* v1 = static_cast<const vertex_state*> (this->vertex(0));
                const vertex_landmark* v2 = static_cast<const vertex_landmark*> (this->vertex(1));
                feature_type predicted = -v1->estimate() + v2->estimate();
                this->error() = ObservationModel::subtract (ObservationModel::observe (-v1->estimate() + v2->estimate()),
                                                            this->measurement());
            }
        };
                
        
        /** Private data members */
        
        random_source random;
        
        unsigned int g2o_steps = 0;
        unsigned int g2o_end_steps = 0;
        bool need_init = false;
        
        g2o::SparseOptimizer optimizer;
        int next_vertex_id = 0;
        
        std::vector<vertex_state*> state_vertices;
        std::map<featureid_type, vertex_landmark*> feature_vertices;
        
        mutable utility::bitree<state_type> trajectory_estimate;
        mutable utility::flat_map<featureid_type, feature_type> map_estimate;
        
        timestep_type next_timestep;
        
    public:
        
        explicit g2o_slam (unsigned int seed);
        g2o_slam (boost::program_options::variables_map&, unsigned int seed);
        static boost::program_options::options_description program_options ();
        
        void reinitialise (const slam_result_type&);
        
        void optimise (unsigned int iterations);
        
        // Overridden virtual member functions of slam::slam_data::listener

        virtual void control (timestep_type t, const ControlModel& control) override;
        virtual void observation (timestep_type t, const typename slam_data_type::observation_info& obs) override;
        
        virtual void timestep (timestep_type) override;
        virtual void completed () override;
        
        // Overridden virtual member functions of slam::slam_result
        
        virtual timestep_type current_timestep () const override {
            assert (next_timestep > 0);
            return next_timestep - 1;
        }
        
        virtual state_type get_state (timestep_type t) const override {
            return state_vertices.at(t)->estimate();
        }
        
        virtual feature_type get_feature (featureid_type id) const override {
            return feature_vertices.at(id)->estimate();
        }
        
        virtual const decltype(trajectory_estimate)& get_trajectory () const override;
        
        virtual const decltype(map_estimate)& get_feature_map () const override;
        
    };
    
} // namespace slam


template <class ControlModel, class ObservationModel>
void slam::g2o_slam<ControlModel, ObservationModel>
::reinitialise (const slam_result_type& initialiser) {
    
    const auto& trajectory = initialiser.get_trajectory();
    
    for (timestep_type t (1); t <= trajectory.size(); ++t) {
        state_vertices.at(t)->setEstimate (trajectory.accumulate (t));
    }
    
    for (const auto& feature : initialiser.get_feature_map()) {
        feature_vertices.at(feature.first)->setEstimate (feature.second);
    }
    
    need_init = true;
}


template <class ControlModel, class ObservationModel>
void slam::g2o_slam<ControlModel, ObservationModel>
::optimise (unsigned int iterations) {

    if (iterations > 0) {
        
        if (need_init) {
            optimizer.initializeOptimization();
            need_init = false;
        }
        optimizer.optimize(iterations);
        
        trajectory_estimate.clear();
        map_estimate.clear();
    }
}


template <class ControlModel, class ObservationModel>
void slam::g2o_slam<ControlModel, ObservationModel>
::control (timestep_type t, const ControlModel& control) {
    
    assert (t == current_timestep());
    assert (t == state_vertices.size()-1);
    
    vertex_state* v0 = state_vertices.back();
    
    state_type initial_estimate = v0->estimate() + control.proposal().initial_value(random);
    auto v1 = make_unique<vertex_state>(next_vertex_id++, initial_estimate);
    state_vertices.push_back (v1.get());
    
    auto e = make_unique<edge_control>(v0, v1.get(), control.mean());
    
    typename ControlModel::matrix_type chol_cov_inv;
    chol_cov_inv.setIdentity();
    control.chol_cov().template triangularView<Eigen::Lower>().solveInPlace(chol_cov_inv);
    e->setInformation (chol_cov_inv.transpose() * chol_cov_inv);
    
    optimizer.addVertex (v1.release());
    optimizer.addEdge (e.release());
    
    need_init = true;
}    


template <class ControlModel, class ObservationModel>
void slam::g2o_slam<ControlModel, ObservationModel>
::observation (timestep_type t, const typename slam_data_type::observation_info& obs) {
    
    assert (t == next_timestep);
    assert (t == state_vertices.size()-1);
    
    vertex_state* v0 = state_vertices.back();
    vertex_landmark* v1 = feature_vertices[obs.id()];
    
    if (obs.index() == 0) {
        
        feature_type initial_estimate = v0->estimate() + obs.observation().proposal().initial_value (random);
        auto v = make_unique<vertex_landmark>(next_vertex_id++, initial_estimate);
        feature_vertices[obs.id()] = v.get();
        
        v1 = v.get();
        optimizer.addVertex (v.release());
    }
    
    auto e = make_unique<edge_obs>(v0, v1, obs.observation().mean());
    
    typename ObservationModel::matrix_type chol_cov_inv;
    chol_cov_inv.setIdentity();
    obs.observation().chol_cov().template triangularView<Eigen::Lower>().solveInPlace(chol_cov_inv);
    e->setInformation (chol_cov_inv.transpose() * chol_cov_inv);
    
    optimizer.addEdge (e.release());
    
    need_init = true;
}


template <class ControlModel, class ObservationModel>
void slam::g2o_slam<ControlModel, ObservationModel>
::timestep (timestep_type t) {
    
    if (t < next_timestep) return;
    assert (t == next_timestep);
    
    if (t > 0) optimise (g2o_steps);
    
    ++next_timestep;    
}


template <class ControlModel, class ObservationModel>
void slam::g2o_slam<ControlModel, ObservationModel>
::completed () {
    
    optimise (g2o_end_steps);
}


template <class ControlModel, class ObservationModel>
auto slam::g2o_slam<ControlModel, ObservationModel>
::get_trajectory () const -> const decltype(trajectory_estimate)& {
    
    if (trajectory_estimate.size() != current_timestep()) {
        
        trajectory_estimate.clear();
        trajectory_estimate.reserve (current_timestep());
        
        for (timestep_type t; t < current_timestep(); ++t) {
            
            trajectory_estimate.push_back (-get_state(t) + get_state(t+1));
        }
    }
    
    assert (trajectory_estimate.size() == current_timestep());
    return trajectory_estimate;
}


template <class ControlModel, class ObservationModel>
auto slam::g2o_slam<ControlModel, ObservationModel>
::get_feature_map () const -> const decltype(map_estimate)& {
    
    if (map_estimate.size() != feature_vertices.size()) {
        
        map_estimate.clear();
        map_estimate.reserve (feature_vertices.size());
        
        for (const auto& entry : feature_vertices) {
            map_estimate.emplace_hint (map_estimate.end(), entry.first, entry.second->estimate());
        }
    }
    
    assert (map_estimate.size() == feature_vertices.size());
    return map_estimate;
}

template <class ControlModel, class ObservationModel>
auto slam::g2o_slam<ControlModel, ObservationModel>
::program_options () -> boost::program_options::options_description {
    namespace po = boost::program_options;
    po::options_description options ("G2O-SLAM Parameters");
    options.add_options()
    ("g2o-steps", po::value<unsigned int>()->default_value(1), "MCMC iterations per graph vertex")
    ("g2o-end-steps", po::value<unsigned int>()->default_value(0), "MCMC iterations after simulation");
    return options;
}


template <class ControlModel, class ObservationModel>
slam::g2o_slam<ControlModel, ObservationModel>
::g2o_slam (unsigned int seed) : random (seed)
{
    
    using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using SlamLinearSolver = g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>;
    using OptimizationAlgorithm = g2o::OptimizationAlgorithmGaussNewton;
    
    auto linearSolver = make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    
    auto blockSolver = make_unique<SlamBlockSolver>(linearSolver.release());
    
    auto solver = make_unique<OptimizationAlgorithm>(blockSolver.release());
    optimizer.setAlgorithm(solver.release());
    
    auto v = make_unique<vertex_state>(next_vertex_id++, state_type());
    v->setFixed (true);
    
    state_vertices.push_back (v.get());
    optimizer.addVertex (v.release());
}


template <class ControlModel, class ObservationModel>
slam::g2o_slam<ControlModel, ObservationModel>
::g2o_slam (boost::program_options::variables_map& options, unsigned int seed) : g2o_slam (seed)
{
    g2o_steps = options["g2o-steps"].as<unsigned int>();
    g2o_end_steps = options["g2o-end-steps"].as<unsigned int>();
}


extern template class slam::g2o_slam<control_model_type, observation_model_type>;

#endif
