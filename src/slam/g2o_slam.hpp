//
//  g2o_slam.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-08-07.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_g2o_slam_hpp
#define slam_g2o_slam_hpp

#include "g2o/core/base_vertex.h"

namespace slam {
    
    
    template <class ControlModel, class ObservationModel>
    class g2o_slam :
    public slam_result_of<ControlModel, ObservationModel>,
    public slam_data<ControlModel, ObservationModel>::listener
    {
        
        using state_type = typename ControlModel::result_type;
        using feature_type = typename ObservationModel::result_type;
        
        using slam_data_type = slam_data<ControlModel, ObservationModel>;
        
        
        struct vertex_state : public g2o::BaseVertex<vec::state_dim, state_type> {
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW vertex_state ();
            
            virtual void setToOrigin () override {
                _estimate = state_type();
            }
            
            virtual void oplus (double* update) override {
                using map_type = Eigen::Map<state_type::vector_type>;
                _estimate += state_type::from_vector (map_type (update));
            }
        };
        
        
        struct vertex_landmark : public g2o::BaseVertex<feature_type::vector_dim, feature_type> {
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW vertex_landmark ();
            
            virtual void setToOrigin () override {
                _estimate = feature_type();
            }
            
            virtual void oplus (double* update) override {
                using map_type = Eigen::Map<feature_type::vector_type>;
                _estimate = _estimate::from_vector (_estimate.to_vector() + map_type(update));
            }
        };
        
        
        struct edge_control
        : public g2o::BaseBinaryEdge<ControlModel::vector_dim, state_type, vertex_state, vertex_state> {
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW edge_control ();
            
            void computeError () override {
                const vertex_state* v1 = static_cast<const vertex_state*> (_vertices[0]);
                const vertex_state* v2 = static_cast<const vertex_state*> (_vertices[1]);
                state_type delta = -v1->estimate() + v2->estimate();
                _error = ControlModel::subtract (ControlModel::to_vector (delta),
                                                 ControlModel::to_vector (_measurement));
            }
        };
        
        
        struct edge_obs
        : public g2o::BaseBinaryEdge<ObservationModel::vector_dim, feature_type, vertex_state, vertex_landmark> {
          
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW edge_obs ();
            
            void computeError () override {
                const vertex_state* v1 = static_cast<const vertex_state*> (_vertices[0]);
                const vertex_landmark* v2 = static_cast<const vertex_landmark*> (_vertices[1]);
                feature_type predicted = -v1->estimate() + v2->estimate();
                _error = ObservationModel::subtract (ObservationModel::to_vector (predicted),
                                                     ObservationModel::to_vector (_measurement));
            }
        };
        
        
        

        g2o::SparseOptimizer optimizer;
        int vertex_id_counter = 0;
        
        std::vector<int> state_vertex_ids;
        std::map<featureid_type, int> feature_vertex_ids;
        
        g2o_slam () {

            using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
            using SlamLinearSolver = LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>;
            
            SlamLinearSolver* linearSolver = new SlamLinearSolver();
            linearSolver->setBlockOrdering(false);
            
            SlamBlockSolver* solver = new SlamBlockSolver(&optimizer, linearSolver);
            optimizer.setSolver(solver);
            
            int initial_state_id = vertex_id_counter++;
            vertex_state* initial_state = new vertex_state;
            initial_state->setId(initial_state_id);
            initial_state->setToOrigin();
            initial_state->setFixed(true);
            
            optimizer.addVertex(initial_state);
            state_vertex_ids.push_back(initial_state_id);
            
        }
        
        // Overridden virtual member functions of slam::slam_data::listener
        
        virtual void control (timestep_type t, const ControlModel& control) override {

            assert (t == current_timestep());
            
            int prev_state_id = state_vertex_ids.back();
            int next_state_id = vertex_id_counter++;
            
            vertex_state* prev_state = optimizer.vertex (prev_state_id);
            vertex_state* next_state = new vertex_state;
            next_state->setId (next_state_id);
            next_state->setEstimate (prev_state->estimate + control.mean());

            optimizer.addVertex (next_state);
            state_vertex_ids.push_back (next_state_id);
            
            edge_control* e = new edge_control;
            e->vertices()[0] = prev_state;
            e->vertices()[1] = next_state;
            e->setMeasurement (control.mean());
            
            ControlModel::matrix_type cov = control.vector_model().chol_cov();
            cov *= cov.transpose();
            e->setInformation (cov.inverse());
            
            optimizer.addEdge (e);
        }
        
        virtual void observation (timestep_type t, const typename slam_data_type::observation_info& obs) override {
            assert (t == next_timestep);
            auto& features = obs.index() == 0 ? new_features : seen_features;
            features.emplace_back (obs.id(), obs.observation());
        }
        

    };
    
    
}

#endif
