#ifndef _PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP
#define _PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP

#include <cmath>
#include <functional>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <Eigen/Core>
#include <iostream>
#include <fstream>

#include "planar_robot/position.hpp"
#include "utility/random.hpp"
#include "utility/geometry.hpp"
#include "utility/nnls.hpp"
#include "slam/interfaces.hpp"

namespace planar_robot {
    
    struct range_bearing_model : public independent_normal_base<2, range_bearing_model> {
        
	using base_type = independent_normal_base<2, range_bearing_model>;
        
	using associated_type = position;
        
	range_bearing_model () = default;
        
	range_bearing_model (const vector_type& mean, const vector_type& stddev)
        : base_type(mean, stddev) { }
        
	static auto subtract (const vector_type& a, const vector_type& b) -> vector_type {
            return { a(0)-b(0), wrap_angle(a(1)-b(1)) };
	}
        
	static auto observe (const associated_type& pos) -> vector_type {
            return { pos.distance(), pos.direction() };
	}
        
        static auto inv_observe (const vector_type& obs) -> associated_type {
            return position::polar (obs(0), obs(1));
        }

        static auto obs_jacobian (const associated_type& pos) -> matrix_type {
            const double x = pos.x(), y = pos.y();
            double r = pos.distance(), rsq = pos.distance_squared();
            matrix_type jacobian;
            jacobian <<
                x/r,    y/r,
                -y/rsq, x/rsq;
            return jacobian;
        }

        static auto inv_obs_jacobian (const vector_type& obs) -> matrix_type {
            const double r = obs(0), theta = obs(1);
            const double cos_theta = std::cos(theta), sin_theta = std::sin(theta);
            matrix_type jacobian;
            jacobian <<
                cos_theta, -r*sin_theta,
                sin_theta, r*cos_theta;
            return jacobian;
        }
        
        auto more_accurate_than (const range_bearing_model& other) const -> bool {
            return std::abs(mean()(0)) < std::abs(other.mean()(0));
        }
        
        class proposal_dist {
            
            const range_bearing_model& model;
            
        public:
            
            using result_type = associated_type;
            static constexpr int vector_dim = range_bearing_model::vector_dim;
            
            explicit proposal_dist (const range_bearing_model& model) : model(model) { }
            
            auto operator() (random_source& random) const -> result_type { return inv_observe (model (random)); }
            auto likelihood (const result_type& x) const -> double { return model.likelihood (observe (x)); }
            auto log_likelihood (const result_type& x) const -> double { return model.log_likelihood (observe (x)); }
            auto initial_value (random_source&) const -> result_type { return inv_observe (model.mean()); }
        };
        
        auto proposal () const -> proposal_dist {
            return proposal_dist(*this);
        }
        
	class builder : public std::unary_function<vector_type, range_bearing_model> {
            
            const vector_type stddev;
            
	public:
            
            builder (double range_stddev, double bearing_stddev) : stddev(range_stddev, bearing_stddev) { }
            builder (const boost::program_options::variables_map&);
            
            static auto program_options () -> boost::program_options::options_description;
            
            auto operator() (const vector_type& observation) const -> range_bearing_model {
                return range_bearing_model (observation, stddev);
            }
	};
        
    };
    
    
    struct range_only_model : public independent_normal_base<1, range_only_model> {
        
        using base_type = independent_normal_base<1, range_only_model>;
        
        using associated_type = position;
        
        range_only_model () = default;
        
        range_only_model (const vector_type& mean, const vector_type& stddev) : base_type(mean, stddev) { }
        
	static auto subtract (const vector_type& a, const vector_type& b) -> vector_type {
            return a - b;
        }
        
        static auto observe (const associated_type& pos) -> vector_type {
            vector_type result;
            result(0) = pos.distance();
            return result;
        }
        
        auto more_accurate_than (const range_only_model& other) const -> bool {
            return std::abs(mean()(0)) < std::abs(other.mean()(0));
        }
        
        class proposal_dist {
            
            const range_only_model& model;
            
        public:
            
            using result_type = associated_type;
            static constexpr int vector_dim = range_only_model::vector_dim + 1;
            
            explicit proposal_dist (const range_only_model& model) : model(model) { }
            
            auto operator() (random_source& random) const -> result_type {
                using namespace boost::math::constants;
                return position::polar (model(random)(0), random.uniform()*2*pi<double>());
            }
            
            auto likelihood (const result_type& x) const -> double {
                using namespace boost::math::constants;
                return model.likelihood (observe (x)) / (2*pi<double>());
            }
            
            auto log_likelihood (const result_type& x) const -> double {
                using namespace boost::math::constants;
                return model.log_likelihood (observe (x)) - std::log (2*pi<double>());
            }
            
            auto initial_value (random_source& random) const -> result_type {
                using namespace boost::math::constants;
                return position::polar (model.mean()(0), random.uniform()*2*pi<double>());
            }
        };
        
        auto proposal () const -> proposal_dist {
            return proposal_dist(*this);
        }
        
        class builder : public std::unary_function<vector_type, range_only_model> {
            
            vector_type stddev;
            
        public:
            
            builder (double range_stddev) { stddev(0) = range_stddev; }
            
            builder (const boost::program_options::variables_map&);
            
            static auto program_options () -> boost::program_options::options_description;
            
            auto operator() (const vector_type& observation) const -> range_only_model {
                return range_only_model (observation, stddev);
            }
            
        };
        
        template <class ControlModel, class State>
        static auto learn_from_data (const slam::dataset<ControlModel, range_only_model>& dataset,
                                     const slam::slam_result<State, position>& ground_truth)
        -> builder;
        
    };
    
    template <class ControlModel, class State>
    auto range_only_model::
    learn_from_data (const slam::dataset<ControlModel, range_only_model>& dataset,
                     const slam::slam_result<State, position>& ground_truth) -> builder {
        
        const auto obs = dataset.observations();
        const auto num_obs = obs.size();
        
        Eigen::MatrixX2d observations (num_obs, 2);
        Eigen::VectorXd variance (num_obs);
        
        {
            std::ofstream debug_out ("learn_range_only_model.txt");
            
            observations.col(1).setOnes();
            for (std::size_t i = 0; i < num_obs; ++i) {
                const auto t = obs[i].first;
                const auto& obs_info = obs[i].second;
                const auto rel_pos = -ground_truth.get_state(t) + ground_truth.get_feature(obs_info.id);
                observations(i, 0) = std::abs(obs_info.observation(0));
                variance(i) = std::pow (rel_pos.distance() - obs_info.observation(0), 2.0);
                debug_out << observations(i,0) << ' ' << observations(i,1) << ' ' << variance(i) << '\n';
            }
        }
        Eigen::VectorXd learned = utility::nnls (observations, variance);
            
        std::cout << "Learned observation model: " << learned.transpose() << std::endl;
        return builder (std::sqrt (learned(1)));
    }
    
} // namespace planar_robot

#endif //_PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP
