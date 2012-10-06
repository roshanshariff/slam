#ifndef _PLANAR_ROBOT_VELOCITY_MODEL_HPP
#define _PLANAR_ROBOT_VELOCITY_MODEL_HPP

#include <cstdlib>
#include <cmath>
#include <functional>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>

#include "planar_robot/pose.hpp"
#include "utility/random.hpp"
#include "utility/geometry.hpp"


namespace planar_robot {
    
    
    struct velocity_slip_model : public independent_normal_base<3, velocity_slip_model> {
        
	using base_type = independent_normal_base<3, velocity_slip_model>;
	using associated_type = pose;
        
	velocity_slip_model () { }
	velocity_slip_model (const vector_type& mean, const vector_type& stddev) : base_type(mean, stddev) { }
        
	static auto subtract (const vector_type& a, const vector_type& b) -> vector_type {
            return a - b;
        }
        
	static auto observe (const associated_type& dp) -> vector_type {
            
            const double r = 0.5 * dp.distance_squared() / dp.y();
            
            if (std::isfinite(r)) {
                const double theta = r >= 0 ? std::atan2 (dp.x(), r - dp.y()) : std::atan2 (-dp.x(), dp.y() - r);
                return { r*theta, theta, wrap_angle(dp.bearing()-theta) };
            }
            else {
                return { dp.x(), 0.0, dp.bearing() };
            }
	}
        
	static auto inv_observe (const vector_type& control) -> associated_type {
            
            const double v = control(0), w = control(1), g = control(2), r = v/w;
            
            if (std::isfinite(r)) {
                return pose::cartesian (r*std::sin(w), r-r*std::cos(w), w+g);
            }
            else {
                return pose::cartesian (v, 0.0, g);
            }
	}
        
        class proposal_dist {
            
            const velocity_slip_model& model;
            
        public:
            
            proposal_dist (decltype(model) model) : model(model) { }
            
            using result_type = associated_type;
            static const int vector_dim = velocity_slip_model::vector_dim;
            
            auto operator() (random_source& random) const -> result_type { return inv_observe (model (random)); }
            auto likelihood (const result_type& x) const -> double { return model.likelihood (observe (x)); }
            auto log_likelihood (const result_type& x) const -> double { return model.log_likelihood (observe (x)); }
            auto initial_value (random_source&) const -> result_type { return inv_observe (model.mean()); }
        };
        
        auto proposal () const -> proposal_dist {
            return { *this };
        }
        
	class builder : public std::unary_function<vector_type, velocity_slip_model> {
            
            Eigen::Matrix3d mat_stddev;
            double m_dt;
            
	public:
            
            builder (double a1, double a2, double a3, double a4, double a5, double a6, double dt) : m_dt(dt) {
                mat_stddev <<
                a1, a2, 0,
                a3, a4, 0,
                a5, a6, 0;
                mat_stddev *= std::sqrt(m_dt);
            }
            
            builder (const boost::program_options::variables_map&, double dt);
            
            static auto program_options () -> boost::program_options::options_description;
            
            auto operator() (const vector_type& control) const -> velocity_slip_model {
                return { control*m_dt, mat_stddev*control.cwiseAbs()*m_dt };
            }
            
            auto from_steering (double v, double w, double g = 0.0) -> velocity_slip_model {
                return operator()({v, w, g});
            }
            
            auto dt () const -> double { return m_dt; }
	};
        
    };
    
    
    struct velocity_model : public independent_normal_base<2, velocity_model> {
        
        using base_type = independent_normal_base<2, velocity_model>;
        using associated_type = pose;
        
        velocity_model () { }
        
        velocity_model (const vector_type& mean, const vector_type& stddev) : base_type(mean, stddev) { }
        
        static auto subtract (const vector_type& a, const vector_type& b) -> vector_type {
            return a - b;
        }
        
        static auto observe (const associated_type& dp) -> vector_type {            
            auto observation = velocity_slip_model::observe (dp);
            return { observation(0), observation(1) };
        }
        
	static auto inv_observe (const vector_type& control) -> associated_type {
            return velocity_slip_model::inv_observe ({ control(0), control(1), 0.0 });
	}
        
        class proposal_dist {
            
            const velocity_model& model;
            
        public:
            
            using result_type = associated_type;
            static const int vector_dim = velocity_model::vector_dim;
            
            proposal_dist (decltype(model) model) : model(model) { }
            auto operator() (random_source& random) const -> result_type { return inv_observe (model (random)); }
            auto likelihood (const result_type& x) const -> double { return model.likelihood (observe (x)); }
            auto log_likelihood (const result_type& x) const -> double { return model.log_likelihood (observe (x)); }
            auto initial_value (random_source&) const -> result_type { return inv_observe (model.mean()); }
        };
        
        auto proposal () const -> proposal_dist {
            return { *this };
        }
        
        class builder {
            
            Eigen::Matrix2d mat_stddev;
            double m_dt;
            
        public:
            
            builder (double a1, double a2, double a3, double a4, double dt) : m_dt(dt) {
                mat_stddev <<
                a1, a2,
                a3, a4;
                mat_stddev *= std::sqrt(m_dt);
            }
            
            builder (const boost::program_options::variables_map&, double dt);
            
            static auto program_options () -> boost::program_options::options_description;

            auto operator() (const vector_type& control) const -> velocity_model {
                return { control*m_dt, mat_stddev*control.cwiseAbs()*m_dt };
            }
            
            auto from_steering (double v, double w) -> velocity_model {
                return operator()({v, w});
            }
            
            auto dt () const -> double { return m_dt; }
        };
        
    };
    
    
} // namespace planar_robot

#endif //_PLANAR_ROBOT_VELOCITY_MODEL_HPP
