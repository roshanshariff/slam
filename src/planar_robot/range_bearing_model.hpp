#ifndef _PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP
#define _PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP

#include <cmath>
#include <functional>

#include <boost/program_options.hpp>

#include "planar_robot/position.hpp"
#include "utility/random.hpp"
#include "utility/geometry.hpp"


namespace planar_robot {
    
    struct range_bearing_model : public independent_normal_base<2, range_bearing_model> {
        
	using base_type = independent_normal_base<2, range_bearing_model>;
	using vector_type = base_type::vector_type;
        using matrix_type = base_type::matrix_type;
        
	using associated_type = position;
        
	range_bearing_model () = default;
        
	range_bearing_model (const vector_type& mean, const vector_type& stddev) : base_type(mean, stddev) { }
        
	static auto subtract (const vector_type& a, const vector_type& b) -> vector_type {
            return vector_type (a(0)-b(0), wrap_angle(a(1)-b(1)));
	}
        
	static auto to_vector (const associated_type& pos) -> vector_type {
            return vector_type (pos.distance(), pos.direction());
	}
        
	static auto from_vector (const vector_type& observation) -> associated_type {
            return position::polar (observation(0), observation(1));
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
    
    
    struct range_only_model : public range_bearing_model {
        
        range_only_model () = default;
        
        range_only_model (const vector_type& mean, const vector_type& stddev) : range_bearing_model(mean, stddev) { }
        
        auto operator() (random_source& random) const -> vector_type;
        auto likelihood (const vector_type& x) const -> double;
        auto log_likelihood (const vector_type& x) const -> double;
        
        class builder : public std::unary_function<vector_type, range_only_model> {
            
            const vector_type stddev;
            
        public:
            
            builder (double range_stddev) : stddev(range_stddev, 2*boost::math::constants::pi<double>()/std::sqrt(12)) { }
            
            builder (const boost::program_options::variables_map&);
            
            static auto program_options () -> boost::program_options::options_description;
            
            range_only_model operator() (const vector_type& observation) const {
                return range_only_model (observation, stddev);
            }
            
        };
        
    };
    

} // namespace planar_robot

#endif //_PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP
