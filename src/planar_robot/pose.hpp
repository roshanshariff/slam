#ifndef _PLANAR_ROBOT_POSE_HPP
#define _PLANAR_ROBOT_POSE_HPP

#include <complex>

#include <Eigen/Core>

#include "planar_robot/position.hpp"
#include "utility/geometry.hpp"

namespace planar_robot {
    
    
    inline position operator+ (const pose&, const position&); // Forward declaration
    
    
    class pose {
        
	std::complex<double> translation;
	std::complex<double> rotation;
        
    protected:
        
	pose (const std::complex<double>& trans, const std::complex<double>& rot)
	: translation(trans), rotation(rot) { }
        
    public:
        
	static const int vector_dim = 3;
	using vector_type = Eigen::Vector3d;
        
	pose () : translation(), rotation(1) { }
        
	static pose cartesian (double x, double y, double bearing) {
            return pose (std::complex<double>(x,y), std::polar(1.0, bearing));
	}
        
	static pose polar (double distance, double direction, double bearing) {
            return pose (std::polar(distance, direction), std::polar(1.0, bearing));
	}
        
	static pose from_position (const position& position, double bearing) {
            return pose (position.pos, std::polar(1.0, bearing));
	}
        
	double x () const { return translation.real(); }
	double y () const { return translation.imag(); }
	double bearing () const { return std::arg(rotation); }
	double distance () const { return std::abs(translation); }
	double direction () const { return std::arg(translation); }
	double distance_squared () const { return std::norm(translation); }
        
	vector_type to_vector () const { return vector_type(x(), y(), bearing()); }
        
	static pose from_vector (const vector_type& v) { return cartesian (v(0), v(1), v(2)); }
        
        static vector_type subtract (const vector_type& a, const vector_type& b) {
            return vector_type (a(0)-b(0), a(1)-b(1), wrap_angle(a(2)-b(2)));
        }
        
	pose& operator+= (const pose& p) {
            translation += rotation * p.translation;
            rotation *= p.rotation;
            return *this;
	}
        
	pose operator- () const {
            std::complex<double> inverse_rot = std::complex<double>(1.0)/rotation;
            return pose (-translation*inverse_rot, inverse_rot);
	}
        
	friend position operator+ (const pose&, const position&);
        
    };
    
    inline pose operator+ (pose a, const pose& b) { return a += b; }
    
    inline position operator+ (const pose& p, const position& o) {
	return position (p.translation + p.rotation*o.pos);
    }
    
    
} // namespace planar_robot

#endif //_PLANAR_ROBOT_POSE_HPP
