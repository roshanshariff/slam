#ifndef _PLANAR_ROBOT_POSE_HPP
#define _PLANAR_ROBOT_POSE_HPP

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "planar_robot/position.hpp"
#include "utility/geometry.hpp"

namespace planar_robot {
    
    
    inline position operator+ (const pose&, const position&); // Forward declaration
    
    
    class pose {
        
        Eigen::Vector2d translation;
        Eigen::Rotation2Dd rotation;
        
    protected:
        
	pose (const Eigen::Vector2d& trans, const Eigen::Rotation2Dd& rot)
	: translation(trans), rotation(rot) { }
        
    public:
        
	static const int vector_dim = 3;
	using vector_type = Eigen::Vector3d;
        
	pose () : pose({ 0.0, 0.0 }, 0.0) { }
        
	static pose cartesian (double x, double y, double bearing) {
            return pose ({ x, y }, bearing);
	}
        
	static pose polar (double dist, double dir, double bearing) {
            return pose ({ dist*std::cos(dir), dist*std::sin(dir) }, bearing);
	}
        
	static pose from_position (const position& position, double bearing) {
            return pose (position.pos, bearing);
	}
        
        static pose from_trans_rot (const Eigen::Vector2d& trans, const Eigen::Rotation2Dd& rot) {
            return pose (trans, rot);
        }
        
	double x () const { return translation.x(); }
	double y () const { return translation.y(); }
	double bearing () const { return rotation.angle(); }
	double distance () const { return translation.norm(); }
	double direction () const { return std::atan2 (y(), x()); }
	double distance_squared () const { return translation.squaredNorm(); }
        
	vector_type to_vector () const { return { x(), y(), bearing() }; }
        
	static pose from_vector (const vector_type& v) { return cartesian (v(0), v(1), v(2)); }
        
        static vector_type subtract (const vector_type& a, const vector_type& b) {
            return { a(0)-b(0), a(1)-b(1), wrap_angle(a(2)-b(2)) };
        }
        
	pose& operator+= (const pose& p) {
            translation += rotation * p.translation;
            rotation *= p.rotation;
            return *this;
	}
        
	pose operator- () const {
            Eigen::Rotation2Dd inverse_rot = rotation.inverse();
            return { inverse_rot*(-translation), inverse_rot };
	}
        
	friend position operator+ (const pose&, const position&);
        
    };
    
    inline pose operator+ (pose a, const pose& b) { return a += b; }
    
    inline position operator+ (const pose& p, const position& o) {
	return position (p.translation + p.rotation*o.pos);
    }
    
    
} // namespace planar_robot

#endif //_PLANAR_ROBOT_POSE_HPP
