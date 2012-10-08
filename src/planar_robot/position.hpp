#ifndef _PLANAR_ROBOT_POSITION_HPP
#define _PLANAR_ROBOT_POSITION_HPP

#include <cmath>

#include <Eigen/Core>


namespace planar_robot {
    
    
    class pose; // Forward declaration
    
    
    class position {
        
        Eigen::Vector2d pos;
        
    protected:
        
	explicit position (const Eigen::Vector2d& p) : pos(p) { }
        
    public:
        
	static const int vector_dim = 2;
	using vector_type = Eigen::Vector2d;
        
        position () : position({ 0.0, 0.0 }) { }
        
	static position cartesian (double x, double y) {
            return position ({ x, y });
	}
        
	static position polar (double dist, double dir) {
            return position ({ dist*std::cos(dir), dist*std::sin(dir) });
	}
        
	double x () const { return pos.x(); }
	double y () const { return pos.y(); }
	double distance () const { return pos.norm(); }
	double direction () const { return std::atan2 (y(), x()); }
	double distance_squared () const { return pos.squaredNorm(); }
        
	vector_type to_vector () const { return pos; }
        
	static position from_vector (const vector_type& v) { return position (v); }
        
        static vector_type subtract (const vector_type& a, const vector_type& b) { return a - b; }
        
	friend class pose;
	friend position operator+ (const pose&, const position&);
        
    };
    
    
} // namespace planar_robot

#endif //_PLANAR_ROBOT_POSITION_HPP
