#ifndef _PLANAR_ROBOT_POSITION_HPP
#define _PLANAR_ROBOT_POSITION_HPP

#include <complex>

#include <Eigen/Core>


namespace planar_robot {
    
    
    class pose; // Forward declaration
    
    
    class position {
        
	std::complex<double> pos;
        
    protected:
        
	position (const std::complex<double>& p) : pos(p) { }
        
    public:
        
	static constexpr int vector_dim = 2;
	using vector_type = Eigen::Vector2d;
        
        position () : pos() { }
        
	static position cartesian (double x, double y) {
            return position (std::complex<double> (x, y));
	}
        
	static position polar (double distance, double direction) {
            return position (std::polar (distance, direction));
	}
        
	double x () const { return pos.real(); }
	double y () const { return pos.imag(); }
	double distance () const { return std::abs(pos); }
	double direction () const { return std::arg(pos); }
	double distance_squared () const { return std::norm(pos); }
        
	vector_type to_vector () const { return vector_type (x(), y()); }
        
	static position from_vector (const vector_type& v) { return cartesian (v(0), v(1)); }
        
        static vector_type subtract (const vector_type& a, const vector_type& b) { return a - b; }
        
	friend class pose;
	friend position operator+ (const pose&, const position&);
        
    };
    
    
} // namespace planar_robot

#endif //_PLANAR_ROBOT_POSITION_HPP
