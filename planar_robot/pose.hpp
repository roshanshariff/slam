#ifndef _PLANAR_ROBOT_POSE_HPP
#define _PLANAR_ROBOT_POSE_HPP

#include <complex>

#include "utilities/random.hpp"
#include "utilities/geometry.hpp"

namespace planar_robot {

  // Forward declarations
  class pose;
  class observation;
  inline observation operator+ (const pose&, const observation&);

  class pose {

    std::complex<double> translation;
    std::complex<double> rotation;

    pose (const std::complex<double>& t, const std::complex<double>& r)
      : translation(t), rotation(r) { }

  public:
  
    pose () : translation(), rotation(1) { }

    pose (double x, double y, double bearing = 0)
      : translation(x,y), rotation(std::polar(1.0, bearing)) { }

    double x () const { return translation.real(); }
    double y () const { return translation.imag(); }
    double bearing () const { return std::arg(rotation); }
    double distance () const { return std::abs(translation); }
    double direction () const { return std::arg(translation); }

    pose& operator+= (const pose& p) {
      translation += rotation * p.translation;
      rotation *= p.rotation;
      return *this;
    }

    pose operator- () const {
      std::complex<double> inverse_rot = std::complex<double>(1.0)/rotation;
      return pose (-translation*inverse_rot, inverse_rot);
    }

    friend class pose_dist;

    friend observation operator+ (const pose&, const observation&);

  };

  inline pose operator+ (pose a, const pose& b) { return a += b; }


  class pose_dist {

    normal_dist<double> distance;
    normal_dist<double> direction;
    normal_dist<double> bearing;

  public:

    typedef pose result_type;

    pose_dist (const pose& mean, double distance_sigma,
	       double direction_sigma, double bearing_sigma)
      : distance (mean.distance(), distance_sigma),
	direction (mean.direction(), direction_sigma),
	bearing (mean.bearing(), bearing_sigma) { }

    pose mean () const {
      return pose (std::polar (distance.mean(), direction.mean()),
		   std::polar (1.0, bearing.mean()));
    }

    pose operator() (random_source& random) const {
      return pose (std::polar (distance(random), direction(random)),
		   std::polar (1.0, bearing(random)));
    }

    double likelihood (const pose& p) const {
      return distance.likelihood (p.distance())
	* direction.likelihood (wrap_angle (p.direction(), direction.mean()))
	* bearing.likelihood (wrap_angle (p.bearing(), bearing.mean()));
    }

    double log_likelihood (const pose& p) const {
      return distance.log_likelihood (p.distance())
	+ direction.log_likelihood (wrap_angle (p.direction(), direction.mean()))
	+ bearing.log_likelihood (wrap_angle (p.bearing(), bearing.mean()));
    }

  };

  
  class pose_dist_odometry {
    const double alpha1, alpha2, alpha3, alpha4;
  public:
    pose_dist_odometry (double a1, double a2, double a3, double a4)
      : alpha1(a1), alpha2(a2), alpha3(a3), alpha4(a4) { }
    pose_dist operator() (const pose& reading) const {
      double translation = reading.distance();
      double direction = reading.direction();
      double bearing = reading.bearing();
      double rotation = wrap_angle(bearing - direction);
      double direction_sigma = alpha1*std::abs(direction) + alpha2*translation;
      double distance_sigma = alpha3*translation + alpha4*std::abs(bearing);
      double bearing_sigma = alpha1*std::abs(rotation) + alpha2*translation;
      return pose_dist (reading, distance_sigma, direction_sigma, bearing_sigma);
    }
  };

}

#endif //_PLANAR_ROBOT_POSE_HPP
