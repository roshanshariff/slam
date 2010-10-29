#ifndef _PLANAR_ROBOT_ODOMETRY_MODEL_HPP
#define _PLANAR_ROBOT_ODOMETRY_MODEL_HPP

#include <cstdlib>
#include <functional>

#include "planar_robot/pose.hpp"
#include "utilities/random.hpp"
#include "utilities/geometry.hpp"


namespace planar_robot {


  class odometry_model {

    normal_dist<double> distance;
    normal_dist<double> direction;
    normal_dist<double> bearing;

  public:

    typedef pose result_type;

    odometry_model (const pose& mean, double distance_sigma,
	       double direction_sigma, double bearing_sigma)
      : distance (mean.distance(), distance_sigma),
	direction (mean.direction(), direction_sigma),
	bearing (mean.bearing(), bearing_sigma) { }

    pose mean () const {
      return pose_polar (distance.mean(), direction.mean(), bearing.mean());
    }

    pose operator() (random_source& random) const {
      return pose_polar (distance(random), direction(random), bearing(random));
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

    class builder : public std::binary_function<double, pose, odometry_model> {

      double alpha1, alpha2, alpha3, alpha4;

    public:

      builder (double a1, double a2, double a3, double a4)
	: alpha1(a1), alpha2(a2), alpha3(a3), alpha4(a4) { }

      odometry_model operator() (double dt, const pose& reading) const {

	double translation = reading.distance();
	double direction = reading.direction();
	double bearing = reading.bearing();
	double rotation = wrap_angle(bearing - direction);

	double direction_sigma = alpha1 * std::abs(direction) + alpha2 * translation;
	double distance_sigma =  alpha3 * translation         + alpha4 * std::abs(bearing);
	double bearing_sigma =   alpha1 * std::abs(rotation)  + alpha2 * translation;

	return odometry_model (reading, distance_sigma, direction_sigma, bearing_sigma);
      };

    };

  };


} // namespace planar_robot

#endif //_PLANAR_ROBOT_ODOMETRY_MODEL_HPP
