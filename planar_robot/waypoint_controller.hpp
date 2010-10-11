#ifndef _PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
#define _PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP

#include <vector>
#include <cstdlib>
#include <functional>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/velocity_model.hpp"
#include "utilities/bitree.hpp"


namespace planar_robot {


  class waypoint_controller : public std::binary_function<double, bitree<pose>, velocity_control> {

    pose initial;
    std::vector<position> waypoints;
    double proximity, repetitions;
    double speed, steering_max, steering_rate;

    double current_steering;
    size_t current_waypoint;

  public:

    waypoint_controller (const pose& _initial, const std::vector<position>& _waypoints,
			 double _proximity, double _repetitions, double _speed,
			 double _steering_max, double _steering_rate)
      : initial(_initial), waypoints(_waypoints), proximity(_proximity), repetitions(_repetitions),
	speed(_speed), steering_max(_steering_max), steering_rate(_steering_rate),
	current_steering(0.0), current_waypoint(0) { }

    pose initial_pose () const {
      return initial;
    }

    bool finished () const {
      return current_waypoint > waypoints.size() * repetitions;
    }

    velocity_control operator() (double dt, const bitree<pose>& trajectory) {

      if (!waypoints.empty()) {

	position to_waypoint = -(initial_pose() + trajectory.accumulate())
	  + waypoints[current_waypoint % waypoints.size()];

	double steering_change = to_waypoint.bearing()/dt - current_steering;
	double steering_change_max = steering_rate * dt;
	if (std::abs(steering_change) > steering_change_max) {
	  steering_change = steering_change < 0 ? -steering_change_max : steering_change_max;
	}

	current_steering += steering_change;
	if (std::abs(current_steering) > steering_max) {
	  current_steering = current_steering < 0 ? -steering_max : steering_max;
	}

	if (to_waypoint.range() < proximity) {
	  ++current_waypoint;
	  //std::cout << "Seeking waypoint " << current_waypoint << '\n';
	}

      }

      return velocity_control (speed, current_steering);
    }

  };


} // namespace planar_robot

#endif //_PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
