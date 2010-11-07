#ifndef _PLANAR_ROBOT_CIRCLE_CONTROLLER_HPP
#define _PLANAR_ROBOT_CIRCLE_CONTROLLER_HPP

#include <functional>

#include "planar_robot/pose.hpp"
#include "utility/bitree.hpp"


namespace planar_robot {


  class circle_controller : public std::binary_function<double, bitree<pose>, pose> {

    double centre_x, centre_y, radius, speed, revolutions;
    double current_time;
    pose current_pose;

    pose get_pose (double time) const {
      pose result = pose (centre_x, centre_y, speed * time / radius);
      result += pose (radius, 0, boost::math::constants::pi<double>()/2);
      return result;
    }

  public:

    circle_controller (double cx, double cy, double r, double v, double revs)
      : centre_x(cx), centre_y(cy), radius(r), speed(v), revolutions(revs),
	current_time(0.0), current_pose(get_pose(current_time)) { }

    pose operator() (double dt, const bitree<pose>&) {
      pose old_pose = current_pose;
      current_pose = get_pose (current_time += dt);
      return -old_pose + current_pose;
    }

    pose initial_pose () const {
      return get_pose (0.0);
    }

    bool finished () const {
      return speed*current_time >= revolutions * 2*boost::math::constants::pi<double>()*radius;
    }

  };


} // namespace planar_robot

#endif //_PLANAR_ROBOT_CIRCLE_CONTROLLER_HPP
