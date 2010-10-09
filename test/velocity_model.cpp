#include <iostream>

#include "planar_robot/pose.hpp"
#include "planar_robot/velocity_model.hpp"

using namespace planar_robot;

int main () {

  velocity_control c (3.0, 0.0, 1.0);
  pose p = velocity_to_pose_delta (c);
  velocity_control c2 = pose_delta_to_velocity (p);

  std::cout << "Control: v = " << c.v() << ", w = " << c.w() << ", g = " << c.g() << '\n';
  std::cout << "Pose: x = " << p.x() << ", y = " << p.y() << ", theta = " << p.bearing() << '\n';
  std::cout << "Control: v = " << c2.v() << ", w = " << c2.w() << ", g = " << c2.g() << '\n';

}
