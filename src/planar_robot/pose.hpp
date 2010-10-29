#ifndef _PLANAR_ROBOT_POSE_HPP
#define _PLANAR_ROBOT_POSE_HPP

#include <complex>


namespace planar_robot {


  // Forward declarations
  class pose;
  class position;
  inline position operator+ (const pose&, const position&);


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
    double distance_squared () const { return std::norm(translation); }

    pose& operator+= (const pose& p) {
      translation += rotation * p.translation;
      rotation *= p.rotation;
      return *this;
    }

    pose operator- () const {
      std::complex<double> inverse_rot = std::complex<double>(1.0)/rotation;
      return pose (-translation*inverse_rot, inverse_rot);
    }

    friend pose pose_polar (double r, double theta, double bearing);
    friend position operator+ (const pose&, const position&);

  };


  inline pose pose_polar (double r, double theta, double bearing) {
    return pose (std::polar (r, theta), std::polar (1.0, bearing));
  }


  inline pose operator+ (pose a, const pose& b) { return a += b; }


} // namespace planar_robot

#endif //_PLANAR_ROBOT_POSE_HPP
