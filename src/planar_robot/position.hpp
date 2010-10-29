#ifndef _PLANAR_ROBOT_POSITION_HPP
#define _PLANAR_ROBOT_POSITION_HPP

#include <complex>

#include "planar_robot/pose.hpp"


namespace planar_robot {


  class position {

    std::complex<double> pos;

    position (const std::complex<double>& p) : pos(p) { }

  public:

    position (double x = 0.0, double y = 0.0) : pos(x, y) { }

    double x () const { return pos.real(); }
    double y () const { return pos.imag(); }
    double range () const { return std::abs(pos); }
    double bearing () const { return std::arg(pos); }
    double range_squared () const { return std::norm(pos); }

    friend position position_polar (double range, double bearing);
    friend position operator+ (const pose&, const position&);

  };


  inline position position_polar (double range, double bearing) {
    return position (std::polar (range, bearing));
  }


  inline position operator+ (const pose& p, const position& o) {
    return position (p.translation + p.rotation*o.pos);
  }


} // namespace planar_robot

#endif //_PLANAR_ROBOT_POSITION_HPP
