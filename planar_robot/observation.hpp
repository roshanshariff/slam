#include <complex>

#include "pose.hpp"

class observation {

  using std::complex;

  complex<double> position;

  observation (const complex<double>& p) : position(p) { }

public:

  observation (double range, double bearing)
    : position(std::polar(range, bearing)) { }

  double x () const       { return position.real(); }
  double y () const       { return position.imag(); }
  double range () const   { return std::abs(position); }
  double bearing () const { return std::arg(position); }

  observation& operator-= (const pose& p) {
    position -= p.translation * conj(p.rotation);
  }

};

inline observation operator+ (const pose& p, const observation& o) {
  std::complex<double position = p.translation + p.rotation*o.position;
  return observation(position);
}
