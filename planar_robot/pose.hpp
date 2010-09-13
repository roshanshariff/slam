#include <complex>

class pose {

  using std::complex;

  complex<double> translation;
  complex<double> rotation;

  pose (const complex<double>& t, const complex<double>& r)
    : translation(t), rotation(r) { }

public:
  
  pose () : translation(), rotation(1) { }

  pose (double x = 0, double y = 0, double bearing = 0)
    : translation(x,y), rotation(std::polar(1.0, bearing)) { }

  double x () const       { return translation.real(); }
  double y () const       { return translation.imag(); }
  double bearing () const { return std::arg(rotation); }
  double distance () const { return std::abs(translation); }
  double direction () const { return std::arg(translation); }

  pose& operator+= (const pose& p) {
    translation += rotation * p.translation;
    rotation *= p.rotation;
    return *this;
  }

  pose operator- () const {
    complex<double> inverse_rot = conj(rotation);
    return pose (-translation*inverse_rot, inverse_rot);
  }

  friend class observation;
  friend observation operator+ (const pose&, const observation&);

};

inline pose operator+ (pose a, const pose& b) { return a += b; }
