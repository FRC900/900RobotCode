#ifndef PARTICLE_HEADER
#define PARTICLE_HEADER
#include <cmath>
#include <ostream>
// #define CHECK_NAN
struct Particle {
  double x_;
  double y_;
  double rot_;
  double weight_;

  Particle(double x, double y, double rot): x_(x), y_(y), rot_(rot), weight_(1) {}
  Particle() : x_(0), y_(0), rot_(0), weight_(1) {}
  bool isValid(void) const
  {
	  const double sum = x_ + y_ + rot_ + weight_;
	  return !std::isnan(sum) && !std::isinf(sum);
  }
};

std::ostream& operator<<(std::ostream& os, const Particle &p);

#endif
