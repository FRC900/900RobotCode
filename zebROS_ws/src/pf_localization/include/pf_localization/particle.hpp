#ifndef PARTICLE_HEADER
#define PARTICLE_HEADER
#include <cmath>
#include <ostream>
// #define CHECK_NAN
struct Particle {
  double x_;
  double y_;
  double rot_;
  double weight_{1.0};

  Particle(double x, double y, double rot): x_(x), y_(y), rot_(rot) {}
  Particle() : x_(0), y_(0), rot_(0) {}
  bool isValid(void) const
  {
	  const double sum = x_ + y_ + rot_ + weight_;
	  return std::isfinite(sum);
  }
  bool operator<(const Particle &rhs) const {
    if (x_ < rhs.x_) {
      return true;
    }
    if (x_ == rhs.x_) {
      if (y_ < rhs.y_) {
        return true;
      }
      if ((y_ == rhs.y_) && (rot_< rhs.rot_)) {
        return true;
      }
    }
    return false;
  }

  friend std::ostream &operator<<(std::ostream &os, const Particle &p)
  {
    os << "Particle(" << p.x_ << ", " << p.y_ << ", " << p.rot_ << ", " << p.weight_ << ")";
    return os;
  }
};


#endif
