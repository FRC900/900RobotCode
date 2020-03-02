#ifndef PARTICLE_HEADER
#define PARTICLE_HEADER

struct Particle {
  double x_;
  double y_;
  double rot_;
  double weight_;

  Particle(double x, double y, double rot): x_(x), y_(y), rot_(rot), weight_(1) {}
};

#endif
